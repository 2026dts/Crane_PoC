#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>

// ==========================================
// 1. Network & ThingsBoard Configuration
// ==========================================
const char* ssid = "DTS_Chennai_2.4";
const char* password = "Dts@2025";

const char* mqtt_server = "10.10.10.52"; 
const int mqtt_port = 1883;

const char* mqtt_user = "hc8OBlRQv2H2WvQ4kKTo"; 
const char* mqtt_password = ""; 

const char* pub_topic = "v1/devices/me/telemetry";
const char* rpc_sub_topic = "v1/devices/me/rpc/request/+";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ==========================================
// 2. Hardware Pins Configuration
// ==========================================
#define ACS712_PIN    34 
#define VOLTAGE_PIN   35 
const int MPU_ADDR = 0x68; 
#define SDA_PIN 21
#define SCL_PIN 22

#define MOTOR_IN1     25
#define MOTOR_IN2     26

unsigned long lastMsgTime = 0;
const long interval = 2000;

// ==========================================
// 3. Sensor Calibration & Variables
// ==========================================
const float ACS_SENSITIVITY = 0.185; 
float acs_offset_V = 0.0;      
const float ACS_DIVIDER_MULTIPLIER = 2.0; 
const float VOLTAGE_DIVIDER_RATIO = 5.0; 

// ---- MPU6500 orientation state ----
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float accelX_error = 0.0, accelY_error = 0.0, accelZ_error = 0.0;
float gyroX_error  = 0.0, gyroY_error  = 0.0, gyroZ_error  = 0.0;
float gyroZ_stddev = 0.0;
float GYRO_Z_DEADBAND = 0.30; // auto-computed after calibration

// Stationary detection (for auto re-bias / drift correction while idle)
const float STATIONARY_ACCEL_TOL   = 0.03;   // g, how close |a| must be to 1g
const float STATIONARY_GYRO_TOL    = 0.5;    // deg/s, how still gyro must be
unsigned long stationarySinceMillis = 0;
const unsigned long STATIONARY_REBIAS_MS = 4000; // must be still this long before rebias
bool isRebiasing = false;

unsigned long lastTime = 0;

Preferences preferences;
const float POWER_THRESHOLD_W = 0.5;

float bootStartTotalTime_sec = 0.0;
float currentSessionTime_sec = 0.0;
float totalOperationTime_sec = 0.0;

unsigned long lastRuntimeCheckMillis = 0;
unsigned long lastFlashSaveMillis = 0;
const unsigned long FLASH_SAVE_INTERVAL = 5000; // changed from 30000 -> 5000 (5 sec)

float baseline_pitch = 0.0;
float jib_deflection_deg = 0.0;
bool baselineCaptured = false;

const float STRESS_MEDIUM_DEFLECTION_DEG = 3.0;
const float STRESS_HIGH_DEFLECTION_DEG   = 6.0;
String stress_trend = "low";

// ==========================================
// 3E. Motor Control Functions
// ==========================================
void motorForward() {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    Serial.println(">>> [MOTOR ACTION] FORWARD executed <<<");
}

void motorReverse() {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    Serial.println(">>> [MOTOR ACTION] REVERSE executed <<<");
}

void motorStop() {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    Serial.println(">>> [MOTOR ACTION] STOP executed <<<");
}

// ==========================================
// MPU6500 Low-level read helper
// ==========================================
void readRawIMU(int16_t &ax, int16_t &ay, int16_t &az,
                 int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// ==========================================
// MPU6500 Calibration routine
// ==========================================
void calibrateMPU() {
  Serial.println("\n================================================");
  Serial.println("Calibrating MPU6500. DO NOT MOVE / TOUCH THE SENSOR.");
  Serial.println("Keep it perfectly still on a flat, level surface.");
  Serial.println("================================================");
  delay(2000);

  const int N = 1000; // more samples for stability
  long aX_sum = 0, aY_sum = 0, aZ_sum = 0;
  double gX_sum = 0, gY_sum = 0, gZ_sum = 0;
  double gZ_sq_sum = 0; // for std-dev

  for (int i = 0; i < N; i++) {
    int16_t rAx, rAy, rAz, rGx, rGy, rGz;
    readRawIMU(rAx, rAy, rAz, rGx, rGy, rGz);

    aX_sum += rAx;
    aY_sum += rAy;
    aZ_sum += rAz;

    float gz_dps = (float)rGz / 131.0;
    gX_sum += (float)rGx / 131.0;
    gY_sum += (float)rGy / 131.0;
    gZ_sum += gz_dps;
    gZ_sq_sum += (double)gz_dps * gz_dps;

    delay(3);
  }

  accelX_error = aX_sum / (float)N;
  accelY_error = aY_sum / (float)N;
  accelZ_error = (aZ_sum / (float)N) - 16384.0; // 1g offset at rest

  gyroX_error = gX_sum / N;
  gyroY_error = gY_sum / N;
  gyroZ_error = gZ_sum / N;

  double meanGz = gZ_sum / N;
  double variance = (gZ_sq_sum / N) - (meanGz * meanGz);
  gyroZ_stddev = sqrt(variance > 0 ? variance : 0);

  // Set deadband = 4x std-dev of noise, with a sane floor/ceiling
  GYRO_Z_DEADBAND = gyroZ_stddev * 4.0;
  if (GYRO_Z_DEADBAND < 0.15) GYRO_Z_DEADBAND = 0.15;
  if (GYRO_Z_DEADBAND > 1.0)  GYRO_Z_DEADBAND = 1.0;

  Serial.println("\n--- Calibration Results ---");
  Serial.print("accelX_error: "); Serial.println(accelX_error, 3);
  Serial.print("accelY_error: "); Serial.println(accelY_error, 3);
  Serial.print("accelZ_error: "); Serial.println(accelZ_error, 3);
  Serial.print("gyroX_error : "); Serial.println(gyroX_error, 4);
  Serial.print("gyroY_error : "); Serial.println(gyroY_error, 4);
  Serial.print("gyroZ_error : "); Serial.println(gyroZ_error, 4);
  Serial.print("gyroZ_stddev(noise): "); Serial.println(gyroZ_stddev, 4);
  Serial.print("Auto deadband set to: "); Serial.println(GYRO_Z_DEADBAND, 3);
  Serial.println("----------------------------\n");

  if (gyroZ_stddev > 0.5) {
    Serial.println("[WARNING] Gyro noise is high (stddev > 0.5). Sensor may be");
    Serial.println("vibrating, poorly mounted, or on a noisy power rail.");
    Serial.println("Yaw drift will be worse than normal. Check mounting/wiring.");
  }
}

// ==========================================
// 4. Setup Functions
// ==========================================
void setup_wifi() {
    delay(10);
    Serial.println("\nConnecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected.");
}

// ENHANCED: MQTT callback with full raw debug printing
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    Serial.println("========================================");
    Serial.println("[RPC] >>> MESSAGE RECEIVED <<<");
    Serial.print("[RPC] Topic: ");
    Serial.println(topic);
    Serial.print("[RPC] Raw Payload: ");
    Serial.println(msg);
    Serial.println("========================================");

    StaticJsonDocument<200> rpcDoc;
    DeserializationError err = deserializeJson(rpcDoc, msg);
    if (err) {
        Serial.print("[RPC] JSON parse FAILED: ");
        Serial.println(err.c_str());
        return;
    }

    String method = rpcDoc["method"].as<String>();
    Serial.print("[RPC] Parsed method = '");
    Serial.print(method);
    Serial.println("'");

    if (method == "setForward") {
        motorForward();
    } else if (method == "setReverse") {
        motorReverse();
    } else if (method == "setStop") {
        motorStop();
    } else {
        Serial.println("[RPC] Method did NOT match any known command!");
    }
}

void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to ThingsBoard...");
        String clientId = "ESP32Crane-";
        clientId += String(random(0xffff), HEX);
        
        if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("connected!");
            bool subOk = mqttClient.subscribe(rpc_sub_topic);
            Serial.print("[RPC] Subscribe to '");
            Serial.print(rpc_sub_topic);
            Serial.print("' -> ");
            Serial.println(subOk ? "SUCCESS" : "FAILED");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retrying in 5s");
            delay(5000);
        }
    }
}

String mqttStateToText(int state) {
    switch (state) {
        case -4: return "MQTT_CONNECTION_TIMEOUT";
        case -3: return "MQTT_CONNECTION_LOST";
        case -2: return "MQTT_CONNECT_FAILED";
        case -1: return "MQTT_DISCONNECTED";
        case  0: return "MQTT_CONNECTED";
        case  1: return "MQTT_CONNECT_BAD_PROTOCOL";
        case  2: return "MQTT_CONNECT_BAD_CLIENT_ID";
        case  3: return "MQTT_CONNECT_UNAVAILABLE";
        case  4: return "MQTT_CONNECT_BAD_CREDENTIALS";
        case  5: return "MQTT_CONNECT_UNAUTHORIZED";
        default: return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    Wire.begin(SDA_PIN, SCL_PIN);

    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);

    Serial.println("\nCalibrating ACS712...");
    float total_voltage = 0;
    for(int i = 0; i < 100; i++) {
        int acsRaw = analogRead(ACS712_PIN);
        total_voltage += ((acsRaw / 4095.0) * 3.3) * ACS_DIVIDER_MULTIPLIER;
        delay(10);
    }
    acs_offset_V = total_voltage / 100.0;

    // ---- MPU6500 wake + config (±2g, ±250dps) ----
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission(true);
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); Wire.write(0x00);
    Wire.endTransmission(true);
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00);
    Wire.endTransmission(true);
    
    delay(1000);

    // ---- Calibration routine (bias measurement, always runs at boot) ----
    calibrateMPU();

    // Initial pitch/roll from accel (still needed as starting reference)
    int16_t bAx, bAy, bAz, bGx, bGy, bGz;
    readRawIMU(bAx, bAy, bAz, bGx, bGy, bGz);
    float bax = (bAx - accelX_error) / 16384.0;
    float bay = (bAy - accelY_error) / 16384.0;
    float baz = (bAz - accelZ_error) / 16384.0;
    pitch = atan2(-bax, sqrt(bay * bay + baz * baz)) * 180.0 / PI;
    roll  = atan2(bay, baz) * 180.0 / PI;

    setup_wifi();
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);

    // ---- Load persisted values from flash (total_time AND yaw) ----
    preferences.begin("crane_data", false);
    bootStartTotalTime_sec = preferences.getFloat("total_time", 0.0);
    yaw = preferences.getFloat("last_yaw", 0.0); // load last saved yaw instead of 0
    preferences.end();

    Serial.print("Loaded total operation time from flash (sec): ");
    Serial.println(bootStartTotalTime_sec, 2);
    Serial.print("Loaded last yaw angle from flash (deg): ");
    Serial.println(yaw, 2);

    totalOperationTime_sec = bootStartTotalTime_sec;
    currentSessionTime_sec = 0.0;

    lastRuntimeCheckMillis = millis();
    lastFlashSaveMillis = millis();

    lastTime = millis();
    stationarySinceMillis = millis();
    Serial.println("System Initialized & Ready.");
}

unsigned long lastMqttCheckMillis = 0;

void loop() {
    if (millis() - lastMqttCheckMillis > 5000) {
        lastMqttCheckMillis = millis();
        if (!mqttClient.connected()) {
            Serial.print("[DEBUG] MQTT NOT connected. State: ");
            Serial.print(mqttClient.state());
            Serial.print(" (");
            Serial.print(mqttStateToText(mqttClient.state()));
            Serial.println(")");
        } else {
            Serial.println("[DEBUG] MQTT connected OK.");
        }
    }

    if (!mqttClient.connected()) { reconnect(); }
    mqttClient.loop();

    unsigned long currentTime = millis();
    
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;
    if (dt <= 0 || dt > 0.5) dt = 0.02; // guard against startup/glitch spikes

    // ---- MPU6500 read ----
    int16_t rAx, rAy, rAz, rGx, rGy, rGz;
    readRawIMU(rAx, rAy, rAz, rGx, rGy, rGz);

    float ax = (rAx - accelX_error) / 16384.0;
    float ay = (rAy - accelY_error) / 16384.0;
    float az = (rAz - accelZ_error) / 16384.0;
    float gx = ((float)rGx / 131.0) - gyroX_error;
    float gy = ((float)rGy / 131.0) - gyroY_error;
    float gz = ((float)rGz / 131.0) - gyroZ_error;

    float gz_deadbanded = gz;
    if (fabs(gz_deadbanded) < GYRO_Z_DEADBAND) gz_deadbanded = 0.0;

    // Pitch/roll: accel + gyro complementary filter (gravity reference available)
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float accelRoll  = atan2(ay, az) * 180.0 / PI;
    pitch = 0.98 * (pitch + gx * dt) + 0.02 * accelPitch;
    roll  = 0.98 * (roll + gy * dt) + 0.02 * accelRoll;

    // Yaw: pure gyro integration, continues from the flash-loaded starting value
    yaw += gz_deadbanded * dt;

    // Wrap into 0-360 range
    while (yaw < 0.0)    yaw += 360.0;
    while (yaw >= 360.0) yaw -= 360.0;

    // ---- Stationary detection for soft drift correction ----
    float accelMag = sqrt(ax * ax + ay * ay + az * az);
    bool accelStill = fabs(accelMag - 1.0) < STATIONARY_ACCEL_TOL;
    bool gyroStill  = (fabs(gx) < STATIONARY_GYRO_TOL) &&
                       (fabs(gy) < STATIONARY_GYRO_TOL) &&
                       (fabs(gz) < STATIONARY_GYRO_TOL);
    bool stationary = accelStill && gyroStill;

    if (stationary) {
        if (stationarySinceMillis == 0) stationarySinceMillis = currentTime;
        if (!isRebiasing && (currentTime - stationarySinceMillis > STATIONARY_REBIAS_MS)) {
            float rawGz = (float)rGz / 131.0;
            gyroZ_error = 0.98 * gyroZ_error + 0.02 * rawGz;
            isRebiasing = true;
        }
    } else {
        stationarySinceMillis = currentTime;
        isRebiasing = false;
    }

    jib_deflection_deg = pitch - baseline_pitch;

    if (currentTime - lastMsgTime > interval) {
        lastMsgTime = currentTime;

        if (!baselineCaptured) {
            baseline_pitch = pitch;
            jib_deflection_deg = 0.0;
            baselineCaptured = true;
            Serial.print("Deflection baseline locked at pitch (deg): ");
            Serial.println(baseline_pitch, 2);
        }
        
        float acsPinVoltage = (analogRead(ACS712_PIN) / 4095.0) * 3.3;
        float actual_V = (analogRead(VOLTAGE_PIN) / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
        
        float acsTrueVoltage = acsPinVoltage * ACS_DIVIDER_MULTIPLIER;
        float current_A = (acs_offset_V - acsTrueVoltage) / ACS_SENSITIVITY;
        if (current_A < 0.15 && current_A > -0.15) current_A = 0; 
        float power_W = actual_V * current_A;

        if (jib_deflection_deg < 0) {
            float defMagnitude = fabs(jib_deflection_deg);

            if (defMagnitude >= STRESS_HIGH_DEFLECTION_DEG) {
                stress_trend = "high";
            } else if (defMagnitude >= STRESS_MEDIUM_DEFLECTION_DEG) {
                stress_trend = "medium";
            } else {
                stress_trend = "low";
            }
        } else {
            stress_trend = "low";
        }

        unsigned long nowMillis = millis();
        float elapsedSec = (nowMillis - lastRuntimeCheckMillis) / 1000.0;
        lastRuntimeCheckMillis = nowMillis;

        if (power_W > POWER_THRESHOLD_W) {
            currentSessionTime_sec += elapsedSec;
            totalOperationTime_sec += elapsedSec;
        }

        // ---- Flash save: now every 5 sec, saves BOTH total_time and yaw ----
        if (nowMillis - lastFlashSaveMillis >= FLASH_SAVE_INTERVAL) {
            lastFlashSaveMillis = nowMillis;
            preferences.begin("crane_data", false);
            preferences.putFloat("total_time", totalOperationTime_sec);
            preferences.putFloat("last_yaw", yaw);
            preferences.end();
            Serial.print("Flash Saved -> Total Operation Time (sec): ");
            Serial.print(totalOperationTime_sec, 2);
            Serial.print(" | Yaw (deg): ");
            Serial.println(yaw, 2);
        }

        StaticJsonDocument<768> doc;
        
        doc["voltage_V"] = serialized(String(actual_V, 2));
        doc["current_A"] = serialized(String(current_A, 2));
        doc["power_W"] = serialized(String(power_W, 2));

        doc["jib_pitch_deg"] = serialized(String(pitch, 2));
        doc["jib_roll_deg"] = serialized(String(roll, 2));
        doc["jib_yaw_deg"] = serialized(String(yaw, 2));
        doc["jib_deflection_deg"] = serialized(String(jib_deflection_deg, 2));
        doc["stress_trend"] = stress_trend;

        float currentSessionTime_hr = currentSessionTime_sec / 3600.0;
        float totalOperationTime_hr = totalOperationTime_sec / 3600.0;

        doc["current_session_time_hr"] = serialized(String(currentSessionTime_hr, 4));
        doc["total_operation_time_hr"] = serialized(String(totalOperationTime_hr, 4));

        char payload[768];
        serializeJson(doc, payload);
        
        Serial.print("Publishing: ");
        Serial.println(payload);

        bool sent = mqttClient.publish(pub_topic, payload);
        Serial.print("[DEBUG] Publish status: ");
        Serial.println(sent ? "SUCCESS" : "FAILED");

        if (!sent) {
            Serial.print("[DEBUG] MQTT state at failure: ");
            Serial.print(mqttClient.state());
            Serial.print(" (");
            Serial.print(mqttStateToText(mqttClient.state()));
            Serial.println(")");
        }
    }

    delay(20);
}