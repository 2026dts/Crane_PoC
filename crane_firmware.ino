#include <esp_task_wdt.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include <INA226_WE.h>
#include <HTTPClient.h>
#include <Update.h>
#include <esp_ota_ops.h>

// ==========================================
// 1. Network & ThingsBoard Configuration
// ==========================================
// Always use the public Dynamic DNS domain for remote Dubai operation
constexpr char mqtt_server[]   = "allcad-chennai.selfip.com";
constexpr int  mqtt_port       = 1883; 
constexpr char mqtt_user[]     = "XdT60OxOOSM06LuOej8r"; 
constexpr char mqtt_password[] = "";

constexpr char pub_topic[]     = "v1/devices/me/telemetry";
constexpr char rpc_sub_topic[] = "v1/devices/me/rpc/request/+";

// --- OTA firmware update configuration ---
// ThingsBoard's HTTP API port - confirmed as 8081 for this instance
// (same port the ThingsBoard UI is served on at allcad-chennai.selfip.com:8081/home).
constexpr int OTA_HTTP_PORT = 8081;

// Bump this string on every release you flash. ThingsBoard compares this
// against the fw_version shared attribute of whatever package you assign
// to the device/profile - if they differ, an update is triggered.
constexpr char FW_TITLE[]   = "CraneStaticModelFirmware";
constexpr char FW_VERSION[] = "1.0.1 - OTA test";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WiFiMulti wifiMulti;

// ==========================================
// 2. Hardware Pins & Addresses
// ==========================================
constexpr uint8_t SDA_PIN     = 21;
constexpr uint8_t SCL_PIN     = 22;

// BTS7960 Motor Driver Pins
constexpr uint8_t MOTOR_RPWM  = 25; 
constexpr uint8_t MOTOR_LPWM  = 26; 

// 5V Relay Pin for Electromagnet
constexpr uint8_t MAGNET_RELAY_PIN = 33; 

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

constexpr int PWM_FREQ        = 1000; 
constexpr int PWM_RESOLUTION  = 8;    
int MOTOR_RUN_SPEED           = 30; // Default speed (0 - 255), updatable via ThingsBoard

// INA226 Configuration
constexpr uint8_t INA226_I2C_ADDR = 0x40;
INA226_WE ina226 = INA226_WE(INA226_I2C_ADDR);
constexpr float INA226_SHUNT_OHMS   = 0.1;   
constexpr float INA226_MAX_CURRENT  = 5.0;   

// Overcurrent protection - motor stops if current draw exceeds this
constexpr float OVERCURRENT_THRESHOLD_MA = 20.0;

// HP5883 Magnetometer Configuration
constexpr uint8_t MAG_ADDR = 0x2C;
constexpr float MAG_OFFSET_X = -341.0;
constexpr float MAG_OFFSET_Y = 39.5;
constexpr float MAG_OFFSET_Z = -83.5;

// MPU6500 Accelerometer Configuration (YOUR CALIBRATED OFFSETS)
constexpr uint8_t MPU6500_ADDR   = 0x68;
constexpr float ACCEL_OFFSET_X   = 1267.73;
constexpr float ACCEL_OFFSET_Y   = -758.88;
constexpr float ACCEL_OFFSET_Z   = 1235.66;

// ==========================================
// 3. Sensor & System Variables
// ==========================================
float jib_yaw_deg        = 0.0;
float current_pitch_deg  = 0.0;
float baseline_pitch_deg = 0.0;
float jib_deflection_deg = 0.0;
bool baselineCaptured    = false;

// --- Sensor health flags so bad/missing sensors don't silently
// publish plausible-looking garbage ---
bool mpuPresent    = false;
bool magPresent    = false;
bool ina226Present = false;

unsigned long lastMsgTime = 0;
constexpr unsigned long TELEMETRY_INTERVAL = 2000;

Preferences preferences;

float bootStartTotalTime_sec = 0.0;
float currentSessionTime_sec = 0.0;
float totalOperationTime_sec = 0.0;

unsigned long lastRuntimeCheckMillis = 0;
unsigned long lastFlashSaveMillis = 0;
constexpr unsigned long FLASH_SAVE_INTERVAL = 60000; 
float last_saved_time = 0.0;

float current_A = 0.0;
float actual_V  = 0.0;
float power_W   = 0.0;

unsigned long lastReconnectAttempt = 0;

// --- WiFi non-blocking reconnect state ---
unsigned long lastWifiAttemptMillis = 0;
constexpr unsigned long WIFI_RETRY_INTERVAL = 5000; // try every 5s while down

// --- MQTT connect timing guard so a slow/blocking connect()
// can't eat the whole watchdog window unexpectedly ---
constexpr unsigned long MQTT_RECONNECT_INTERVAL = 5000;

// --- Throttled re-probe intervals for MPU6500 and HP5883, mirroring
// the INA226_CHECK_INTERVAL pattern below, so a sensor that drops out
// (bad connection, brownout, cable wiggle) can recover without a reboot ---
constexpr unsigned long MPU_CHECK_INTERVAL = 5000;
constexpr unsigned long MAG_CHECK_INTERVAL = 5000;
unsigned long lastMpuCheckMillis = 0;
unsigned long lastMagCheckMillis = 0;

// ==========================================
// MPU6500 Accelerometer (Deflection Angle)
// ==========================================
void initMPU6500() {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0x00); // Wake up MPU6500
    if (Wire.endTransmission() == 0) {
        mpuPresent = true;
        Serial.println("[MPU] MPU6500 Initialized.");
    } else {
        mpuPresent = false;
        Serial.println("[MPU] MPU6500 Not Found! Check Address (0x68). Deflection readings disabled.");
    }
}

void updateDeflection() {
    // Throttled re-probe so a sensor that comes back after a drop
    // (loose cable, brief brownout) is picked back up without a reboot -
    // same pattern as the INA226 recheck in readPowerSensors().
    unsigned long nowCheck = millis();
    if (!mpuPresent && (nowCheck - lastMpuCheckMillis >= MPU_CHECK_INTERVAL)) {
        lastMpuCheckMillis = nowCheck;
        initMPU6500();
        if (mpuPresent) {
            Serial.println("[MPU] MPU6500 detected - resuming deflection readings.");
        }
    }

    if (!mpuPresent) return; // don't publish stale/garbage data for a missing sensor

    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x3B); // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) {
        // Device stopped responding after init succeeded - mark it down
        mpuPresent = false;
        Serial.println("[MPU] Lost communication with MPU6500.");
        return;
    }
    Wire.requestFrom(MPU6500_ADDR, (uint8_t)6);

    if (Wire.available() >= 6) {
        int16_t rawAccX = (Wire.read() << 8) | Wire.read();
        int16_t rawAccY = (Wire.read() << 8) | Wire.read();
        int16_t rawAccZ = (Wire.read() << 8) | Wire.read();

        // Apply saved calibration offsets
        float calAccX = (rawAccX - ACCEL_OFFSET_X) / 16384.0; 
        float calAccY = (rawAccY - ACCEL_OFFSET_Y) / 16384.0;
        float calAccZ = (rawAccZ - ACCEL_OFFSET_Z) / 16384.0;

        // Calculate pitch angle (bending angle)
        current_pitch_deg = atan2(calAccX, sqrt(calAccY * calAccY + calAccZ * calAccZ)) * 180.0 / M_PI;

        // Capture zero-load baseline position on startup
        // NOTE: only counts samples that actually made it this far,
        // so a slow-starting sensor no longer bakes garbage into the baseline
        if (!baselineCaptured) {
            static int samples = 0;
            static float accumPitch = 0.0;
            accumPitch += current_pitch_deg;
            samples++;

            if (samples >= 15) {
                baseline_pitch_deg = accumPitch / 15.0;
                baselineCaptured = true;
                Serial.printf("[MPU] Zero-Load Baseline Anchored at: %.2f°\n", baseline_pitch_deg);
            }
        }

        // Calculate physical deflection magnitude
        if (baselineCaptured) {
            jib_deflection_deg = fabs(current_pitch_deg - baseline_pitch_deg);
        }
    }
    // else: short/partial read - silently skip this cycle, try again next loop.
    // We intentionally do NOT mark the sensor down on a single missed read;
    // only a hard endTransmission() failure (handled above) does that.
}

// ==========================================
// HP5883 Magnetometer (Yaw Angle)
// ==========================================
void writeMagRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readMagRegister(uint8_t reg) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MAG_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

// --- single burst read of X (0x01/0x02) and Y (0x03/0x04) instead
// of 4 separate register transactions per updateHeading() call ---
bool readMagXY(int16_t &outX, int16_t &outY) {
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x01); // start at X LSB; X MSB, Y LSB, Y MSB follow contiguously
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom(MAG_ADDR, (uint8_t)4);
    if (Wire.available() < 4) return false;

    uint8_t xLsb = Wire.read();
    uint8_t xMsb = Wire.read();
    uint8_t yLsb = Wire.read();
    uint8_t yMsb = Wire.read();

    outX = (int16_t)((xMsb << 8) | xLsb);
    outY = (int16_t)((yMsb << 8) | yLsb);
    return true;
}

void initHP5883() {
    writeMagRegister(0x0A, 0x4D); // Continuous measurement mode
    delay(10);
    writeMagRegister(0x0B, 0x01); // Set/Reset Period
    delay(10);

    // Verify the device actually acked instead of assuming success
    Wire.beginTransmission(MAG_ADDR);
    if (Wire.endTransmission() == 0) {
        magPresent = true;
        Serial.println("[MAG] HP5883 Initialized.");
    } else {
        magPresent = false;
        Serial.println("[MAG] HP5883 Not Found! Yaw readings disabled.");
    }
}

void updateHeading() {
    // Throttled re-probe so a sensor that comes back after a drop
    // is picked back up without a reboot - same pattern as INA226.
    unsigned long nowCheck = millis();
    if (!magPresent && (nowCheck - lastMagCheckMillis >= MAG_CHECK_INTERVAL)) {
        lastMagCheckMillis = nowCheck;
        initHP5883();
        if (magPresent) {
            Serial.println("[MAG] HP5883 detected - resuming yaw readings.");
        }
    }

    if (!magPresent) return;

    // Freeze yaw calculations when Electromagnet is powered to prevent magnetic EMI distortion
    if (digitalRead(MAGNET_RELAY_PIN) == RELAY_ON) {
        return; 
    }

    int16_t rawX, rawY;
    if (!readMagXY(rawX, rawY)) {
        magPresent = false;
        Serial.println("[MAG] Lost communication with HP5883.");
        return;
    }

    float calX = rawX - MAG_OFFSET_X;
    float calY = rawY - MAG_OFFSET_Y;

    float raw_heading = atan2(calY, calX) * 180.0 / M_PI;

    if (raw_heading < 0)   raw_heading += 360.0;
    if (raw_heading >= 360) raw_heading -= 360.0;

    // Low-Pass Filter (Smoothing)
    static float smoothed_yaw = 0.0;
    if (smoothed_yaw == 0.0) smoothed_yaw = raw_heading; 

    smoothed_yaw = (0.2 * raw_heading) + (0.8 * smoothed_yaw);
    jib_yaw_deg = smoothed_yaw;
}

// ==========================================
// BTS7960 Motor Driver & Relay Controls
// ==========================================
void motorForward() {
    ledcWrite(MOTOR_LPWM, 0);                 
    ledcWrite(MOTOR_RPWM, MOTOR_RUN_SPEED);   
    Serial.printf(">>> [MOTOR] FORWARD (Speed: %d) <<<\n", MOTOR_RUN_SPEED);
}

void motorReverse() {
    ledcWrite(MOTOR_RPWM, 0);                 
    ledcWrite(MOTOR_LPWM, MOTOR_RUN_SPEED);   
    Serial.printf(">>> [MOTOR] REVERSE (Speed: %d) <<<\n", MOTOR_RUN_SPEED);
}

void motorStop() {
    ledcWrite(MOTOR_RPWM, 0);
    ledcWrite(MOTOR_LPWM, 0);
    Serial.println(">>> [MOTOR] STOP <<<");
}

void magnetOn() {
    digitalWrite(MAGNET_RELAY_PIN, RELAY_ON); 
    Serial.println(">>> [MAGNET] RELAY CLOSED -> ON <<<");
}

void magnetOff() {
    digitalWrite(MAGNET_RELAY_PIN, RELAY_OFF); 
    Serial.println(">>> [MAGNET] RELAY OPENED -> OFF <<<");
}

// ==========================================
// INA226 Power Sensor & Flash Storage
// ==========================================

// The INA226_WE library doesn't expose a "present/absent" check itself,
// so we verify the device acks on the bus directly - same pattern used
// for the MPU6500 and HP5883 above.
bool checkINA226Ack() {
    Wire.beginTransmission(INA226_I2C_ADDR);
    return (Wire.endTransmission() == 0);
}

// Throttles the bus probe so a bare/floating I2C bus (nothing wired,
// NACKs not returning cleanly) can't stall the main loop every iteration
// waiting on Wire.setTimeOut(). Same pattern as the WiFi/MQTT reconnect
// throttling already used elsewhere.
constexpr unsigned long INA226_CHECK_INTERVAL = 5000;
unsigned long lastIna226CheckMillis = 0;

void readPowerSensors() {
    unsigned long now = millis();

    if (now - lastIna226CheckMillis >= INA226_CHECK_INTERVAL) {
        lastIna226CheckMillis = now;
        bool ackNow = checkINA226Ack();
        if (ackNow && !ina226Present) {
            ina226Present = true;
            Serial.println("[PWR] INA226 detected - resuming power readings.");
        } else if (!ackNow && ina226Present) {
            ina226Present = false;
            Serial.println("[PWR] Lost communication with INA226.");
        }
    }

    if (!ina226Present) {
        actual_V = 0.0;
        current_A = 0.0;
        power_W = 0.0;
        return;
    }

    ina226.readAndClearFlags();
    actual_V  = ina226.getBusVoltage_V();      
    current_A = ina226.getCurrent_mA() / 1000.0; 

    if (fabs(current_A) < 0.005) current_A = 0.0; 

    power_W = actual_V * current_A; 

    // Overcurrent protection: cut the motor if draw exceeds threshold.
    // Only acts on a valid sensor reading (ina226Present already checked above).
    if ((current_A * 1000.0) > OVERCURRENT_THRESHOLD_MA) {
        motorStop();
        Serial.printf("[PWR] Overcurrent! %.2f mA > %.1f mA threshold - motor stopped.\n",
                      current_A * 1000.0, OVERCURRENT_THRESHOLD_MA);
    }
}

void manageFlashStorage() {
    unsigned long nowMillis = millis();
    float elapsedSec = (nowMillis - lastRuntimeCheckMillis) / 1000.0;
    lastRuntimeCheckMillis = nowMillis;

    currentSessionTime_sec += elapsedSec;
    totalOperationTime_sec += elapsedSec;

    if (nowMillis - lastFlashSaveMillis >= FLASH_SAVE_INTERVAL) {
        if ((totalOperationTime_sec - last_saved_time) > 10.0) {
            preferences.begin("crane_data", false);
            preferences.putFloat("total_time", totalOperationTime_sec);
            preferences.end();

            last_saved_time = totalOperationTime_sec;
            lastFlashSaveMillis = nowMillis;
            Serial.println("[SYS] Lifetime state saved to Flash Memory.");
        }
    }
}

// ==========================================
// WiFi Connection (Throttled Reconnect)
// ==========================================
void handleWiFiConnect() {
    static bool wasConnected = false;
    
    // Fast-path: If already connected, do nothing and return immediately
    if (WiFi.status() == WL_CONNECTED) {
        if (!wasConnected) {
            wasConnected = true;
            Serial.print("\n[WIFI] Connected! Active SSID: ");
            Serial.print(WiFi.SSID());
            Serial.print(" | Local IP: ");
            Serial.println(WiFi.localIP());
        }
        return;
    }

    // We are disconnected - throttle reconnection scans to once every 5 seconds
    wasConnected = false;
    unsigned long now = millis();
    if (now - lastWifiAttemptMillis < WIFI_RETRY_INTERVAL) return;
    lastWifiAttemptMillis = now;

    Serial.println("[WIFI] Scanning and connecting to known networks...");
    wifiMulti.run(); // Now only executes once every 5 seconds!
}

// ==========================================
// OTA Firmware Update (ThingsBoard-driven)
// ==========================================
bool otaInProgress = false;
bool otaCheckedThisSession = false; // only request shared attrs once per boot/connect

void reportFirmwareState(const char* state, const char* error = nullptr) {
    StaticJsonDocument<128> doc;
    doc["fw_state"] = state;
    if (error != nullptr) doc["fw_error"] = error;
    char payload[128];
    serializeJson(doc, payload);
    mqttClient.publish(pub_topic, payload);
    Serial.printf("[OTA] State: %s%s%s\n", state, error ? " - " : "", error ? error : "");
}

// Requests the firmware-related shared attributes from ThingsBoard.
// Response arrives asynchronously on "v1/devices/me/attributes/response/+"
// and is handled in mqttCallback().
void requestFirmwareAttributes() {
    StaticJsonDocument<128> reqDoc;
    reqDoc["sharedKeys"] = "fw_title,fw_version,fw_checksum,fw_checksum_algorithm,fw_size";
    char payload[128];
    serializeJson(reqDoc, payload);
    mqttClient.publish("v1/devices/me/attributes/request/1", payload);
    Serial.println("[OTA] Requested firmware shared attributes.");
}

// Downloads and flashes new firmware from ThingsBoard's HTTP firmware
// endpoint, verifies it, and reboots. Returns without side effects (old
// firmware keeps running) if anything fails along the way.
void performOtaUpdate(const String &fwTitle, const String &fwVersion,
                      const String &fwChecksum, const String &fwChecksumAlgo,
                      long fwSize) {
    if (otaInProgress) return; // guard against re-entry from a duplicate attribute push
    otaInProgress = true;

    Serial.printf("[OTA] Update available: %s (current: %s)\n", fwVersion.c_str(), FW_VERSION);
    reportFirmwareState("DOWNLOADING");

    if (fwChecksumAlgo != "MD5") {
        // We only implement MD5 auto-verification via the Update library here.
        // If your ThingsBoard package uses SHA256 (the ThingsBoard default),
        // re-upload the package with checksum algorithm set to MD5, or this
        // update needs a SHA256 verification path added.
        reportFirmwareState("FAILED", "Unsupported checksum algorithm (expected MD5)");
        otaInProgress = false;
        return;
    }

    String url = "http://" + String(mqtt_server) + ":" + String(OTA_HTTP_PORT) +
                 "/api/v1/" + String(mqtt_user) + "/firmware?title=" + fwTitle +
                 "&version=" + fwVersion;

    // IMPORTANT: use a dedicated WiFiClient for the HTTP download, separate
    // from the global espClient used by mqttClient. Reusing espClient here
    // would hand the MQTT connection's socket over to the HTTP transfer and
    // corrupt/drop the MQTT session mid-update.
    WiFiClient otaClient;
    HTTPClient http;

    // Bound how long a single connect/GET attempt can block. Without this,
    // an unreachable host/port (e.g. port 8080 not actually open) can hang
    // well past the 10s watchdog window with zero chance to feed it,
    // triggering a panic-reboot instead of a clean "FAILED" report.
    http.setConnectTimeout(10000); // ms
    http.setTimeout(10000);        // ms

    http.begin(otaClient, url);

    esp_task_wdt_reset(); // feed right before the blocking call
    int httpCode = http.GET();
    esp_task_wdt_reset(); // feed immediately after, in case it ran close to the limit

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("[OTA] HTTP GET failed, code: %d\n", httpCode);
        reportFirmwareState("FAILED", "HTTP download request failed");
        http.end();
        otaInProgress = false;
        return;
    }

    int contentLen = http.getSize();
    if (contentLen <= 0 || (fwSize > 0 && contentLen != fwSize)) {
        Serial.println("[OTA] Content length missing or mismatched with fw_size attribute.");
        reportFirmwareState("FAILED", "Content length mismatch");
        http.end();
        otaInProgress = false;
        return;
    }

    if (!Update.begin(contentLen)) {
        Serial.println("[OTA] Not enough space for OTA update.");
        reportFirmwareState("FAILED", "Insufficient OTA partition space");
        http.end();
        otaInProgress = false;
        return;
    }

    Update.setMD5(fwChecksum.c_str());

    WiFiClient *stream = http.getStreamPtr();
    uint8_t buf[512];
    int written = 0;
    unsigned long lastProgressLog = millis();

    while (http.connected() && written < contentLen) {
        size_t avail = stream->available();
        if (avail) {
            int toRead = avail > sizeof(buf) ? sizeof(buf) : avail;
            int n = stream->readBytes(buf, toRead);
            if (n > 0) {
                if (Update.write(buf, n) != (size_t)n) {
                    Serial.println("[OTA] Flash write error mid-download.");
                    reportFirmwareState("FAILED", "Flash write error");
                    Update.abort();
                    http.end();
                    otaInProgress = false;
                    return;
                }
                written += n;
            }
        }
        // Downloading a full firmware image can take a while over a remote
        // link - keep the watchdog fed throughout, same principle as the
        // MQTT connect guard above.
        esp_task_wdt_reset();

        // Service the MQTT connection (separate socket from otaClient) so
        // its keepalive PINGREQ still goes out during a long download -
        // otherwise ThingsBoard may drop the MQTT session before our
        // post-download "VERIFIED"/"UPDATED" status reports get sent.
        if (mqttClient.connected()) {
            mqttClient.loop();
        }

        if (millis() - lastProgressLog > 3000) {
            lastProgressLog = millis();
            Serial.printf("[OTA] Progress: %d / %d bytes\n", written, contentLen);
        }
        delay(1);
    }
    http.end();

    if (written != contentLen) {
        Serial.println("[OTA] Download incomplete.");
        reportFirmwareState("FAILED", "Download incomplete");
        Update.abort();
        otaInProgress = false;
        return;
    }

    reportFirmwareState("VERIFIED");
    reportFirmwareState("UPDATING");

    if (!Update.end(true)) {
        Serial.printf("[OTA] Update.end() failed: %s\n", Update.errorString());
        reportFirmwareState("FAILED", Update.errorString());
        otaInProgress = false;
        return;
    }

    reportFirmwareState("UPDATED");
    Serial.println("[OTA] Update successful. Rebooting...");
    delay(1000); // let the MQTT publish above actually flush before reset
    esp_restart();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

    String topicStr = String(topic);

    // Firmware shared-attribute handling
    if (topicStr.startsWith("v1/devices/me/attributes")) {
        StaticJsonDocument<256> attrDoc;
        if (deserializeJson(attrDoc, msg)) {
            Serial.println("[OTA] Attribute JSON parse FAILED");
            return;
        }

        JsonVariant attrs;
        if (attrDoc.containsKey("shared")) {
            attrs = attrDoc["shared"];
        } else {
            attrs = attrDoc.as<JsonVariant>();
        }

        if (attrs.containsKey("fw_title") && attrs.containsKey("fw_version") &&
            attrs.containsKey("fw_checksum") && attrs.containsKey("fw_checksum_algorithm")) {

            String fwTitle    = attrs["fw_title"].as<String>();
            String fwVersion  = attrs["fw_version"].as<String>();
            String fwChecksum = attrs["fw_checksum"].as<String>();
            String fwAlgo     = attrs["fw_checksum_algorithm"].as<String>();
            long fwSize       = attrs.containsKey("fw_size") ? attrs["fw_size"].as<long>() : 0;

            if (fwTitle == FW_TITLE && fwVersion != String(FW_VERSION)) {
                performOtaUpdate(fwTitle, fwVersion, fwChecksum, fwAlgo, fwSize);
            }
        }
        return;
    }

    StaticJsonDocument<200> rpcDoc;
    if (deserializeJson(rpcDoc, msg)) {
        Serial.println("[RPC] JSON parse FAILED");
        return;
    }

    String method = rpcDoc["method"].as<String>();
    
    // --> X-RAY DEBUGGER <--
    Serial.printf("\n>>> [RPC X-RAY] Arrived: '%s' <<<\n", method.c_str());
    
    if (method == "setForward")       motorForward();
    else if (method == "setReverse")  motorReverse();
    else if (method == "setStop")     motorStop();
    else if (method == "setMagnetOn") magnetOn();
    else if (method == "setMagnetOff") magnetOff();
    else if (method == "setSpeed") {
        if (rpcDoc["params"].is<int>()) {
            MOTOR_RUN_SPEED = rpcDoc["params"].as<int>();
            MOTOR_RUN_SPEED = constrain(MOTOR_RUN_SPEED, 0, 255);
            Serial.printf(">>> [MOTOR] Speed Updated via RPC to: %d <<<\n", MOTOR_RUN_SPEED);
        }
    }
    // --> REMOTE REBOOT COMMAND <--
    else if (method == "reboot") {
        Serial.println("[SYS] Remote reboot triggered via RPC!");
        reportFirmwareState("REBOOTING", "Remote RPC command");
        delay(1000);
        esp_restart();
    }
    else {
        Serial.printf("[RPC ERROR] Unrecognized method: '%s'\n", method.c_str());
    }
}

void handleMQTTConnect() {
    if (!mqttClient.connected()) {
        unsigned long now = millis();
        if (now - lastReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
            lastReconnectAttempt = now;

            // Feed the watchdog immediately before the blocking connect() call.
            // PubSubClient's connect() can take several seconds over a flaky
            // or remote link; this guarantees we don't panic-reboot mid-attempt.
            esp_task_wdt_reset();

            String clientId = "ESP32Crane-" + String(random(0xffff), HEX);
            Serial.printf("[MQTT] Connecting to target server: %s:%d ...\n", mqtt_server, mqtt_port);

            if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
                mqttClient.subscribe(rpc_sub_topic);
                // Subscribe to shared-attribute push updates and our own
                // request/response topic for the firmware check
                mqttClient.subscribe("v1/devices/me/attributes");
                mqttClient.subscribe("v1/devices/me/attributes/response/+");
                Serial.println("[MQTT] Connected and RPC/attribute channels subscribed.");

                // This is the first successful MQTT connection since boot -
                // if we just came up from a fresh OTA image, this is our
                // signal that the new firmware works well enough to talk
                // to the network, so cancel any pending rollback. Requires
                // bootloader app-rollback support to be compiled into your
                // Arduino-ESP32 core; if this function isn't available for
                // your core version, remove this block.
                static bool rollbackChecked = false;
                if (!rollbackChecked) {
                    rollbackChecked = true;
                    const esp_partition_t *running = esp_ota_get_running_partition();
                    esp_ota_img_states_t otaState;
                    if (esp_ota_get_state_partition(running, &otaState) == ESP_OK &&
                        otaState == ESP_OTA_IMG_PENDING_VERIFY) {
                        esp_ota_mark_app_valid_cancel_rollback();
                        Serial.println("[OTA] New firmware confirmed working - rollback cancelled.");
                    }
                }

                // Ask ThingsBoard if a newer firmware package is assigned
                // to this device, once per boot.
                if (!otaCheckedThisSession) {
                    otaCheckedThisSession = true;
                    requestFirmwareAttributes();
                }
            } else {
                Serial.printf("[MQTT] Connection failed. Error Code: %d\n", mqttClient.state());
            }

            // Feed again right after, in case connect() itself ran long.
            esp_task_wdt_reset();
        }
    } else {
        mqttClient.loop();
    }
}

void publishTelemetry() {
    StaticJsonDocument<512> doc;

    doc["voltage_V"]                = round(actual_V * 100.0) / 100.0;
    doc["current_mA"]              = round((current_A * 1000.0) * 100.0) / 100.0; 
    doc["power_mW"]                = round((power_W * 1000.0) * 100.0) / 100.0;   
    doc["jib_yaw_deg"]             = round(jib_yaw_deg * 10.0) / 10.0; 
    doc["jib_deflection_deg"]      = round(jib_deflection_deg * 100.0) / 100.0;
    doc["current_session_time_sec"] = round(currentSessionTime_sec * 100.0) / 100.0;
    doc["total_operation_time_sec"] = round(totalOperationTime_sec * 100.0) / 100.0;

    // Surface sensor health so a dashboard/alert can catch a
    // disconnected or failed sensor instead of it going unnoticed
    doc["mpu_ok"] = mpuPresent;
    doc["mag_ok"] = magPresent;
    doc["ina226_ok"] = ina226Present;

    char payload[512];
    serializeJson(doc, payload);

    if (mqttClient.publish(pub_topic, payload)) {
        Serial.print("[MQTT] Telemetry Sent: ");
        Serial.println(payload);
    }
}

// ==========================================
// Main Setup & Loop
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== CRANE FIRMWARE INITIALIZATION ===");
    Serial.print("Firmware Version: ");
    Serial.println(FW_VERSION);
  
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(50000); // 50kHz for long cable stability
    Wire.setTimeOut(150);

    // Bind BTS7960 PWM outputs
    ledcAttach(MOTOR_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(MOTOR_LPWM, PWM_FREQ, PWM_RESOLUTION);
    motorStop();

    // Initialize Electromagnet Relay
    pinMode(MAGNET_RELAY_PIN, OUTPUT);
    magnetOff(); 

    // Init Sensors
    initHP5883();
    initMPU6500();

    // Init INA226 Power Engine
    Serial.println("[PWR] Initializing INA226...");
    if (checkINA226Ack()) {
        ina226Present = true;
        ina226.setAverage(INA226_AVERAGE_16);
        ina226.setConversionTime(INA226_CONV_TIME_1100);
        ina226.setMeasureMode(INA226_CONTINUOUS);
        ina226.setResistorRange(INA226_SHUNT_OHMS, INA226_MAX_CURRENT);
        Serial.println("[PWR] INA226 Ready.");
    } else {
        ina226Present = false;
        Serial.println("[PWR] INA226 Not Found! Power readings disabled.");
    }

    preferences.begin("crane_data", false);
    bootStartTotalTime_sec = preferences.getFloat("total_time", 0.0);
    preferences.end();

    last_saved_time = bootStartTotalTime_sec;
    totalOperationTime_sec = bootStartTotalTime_sec;

    // WiFi Setup
    Serial.println("[WIFI] Registering known Wi-Fi networks...");
    WiFi.mode(WIFI_STA);
    
    // Add all known locations - ESP32 will automatically connect to the strongest available
    wifiMulti.addAP("DTS_Chennai_2.4", "Dts@2025");         // Chennai Office
    wifiMulti.addAP("Harsha ✨", "Hahakob09");              // Mobile Hotspot
    // wifiMulti.addAP("Airtel_vasa_2493", "Air@25219");        // Jor life
    wifiMulti.addAP("OOMNI-EYE-2.4GHz", "Admin@2025");      // Dubai network
    
    Serial.println("[WIFI] Connecting to available Wi-Fi...");
    lastWifiAttemptMillis = millis();

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);

    // Bound how long a single connect() attempt can block, so it can
    // never on its own exceed the watchdog window.
    mqttClient.setSocketTimeout(8); // seconds

    lastRuntimeCheckMillis = millis();

    // Task Watchdog Timer
    // NOTE: recent Arduino-ESP32 cores auto-initialize the TWDT before setup()
    // runs. Calling esp_task_wdt_init() again then fails silently, meaning our
    // intended 10s/panic config might not actually be active. Reconfigure
    // instead of re-initializing if it's already running.
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 10000,                               
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  
        .trigger_panic = true                             
    };
    esp_err_t wdtInitResult = esp_task_wdt_init(&twdt_config);
    if (wdtInitResult == ESP_ERR_INVALID_STATE) {
        // Already initialized by the framework - reconfigure it to our settings
        esp_task_wdt_reconfigure(&twdt_config);
        Serial.println("[SYS] TWDT already running - reconfigured to 10000ms/panic.");
    } else {
        Serial.println("[SYS] TWDT initialized to 10000ms/panic.");
    }
    esp_task_wdt_add(NULL);
    
    Serial.println("[SYS] System Ready.");
}

void loop() {
    esp_task_wdt_reset(); 

    unsigned long currentTime = millis();

    handleWiFiConnect(); // keeps retrying in the background if WiFi drops

    if (WiFi.status() == WL_CONNECTED) {
        handleMQTTConnect();
    }

    updateHeading();     // HP5883 Yaw
    updateDeflection();  // MPU6500 Deflection
    readPowerSensors();  // INA226 Power
    manageFlashStorage();

    if (currentTime - lastMsgTime > TELEMETRY_INTERVAL) {
        lastMsgTime = currentTime;

        if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
            publishTelemetry();
        }
    }

    delay(2); 
}