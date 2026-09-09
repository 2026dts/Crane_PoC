# Under Construction model 

An ESP32-based IoT proof-of-concept for an under-construction tower crane model. The system monitors jib rotation, structural deflection, and motor electrical parameters in real time, streams telemetry to **ThingsBoard Cloud** over MQTT, and supports remote dashboard control and **OTA firmware updates**.


---

## Overview

This project is a scaled physical demonstration model built to showcase an Industrial IoT architecture for tower-crane condition monitoring. It combines motion sensing, structural monitoring, electrical parameter measurement, cloud connectivity, and remote firmware management on a single ESP32 platform.

## Features

- Real-time jib rotation monitoring (0–360°)
- Jib deflection monitoring (pitch, roll, vibration)
- Motor voltage, current, and power monitoring
- DC motor speed control
- Electromagnet control for load pick-and-place demonstration
- MQTT communication with ThingsBoard Cloud
- Live ThingsBoard dashboard
- Over-the-Air (OTA) firmware upgrade
- Sensor health monitoring

## System Architecture

```
                 +--------------------------------------+
                 |            ThingsBoard Cloud          |
                 | Dashboard | Telemetry | OTA Updates   |
                 +------------------▲---------------------+
                                    │
                                MQTT/Wi-Fi
                                    │
                  +-----------------┴-----------------+
                  |               ESP32                |
                  |------------------------------------|
                  | Sensor Processing                  |
                  | Motor Control                       |
                  | OTA Client                           |
                  +----+------------+---------------+---+
                       │            │               │
                I²C Bus│            │PWM/GPIO       │GPIO
                       │            │               │
   +-------------------+------------+---------------+--------+
   │                   │            │                        │
   ▼                   ▼            ▼                        ▼
HMC5883L           MPU6500       INA226                  BTS7960
Jib Angle       Jib Deflection  Power Monitor           Motor Driver
                                                              │
                                            +-----------------+----+
                                            │                      │
                                            ▼                      ▼
                                      DC Gear Motor          Electromagnet
```

## Hardware Components

| Component | Function |
|---|---|
| ESP32 | Main controller |
| HMC5883L Magnetometer | Jib rotation (0–360°) |
| MPU6500 IMU | Jib deflection (pitch, roll, vibration) |
| INA226 | Voltage, current, and power measurement |
| BTS7960 | Motor driver |
| 12V DC Gear Motor | Crane rotation |
| Electromagnet | Load pick-and-place |
| Mini PC | Dashboard host and USB power source |
| ThingsBoard Cloud | IoT platform |

## Wiring

| ESP32 Pin | Device |
|---|---|
| GPIO21 | I²C SDA |
| GPIO22 | I²C SCL |
| GPIO25 | BTS7960 RPWM |
| GPIO26 | BTS7960 LPWM |
| GPIO27 *(or assigned GPIO)* | Electromagnet control |
| 3.3V | Sensors |
| GND | Common ground |

HMC5883L, MPU6500, and INA226 share a single I²C bus, minimizing GPIO usage and simplifying wiring/expansion.

## Power Architecture

- **Logic power (3.3V/5V):** Mini PC → USB → ESP32 → sensors
- **High power (12V):** 230V AC → 12V DC adapter → INA226 → BTS7960 → DC gear motor; a separate 12V adapter/driver stage powers the electromagnet

Separating logic and high-power rails reduces electrical noise, keeps sensor readings stable, and lowers the risk of ESP32 resets from motor current spikes.

## Software Workflow

```
Power ON → Init ESP32 → Connect Wi-Fi → Connect MQTT → Init Sensors
   → Read HMC5883L / MPU6500 / INA226 → Filter Data → Build JSON Payload
   → Upload Telemetry → Receive Dashboard Commands → Motor/Electromagnet Control
   → Repeat
```

## Telemetry Published to ThingsBoard

| Parameter | Source |
|---|---|
| Jib angle | HMC5883L |
| Pitch / Roll / Vibration | MPU6500 |
| Motor voltage / current / power | INA226 |
| PWM / Motor direction | ESP32 |
| Firmware version / Device status | ESP32 |

## OTA Firmware Update

OTA updates are delivered via ThingsBoard firmware management over MQTT:

```
Firmware Upload → ThingsBoard → MQTT Notification → ESP32
   → Download → Verify (checksum) → Flash → Restart → Report New Version
```

## Design Notes

- **Separate power supplies** — isolates sensor/logic rails from motor/electromagnet current spikes, improving stability and reducing resets.
- **Shared I²C bus** — reduces GPIO usage and simplifies wiring/expansion across HMC5883L, MPU6500, and INA226.
- **Sensor placement** — HMC5883L is mounted away from the motor and electromagnet to avoid magnetic interference; MPU6500 sits mid-jib for accurate structural readings; INA226 sits close to the motor power input for accurate current sensing.

## Future Improvements

- [ ] Wind speed sensor (RS485)
- [ ] Load cell
- [ ] Limit switches
- [ ] Industrial environmental sensor
- [ ] Emergency stop monitoring
- [ ] Predictive maintenance algorithms
- [ ] Alarm management
- [ ] Digital twin visualization

## Repository Structure

```
.
├── firmware/          # ESP32 source code
├── docs/              # Design documents, diagrams
├── dashboards/        # ThingsBoard dashboard exports
└── README.md
```

## Getting Started

1. Flash the firmware in `firmware/` to the ESP32 using Arduino IDE or PlatformIO.
2. Configure Wi-Fi credentials and the ThingsBoard MQTT device token in the firmware config.
3. Import the dashboard JSON from `dashboards/` into your ThingsBoard instance.
4. Power on the model — telemetry should appear on the ThingsBoard dashboard within a few seconds.

