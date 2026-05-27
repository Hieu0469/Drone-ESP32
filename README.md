# ESP32 Bluetooth Quadcopter Flight Controller

A custom-built, real-time Flight Controller for a quadcopter drone based on the ESP32 microcontroller. This project features a 1000Hz (1ms) hardware-timer-driven control loop, MPU6050 IMU integration with Kalman filtering, and remote control/telemetry via Bluetooth Classic.

---

## 🚀 Key Features

* **Real-Time OS-Level Control:** Utilizes ESP32 hardware timers (`hw_timer_t`) to guarantee a precise 1ms (1000Hz) PID control loop.
* **Advanced Filtering:** Integrates Low-Pass Filters and a Kalman Filter to process raw MPU6050 data, ensuring stable pitch and roll angle estimation.
* **Bluetooth Telemetry & Control:** Full control via a mobile app or Bluetooth terminal, including live telemetry streaming (angles, motor outputs, throttle).
* **Runtime Configuration:** Tune PID gains, set throttle limits, and trim motors dynamically without reflashing the ESP32.
* **Non-Volatile Storage (NVS):** Motor trim values are saved to the ESP32's flash memory and reloaded on boot to maintain stable hovering.
* **Safety Mechanisms:** Includes Arm/Disarm sequences, Emergency Stop, and auto-disarm when extreme tilt angles (>45°) or signal timeouts are detected.

---

## 🛠 Hardware Configuration

### Pin Mapping

| Component | Pin / GPIO | Description |
| :--- | :--- | :--- |
| **MPU6050 (SDA)** | `GPIO 21` | I2C Data (400kHz Fast Mode) |
| **MPU6050 (SCL)** | `GPIO 22` | I2C Clock |
| **Motor FL** | `GPIO 25` | Front-Left ESC (PWM Channel 0) |
| **Motor FR** | `GPIO 26` | Front-Right ESC (PWM Channel 1) |
| **Motor BL** | `GPIO 27` | Back-Left ESC (PWM Channel 2) |
| **Motor BR** | `GPIO 14` | Back-Right ESC (PWM Channel 3) |

### ESC Specifications
* **PWM Frequency:** 50Hz
* **Resolution:** 16-bit
* **Pulse Width:** 1000µs (Min/Arm) to 2000µs (Max)

---

## 📡 Bluetooth Command Protocol

Connect to the device via Bluetooth Classic. **Device Name:** `ESP32_Drone`.

### 1. Flight Controls (Single Character)
These commands trigger ramping functions (smooth transition) updated every 20ms.

| Key | Action | Key | Action |
| :--- | :--- | :--- | :--- |
| `A` | **Arm** the drone | `S` | **Disarm** the drone |
| `X` | **Emergency Stop** (Cut power) | `H` | **Hold** (Reset angles to 0, keep throttle) |
| `U` / `D` | Throttle Up / Down | `P` | Print Status & PID |
| `L` / `R` | Roll Left / Right | `F` / `B` | Pitch Forward / Backward |
| `Q` / `E` | Yaw Left / Right | | |

### 2. Live PID Tuning
You can adjust the PID gains on the fly.
* **Syntax:** `<axis><param>:<value>`
* **Axes:** `r` (Roll), `p` (Pitch), `y` (Yaw)
* **Params:** `p` (Proportional), `i` (Integral), `d` (Derivative)
* **Examples:** * `rp:1.5` -> Sets Roll Kp to 1.5
  * `yi:0.05` -> Sets Yaw Ki to 0.05

### 3. Motor Trimming
Used to compensate for physical imbalances in the drone frame or motors. Range: -20.0 to +20.0.
* `trim:fl:+5.0` : Add 5% throttle offset to Front-Left motor.
* `trim:?` : View current trim values.
* `trim:reset` : Reset all trims to 0.
* `trim:save` : **Save current trims to NVS memory.**

### 4. Configuration Settings
* `cfg:tmax:55.0` : Set Maximum Throttle to 55% (Range: 20-90%).
* `cfg:ilim:2.0` : Set PID Integral Limit to prevent windup.
* `cfg:?` : View current configuration.

---

## 🏗 Software Architecture

1. **`main.cpp`:** Initializes peripherals. Attaches a hardware timer interrupt that sets a `tickReady` flag every 1ms. The main `loop()` executes the control routine when the flag is high.
2. **`mpu6050_handler.h`:** Manages I2C communication. Auto-calibrates gyro/accel offsets on boot. Applies Low Pass and Kalman filters.
3. **`flight_controller.cpp`:** Contains 3 independent `PIDController` instances. Calculates error, applies PID formulas, and mixes the output into X-quadcopter motor configurations.
4. **`esc_controller.cpp`:** Maps standard 0-100% throttle values to 1000-2000µs pulses utilizing the ESP32 `ledc` hardware PWM API.
5. **`bt_controller.cpp`:** Parses incoming serial strings, applies smooth ramping to joystick commands, and outputs string-based telemetry at 10Hz.

---

## 💻 Getting Started

### Prerequisites
* PlatformIO or Arduino IDE.
* ESP32 Board package installed.
* Dependencies: Standard `Wire` library, `BluetoothSerial`, `Preferences`, and an MPU6050 library (e.g., ElectronicCats/I2Cdevlib-MPU6050).

### Installation
1. Clone the repository.
2. Open the project in your IDE.
3. Ensure the ESP32 partition scheme has Bluetooth enabled.
4. Compile and upload to your ESP32.
5. Open a Serial Monitor at `115200` baud. The drone will ask you to keep it flat for MPU6050 calibration (~1-2 seconds).
6. Connect via a Bluetooth Serial Terminal app on your phone/PC.

---

## ⚠️ Safety Warning

* **PROPELLERS ARE DANGEROUS.** Always test your PID tuning and motor directions with the propellers **REMOVED** from the drone.
* Ensure your ESCs are properly calibrated to the 1000µs - 2000µs range before attaching a battery.
* Keep your finger ready to send the `X` (Emergency Stop) command during initial test flights.

## 👁️ Computer Vision & Object Detection (YOLO Pipeline)

To keep the drone lightweight and cost-effective while enabling advanced AI features, this project utilizes an off-board processing architecture. A smartphone mounted on the drone acts as the camera payload, while a ground-station PC handles the heavy computational lifting using a lightweight YOLO nano model. The model is trained on VisDrone dataset with default Ultralystics training strategy.
<img width="564" height="339" alt="Screenshot 2026-05-17 212513" src="https://github.com/user-attachments/assets/48cf0afd-045b-4bff-a306-7bf61e3655a9" />

### 🔄 System Flow Architecture

1. **Video Capture:** A smartphone mounted securely on the drone frame captures real-time video.
2. **Transmission (Zoom):** The phone joins a **Zoom** meeting and broadcasts its camera feed to the ground-station PC over Wi-Fi/4G.
3. **Extraction (OBS Studio):** The PC runs **OBS Studio** to capture the Zoom meeting window. OBS then outputs this clean feed using the **OBS Virtual Camera** feature.
4. **AI Processing (YOLO):** A local Python script utilizes OpenCV to read the OBS Virtual Camera feed and processes the frames through the **YOLO** object detection model in real-time.


