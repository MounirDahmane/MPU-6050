# MPU6050-Core: Low-Level I2C IMU Driver

A high-performance, library-independent C++ interface for the **MPU6050 6-DOF Inertial Measurement Unit (IMU)**.  
This project implements raw **I²C register access** to provide filtered spatial orientation (**Roll, Pitch, Yaw**) with high temporal precision.

---

## 🚀 Key Features

### Zero-Dependency I²C
- Direct register-level communication using **Wire.h**.
- No external IMU libraries required.

### Static Bias Calibration
- Automated routine to calculate and nullify sensor offsets using **1000 samples**.

### Sensor Fusion
- Implements a **Complementary Filter** to combine:
  - High-frequency **gyroscope data**
  - Long-term **accelerometer stability**

### Precision Timing
- Uses `micros()` for discrete-time integration to minimize cumulative error.

### Non-Blocking Telemetry
- Optimized main loop designed for **real-time flight control or robotics** *(f > 200 Hz execution rate).*

---

## 🛠️ Technical Specifications

| Feature | Configuration | Sensitivity |
| :--- | :--- | :--- |
| Gyroscope Range | ±500°/s | 65.5 LSB/(°/s) |
| Accelerometer Range | ±8g | 4096 LSB/g |
| I²C Clock Speed | 400 kHz (Fast Mode) | N/A |
| Filter Constant (α) | 0.98 | N/A |

---

## 🧠 How It Works

The driver estimates orientation by fusing data from two different sensors to compensate for their individual weaknesses.

### Gyroscope
- Precise for fast motion.
- Suffers from **low-frequency drift**.

### Accelerometer
- Provides an **absolute gravity reference**.
- Susceptible to **high-frequency vibration noise**.

### Complementary Filter

The estimated angle is calculated as:

$$\theta_n = \alpha (\theta_{n-1} + \omega \cdot \Delta t) + (1 - \alpha) a$$

Where:

- **ω** — Angular velocity from the gyroscope  
- **a** — Static tilt angle from the accelerometer  
- **Δt** — Sampling period (delta time)

---

## 📁 Project Structure

```text
.
├── include/
│   └── MPU6050.h        # Hardware definitions, constants, telemetry externs
├── src/
│   ├── MPU6050.cpp      # Driver logic, calibration, complementary filter
│   └── main.cpp         # High-frequency execution loop and telemetry
```

---

## 📦 Getting Started

### Prerequisites

- **PlatformIO** or **Arduino IDE**
- Any compatible microcontroller:
  - ESP32
  - Arduino
  - STM32

### Installation

1. Clone the repository.

```bash
git clone <repository_url>
cd MPU6050-Core
```

2. Connect the MPU6050 to the **I²C pins**:

- **SDA**
- **SCL**

3. Upload the firmware to your microcontroller.

**Important:** Keep the sensor **perfectly still and level during the first ~3 seconds after boot** so the automatic calibration routine can correctly compute the bias offsets.