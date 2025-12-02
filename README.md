# IoT Weather & Rice Grain Quality Monitoring System 🌾⛈️  
**Version:** 1.0 (Final)  

---

## 🌟 Key Features

### Predictive Weather Algorithms  
Detects **Low Pressure Areas (LPA)** combined with **Dark Skies** to predict rain *before it happens*.

### 🌾 Real-time Grain Monitoring  
Uses a **capacitive sensor** to check if stored rice is damp or wet using a **non-corrosive probe**.

### 📲 SMS Alerts  
Sends automatic alerts for:  
- **RAIN ALERT**  
- **GRAIN WET ALERT**  
Includes a **5-minute cooldown** to prevent spam.

### 🔁 Two-Way Communication  
Farmers can text commands (e.g., `STATUS`, `GRAIN`) to receive real-time updates.

### 🖥️ Smart LCD Dashboard  
- **Row 1:** Always shows critical grain status  
- **Row 2:** Rotates Temp, Humidity, Pressure, and Light data  
- **Pop-up alerts** override the screen for danger states.

### 🛡️ Robust Error Handling  
Auto-retry SMS logic, antenna noise filtering, and network self-checks for reliability.

---

## 🛠️ Hardware Requirements

| Component                 | Function                                               |
|--------------------------|---------------------------------------------------------|
| ESP32 Dev Module         | Main microcontroller (The Brain)                        |
| SIM800L (EVB Version)    | GSM Module for SMS communication                        |
| BME280                   | Measures Temp, Humidity, and Atmospheric Pressure       |
| BH1750                   | Measures Light Intensity                                |
| Capacitive Soil Sensor   | Measures Rice Grain Moisture (Corrosion resistant)      |
| LCD 16x2 (I2C)           | Visual display for on-site monitoring                   |
| Power Source             | 3× 18650 Li-ion (12V) with Buck Converter (5V output)   |

---

## 🔌 Wiring & Pinout

### **SIM800L**
| Pin Name | ESP32 Pin       | Notes                           |
|----------|------------------|--------------------------------|
| TXD      | GPIO 16 (RX2)    | Serial Communication           |
| RXD      | GPIO 17 (TX2)    | Serial Communication           |
| VCC      | External 5V       | Do **not** use ESP32 3.3V/5V   |
| GND      | GND               | Common Ground required         |

### **BME280**
| Pin Name | ESP32 Pin | Notes   |
|----------|-----------|---------|
| SDA      | GPIO 21   | I2C Bus |
| SCL      | GPIO 22   | I2C Bus |

### **BH1750**
| Pin Name | ESP32 Pin | Notes                      |
|----------|-----------|----------------------------|
| SDA      | GPIO 21   | Parallel with BME280       |
| SCL      | GPIO 22   | Parallel with BME280       |

### **LCD 16x2**
| Pin Name | ESP32 Pin | Notes                      |
|----------|-----------|----------------------------|
| SDA      | GPIO 21   | Parallel with sensors      |
| SCL      | GPIO 22   | Parallel with sensors      |

### **Grain Sensor**
| Pin Name | ESP32 Pin | Notes                     |
|----------|-----------|---------------------------|
| AOUT     | GPIO 34   | Analog Input Only pin     |

---

## 📚 Software Dependencies

Install via **Arduino IDE → Library Manager**:

- Adafruit Unified Sensor  
- Adafruit BME280 Library  
- BH1750 by Christopher Laws  
- LiquidCrystal_I2C by Frank de Brabander  

---

## ⚙️ Configuration & Calibration

Modify these constants before uploading (line 9-24):

```cpp
// 1. GRAIN SENSOR (Capacitive)
// Lower value = wetter. Calibrate with dry vs wet rice.
const int GRAIN_WET_LIMIT = 2000; 

// 2. WEATHER PREDICTION (BME280)
// Sea Level Pressure is ~1013 hPa. Below 1006 = LPA.
const float LOW_PRESSURE_LIMIT = 1006.0;

// 3. DARKNESS (BH1750)
// < 500 Lux means dark sky or heavy clouds.
const float DARK_SKY_LIMIT = 500.0;

// 4. ADMIN NUMBER
const String ADMIN_PHONE = "+639xxxxxxxxx"; 
