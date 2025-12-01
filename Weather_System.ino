/* * Project: Rice Grain & Weather Monitoring System (SMS Based) - VERSION 9 (Responsive Loop)
 * Board: ESP32 Dev Module
 * Modules: SIM800L, BME280, BH1750, Capacitive Soil Sensor
 * Date: 2025
 * * =============================================================
 * --- CALIBRATION GUIDE / SETTINGS INSTRUCTIONS ---
 * =============================================================
 * * 1. HOW TO CALIBRATE GRAIN SENSOR (Moisture):
 * - Open Serial Monitor (Baud: 115200).
 * - Look at the "M:" (Moisture) value.
 * - Test 1 (DRY): Hold the sensor in dry air. Value should be high (e.g., 3000 - 4095).
 * - Test 2 (WET): Dip the tip in water or wet soil. Value should go low (e.g., 1000 - 1500).
 * - ADJUSTMENT: Change 'moistureLimit' below. 
 * Set it to a number between your Dry and Wet values.
 * (Example: If Dry is 3500 and Wet is 1200, set limit to 2000).
 * * 2. HOW TO CALIBRATE STORM WARNING (Pressure):
 * - Look at the "P:" (Pressure) value in Serial Monitor.
 * - Normal pressure is usually around 1008 hPa to 1014 hPa.
 * - A storm or rain usually drops the pressure.
 * - ADJUSTMENT: Set 'stormPressure' about 3-5 hPa lower than your current normal reading.
 * (Example: If normal is 1010, set threshold to 1006).
 * * 3. HOW TO CALIBRATE DARKNESS (Light):
 * - Look at the "L:" (Light) value.
 * - Cover the sensor with your hand. Value should drop near 0.
 * - ADJUSTMENT: Set 'darkThreshold'. 
 * 2000 is good for "Cloudy". 500 is "Very Dark/Night".
 * =============================================================
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h> 

// System Settings
String ADMIN_PHONE = "+639655666230"; // Your Number

// Thresholds for Alerts (See Calibration Guide above)
int moistureLimit = 2000;       // Lower = Wetter
float stormPressure = 1006.0;   // Lower = Storm
float darkThreshold = 2000.0;   // Lower = Darker

// Pin Definitions
#define GSM_TX 17
#define GSM_RX 16
#define MOISTURE_PIN 34 

// Objects
HardwareSerial gsmSerial(2);
Adafruit_BME280 bme;
BH1750 lightSensor;
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Variables for Timers
unsigned long lastAlertTime = 0;
unsigned long lastLCDUpdate = 0; // For non-blocking LCD

// TIMING CONFIGURATION (Milliseconds)
const long alertCooldown = 60000;   // 1 Minute delay between ALERTS
const long lcdInterval = 2000;      // 2 Seconds per screen change

int displayState = 0;

void setup() {
  Serial.begin(115200);
  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  Wire.begin(); 

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // --- START SELF-CHECK SEQUENCE ---
  lcd.setCursor(0, 0); lcd.print("SYSTEM V9 (FAST)"); 
  lcd.setCursor(0, 1); lcd.print("Starting...");
  delay(2000); 

  Serial.println("--- STARTING SELF-CHECK (V9) ---");

  // 1. CHECK SENSORS
  lcd.clear(); lcd.print("1. Checking Sensors");
  bool sensorsOK = true;
  
  if (!bme.begin(0x76)) { 
    lcd.setCursor(0, 1); lcd.print("Err: BME280     ");
    Serial.println("BME280 Not Found");
    sensorsOK = false;
    delay(2000);
  }
  if (!lightSensor.begin()) {
    lcd.setCursor(0, 1); lcd.print("Err: BH1750     ");
    Serial.println("BH1750 Not Found");
    sensorsOK = false;
    delay(2000);
  }
  
  if(sensorsOK) {
    lcd.setCursor(0, 1); lcd.print("Sensors: OK!    ");
    Serial.println("Sensors OK.");
    delay(1500);
  }

  // 2. CHECK GSM MODULE & SIGNAL
  lcd.clear(); lcd.print("2. Checking Net");
  lcd.setCursor(0, 1); lcd.print("Wait Signal...  ");
  Serial.println("Waiting for GSM Network...");
  
  // Handshake
  gsmSerial.println("AT");
  delay(500);
  
  // Wait for Registration (CREG: 0,1 or 0,5)
  bool networkFound = false;
  int retries = 0;
  
  while(!networkFound && retries < 20) { // Try for 20 seconds max
    gsmSerial.println("AT+CREG?");
    delay(1000);
    if(gsmSerial.available()) {
      String r = gsmSerial.readString();
      Serial.print("Net Status: "); Serial.println(r);
      
      // Check if registered (0,1 is home, 0,5 is roaming)
      if(r.indexOf("0,1") != -1 || r.indexOf("0,5") != -1) {
        networkFound = true;
      }
    }
    retries++;
    if(retries % 2 == 0) { lcd.setCursor(14,1); lcd.print(".."); } 
    else { lcd.setCursor(14,1); lcd.print("  "); }
  }

  if(networkFound) {
    lcd.setCursor(0, 1); lcd.print("Signal: OK!     ");
    Serial.println("Network Registered!");
  } else {
    lcd.setCursor(0, 1); lcd.print("No Signal!      ");
    Serial.println("Network Fail - Check SIM/Antenna");
    delay(2000);
  }
  delay(1500);

  // 3. CONFIGURE GSM
  lcd.clear(); lcd.print("3. Config GSM");
  lcd.setCursor(0, 1); lcd.print("Setting Text Mode");
  gsmSerial.println("AT+CMGF=1"); 
  delay(500);
  gsmSerial.println("AT+CNMI=1,2,0,0,0"); // IMPORTANT: Delivers SMS directly to buffer
  delay(1000);

  // 4. SEND TEST SMS (Only if network found)
  if(networkFound) {
    lcd.clear(); lcd.print("4. Sending Test");
    lcd.setCursor(0, 1); lcd.print("Sending SMS...  ");
    sendSMS("SYSTEM ONLINE: Ready for commands (STATUS, TEMP, GRAIN).");
    lcd.setCursor(0, 1); lcd.print("Check Phone!    ");
  } else {
    lcd.clear(); lcd.print("Skip Test SMS");
    lcd.setCursor(0, 1); lcd.print("No Network      ");
  }
  delay(2000);

  // FINISH
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("System Ready!");
  delay(1000);
}

void loop() {
  // --- NON-BLOCKING LOOP START ---
  
  // 1. Read Sensors
  float t = bme.readTemperature();
  float h = bme.readHumidity(); 
  float p = bme.readPressure() / 100.0F;
  float l = lightSensor.readLightLevel();
  int m = analogRead(MOISTURE_PIN);

  // Debugging (Only print occasionally to avoid spamming serial, or remove if annoying)
  // Serial.print("T:"); Serial.print(t); Serial.print(" M:"); Serial.println(m);

  // 2. ALERT CHECKING
  String smsMessage = "";
  bool triggerAlert = false;
  int alertType = 0; // 0=None, 1=Grain Wet, 2=Storm

  if (m < moistureLimit) {
    smsMessage = "ALERT: GRAIN WET! Val: " + String(m);
    triggerAlert = true;
    alertType = 1; 
  }
  else if (p < stormPressure && l < darkThreshold) {
    smsMessage = "WARNING: STORM ALERT! Low Pressure & Dark.";
    triggerAlert = true;
    alertType = 2;
  }
  
  // 3. DISPLAY LOGIC (Non-blocking Timer)
  if (millis() - lastLCDUpdate > lcdInterval) {
    updateDisplayCycle(t, h, m, alertType);
    lastLCDUpdate = millis();
  }

  // 4. SMS LOGIC (ALERTS ONLY)
  if (triggerAlert) {
    if (millis() - lastAlertTime > alertCooldown || lastAlertTime == 0) {
      sendSMS(smsMessage);
      lastAlertTime = millis();
      if (lastAlertTime == 0) lastAlertTime = 1; 
    }
  }
  
  // 5. CHECK INCOMING SMS COMMANDS (Runs on every loop - VERY FAST)
  readGSM(t, h, p, l, m);
  
  // NO DELAY HERE! The loop runs at full speed.
}

void updateDisplayCycle(float t, float h, int m, int alertType) {
  
  int maxPages = (alertType == 0) ? 2 : 3;

  displayState++;
  if (displayState > maxPages) displayState = 0;

  // We don't clear() every time to reduce flicker, 
  // only partial updates or clear if alert changes drastically.
  // Ideally, just overwrite. But clear() is safer for variable text lengths.
  lcd.clear(); 

  if (displayState == 3) {
    if (alertType == 1) {
      lcd.setCursor(0,0); lcd.print("!! GRAIN WET !!");
      lcd.setCursor(0,1); lcd.print("Val: " + String(m));
    }
    else if (alertType == 2) {
      lcd.setCursor(0,0); lcd.print("! STORM ALERT !");
      lcd.setCursor(0,1); lcd.print("Low Pres & Dark");
    }
    return; 
  }

  lcd.setCursor(0,0); 
  if (alertType == 1)      lcd.print("Status: GRAIN WET");
  else if (alertType == 2) lcd.print("Status: STORM!");
  else                     lcd.print("Status: GRAIN OK");

  lcd.setCursor(0,1);
  if (displayState == 0) {
    lcd.print("Moisture: " + String(m)); 
  }
  else if (displayState == 1) {
    lcd.print("Temp: " + String(t, 1) + " C");
  }
  else if (displayState == 2) {
    lcd.print("Humidity: " + String(h, 0) + " %");
  }
}

void sendSMS(String text) {
  Serial.println("--- Sending SMS ---");
  
  gsmSerial.println("AT+CMGS=\"" + ADMIN_PHONE + "\"");
  delay(500);
  
  while(gsmSerial.available()) {
    Serial.write(gsmSerial.read()); 
  }
  
  gsmSerial.print(text);
  delay(100);
  gsmSerial.write(26); 
  
  Serial.println("\nMessage Body Sent. Waiting for OK...");
  
  // Wait for Confirmation
  long waitStart = millis();
  while(millis() - waitStart < 5000) { 
    while(gsmSerial.available()) {
      Serial.write(gsmSerial.read()); 
    }
  }
  Serial.println("\n--- End SMS Attempt ---");
}

void readGSM(float t, float h, float p, float l, int m) {
  // This runs very fast now.
  if (gsmSerial.available()) {
    // Wait a tiny bit for the full message to arrive in buffer
    delay(100); 
    
    String msg = gsmSerial.readString();
    msg.toUpperCase(); 
    
    // --- DEBUGGING: PRINT RECEIVED TEXT TO SERIAL MONITOR ---
    Serial.print("[GSM IN]: ");
    Serial.println(msg); 
    // --------------------------------------------------------

    // Clean up message (remove whitespace/newlines if needed, though indexOf handles it)
    
    // --- COMMAND PARSING ---
    if (msg.indexOf("STATUS") != -1) {
      Serial.println("Command Found: STATUS");
      String stat = "STATUS: T:" + String(t) + " P:" + String(p) + " L:" + String(l) + " M:" + String(m);
      sendSMS(stat);
    }
    else if (msg.indexOf("TEMP") != -1) {
      Serial.println("Command Found: TEMP");
      sendSMS("CURRENT TEMP: " + String(t) + " C");
    }
    else if (msg.indexOf("HUMIDITY") != -1) {
      Serial.println("Command Found: HUMIDITY");
      sendSMS("CURRENT HUMIDITY: " + String(h) + " %");
    }
    else if (msg.indexOf("GRAIN") != -1) {
      Serial.println("Command Found: GRAIN");
      String status = (m < moistureLimit) ? "WET (DANGER)" : "DRY (SAFE)";
      sendSMS("GRAIN MOISTURE: " + String(m) + "\nState: " + status);
    }
    else if (msg.indexOf("PRESSURE") != -1) {
      Serial.println("Command Found: PRESSURE");
      sendSMS("AIR PRESSURE: " + String(p) + " hPa");
    }
  }
}