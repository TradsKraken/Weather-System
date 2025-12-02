/* * ===================================================================================
 Weather & Rice Grain Quality Monitoring System
 * VERSION: 1.0
 * ===================================================================================
 * * --- CALIBRATION SETTINGS ---
 * Adjust these values based on actual hardware testing.
 */

// 1. GRAIN SENSOR (Capacitive)
// Air value is usually ~3000-4095. Water is ~1000-1500.
// Set LIMIT to the point where rice starts feeling "damp".
const int GRAIN_WET_LIMIT = 2000; 

// 2. WEATHER PREDICTION (BME280)
// Standard Sea Level Pressure is ~1013 hPa.
// A drop below 1006 usually indicates a Low Pressure Area (Rain likely).
const float LOW_PRESSURE_LIMIT = 1006.0;

// 3. DARKNESS (BH1750)
// < 500 Lux means it is very dark (Night or Heavy Clouds).
const float DARK_SKY_LIMIT = 500.0;

// 4. ADMIN NUMBER
const String ADMIN_PHONE = "+639655666230"; 

// ===================================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <LiquidCrystal_I2C.h> 

// Hardware Pin 
#define GSM_TX 17
#define GSM_RX 16
#define MOISTURE_PIN 34 

// Objects
HardwareSerial gsmSerial(2);
Adafruit_BME280 bme;
BH1750 lightSensor;
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// Variables
unsigned long lastAlertTime = 0;
unsigned long lastLCDUpdate = 0; 
const long alertCooldown = 300000; // 5 Minutes between alerts
const long lcdInterval = 2500;     // 2.5 Seconds rotation
int lcdPage = 0;

void setup() {
  Serial.begin(115200); 
  
  // GSM Setup
  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX, GSM_TX);
  gsmSerial.setTimeout(3000); 
  
  Wire.begin(); 
  lcd.init();
  lcd.backlight();
  
  // Startup Screen
  lcd.setCursor(0, 0); lcd.print("GRAIN MONITOR"); 
  lcd.setCursor(0, 1); lcd.print("System Booting..");
  delay(2000); 

  Serial.println("--- SYSTEM BOOT ---");

  // 1. Initialize Sensors
  bool sensorsReady = true;
  if (!bme.begin(0x76)) { Serial.println("BME280 Error"); sensorsReady = false; }
  if (!lightSensor.begin()) { Serial.println("BH1750 Error"); sensorsReady = false; }
  
  if(!sensorsReady) {
    lcd.setCursor(0, 1); lcd.print("Sensor Error!   ");
    delay(2000);
  }

  // 2. Initialize GSM
  lcd.setCursor(0, 1); lcd.print("GSM Init...     ");
  Serial.println("Configuring GSM...");
  
  // Handshake & Config
  sendATCommand("ATE0", 500); // Echo Off
  sendATCommand("AT+CMGF=1", 500); // Text Mode
  sendATCommand("AT+CNMI=1,2,0,0,0", 500); // Direct Message Delivery

  // 3. Network Check
  lcd.setCursor(0, 1); lcd.print("Finding Signal..");
  waitForSignal();

  // 4. Ready
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("System Ready!");
  sendSMS("SYSTEM ONLINE: Monitoring Active.\nSend 'HELP' for commands.");
  delay(1500);
}

void loop() {
  // --- A. DATA ACQUISITION ---
  float t = bme.readTemperature();
  float h = bme.readHumidity(); 
  float p = bme.readPressure() / 100.0F;
  float l = lightSensor.readLightLevel();
  int m = analogRead(MOISTURE_PIN);

  // --- B. SAFETY CHECK (Ignore Sensor Errors) ---
  if (p < 800) p = 1013; // If sensor fails/reads 0, assume normal

  // --- C. INTELLIGENT ALERTS ---
  String alertMsg = "";
  bool dangerState = false;
  int alertType = 0; // 0=Safe, 1=Wet, 2=Rain

  // Priority 1: Grain Wetness
  if (m < GRAIN_WET_LIMIT) {
    alertMsg = "ALERT: Grain Moisture Detected!\nValue: " + String(m) + "\nAction: Check drying area.";
    dangerState = true;
    alertType = 1;
  }
  // Priority 2: Rain Prediction (Softened Language)
  else if (p < LOW_PRESSURE_LIMIT && l < DARK_SKY_LIMIT) {
    alertMsg = "WEATHER ADVISORY:\nLow Pressure (" + String(p,0) + "hPa) detected.\nRain is likely.\nPlease secure the grain.";
    dangerState = true;
    alertType = 2;
  }

  // --- D. LCD INTERFACE (V28 Logic) ---
  updateLCD(t, h, p, l, m, alertType);

  // --- E. SMS ALERT DISPATCHER ---
  if (dangerState) {
    // Check Timer (Don't spam) OR if it's the very first alert
    if (millis() - lastAlertTime > alertCooldown || lastAlertTime == 0) {
      sendSMS(alertMsg);
      lastAlertTime = millis();
      if (lastAlertTime == 0) lastAlertTime = 1; 
    }
  }

  // --- F. COMMAND LISTENER ---
  checkIncomingSMS(t, h, p, l, m);
}

// ================================================================
//   LCD LOGIC
// ================================================================
void updateLCD(float t, float h, float p, float l, int m, int alertType) {
  
  // If Alert Exists: Add Page 4 (The Alert Message)
  int maxPages = (alertType == 0) ? 3 : 4;

  // Normal Cycle Timer
  if (millis() - lastLCDUpdate > lcdInterval) {
    lcdPage++;
    if (lcdPage > maxPages) lcdPage = 0;
    lastLCDUpdate = millis();
    
  } else {
    return; // Wait for timer
  }

  // --- ROW 1: PERMANENT GRAIN STATUS ---
  lcd.setCursor(0,0);
  // Format: G:2000 [SAFE] (Exactly 16 chars or padded)
  String gStat = (m > GRAIN_WET_LIMIT) ? "[SAFE]" : "[WET!]"; // Fixed Variable Name
  lcd.print("G:" + String(m) + " " + gStat + "    "); // Spaces to clear trailing chars

  // --- ROW 2: CYCLING DATA + ALERTS ---
  lcd.setCursor(0,1);
  
  if (lcdPage == 0) {
    lcd.print("Temp: " + String(t, 1) + " C    ");
  } 
  else if (lcdPage == 1) {
    lcd.print("Hum:  " + String(h, 0) + " %    ");
  } 
  else if (lcdPage == 2) {
    lcd.print("Pres: " + String(p, 0) + " hPa ");
  }
  else if (lcdPage == 3) {
    lcd.print("Lght: " + String(l, 0) + " lx  ");
  }
  // This page ONLY appears if alertType > 0
  else if (lcdPage == 4) {
    if (alertType == 1) lcd.print("!! GRAIN WET !! ");
    if (alertType == 2) lcd.print("!! RAIN ALERT !!");
  }
}

// ================================================================
//   COMMAND LOGIC
// ================================================================
void checkIncomingSMS(float t, float h, float p, float l, int m) {
  if (gsmSerial.available()) {
    delay(100); 
    String msg = cleanString(gsmSerial.readString());
    msg.toUpperCase(); // Case insensitive

    Serial.print("Rx: "); Serial.println(msg); // Debug

    // Filter Logic
    if (msg.indexOf("+CMT") == -1) return; // Ignore noise/echoes
    if (msg.indexOf("STATUS:") != -1) return; // Ignore self-replies

    String reply = "";
    bool valid = false;

    if (msg.indexOf("STATUS") != -1) {
      String gStatus = (m > GRAIN_WET_LIMIT) ? "DRY/SAFE" : "WET/UNSAFE";
      reply = "STATUS REPORT:\nGrain: " + gStatus + "\nTemp: " + String(t,1) + "C\nHum: " + String(h,0) + "%\nPres: " + String(p,0) + "hPa";
      valid = true;
    }
    else if (msg.indexOf("GRAIN") != -1) {
      String gStatus = (m > GRAIN_WET_LIMIT) ? "SAFE" : "WET";
      reply = "GRAIN STATUS:\nValue: " + String(m) + "\nCondition: " + gStatus;
      valid = true;
    }
    else if (msg.indexOf("WEATHER") != -1) {
      reply = "WEATHER:\nT: " + String(t,1) + "C\nH: " + String(h,0) + "%\nP: " + String(p,0) + "hPa\nL: " + String(l,0) + "lx";
      valid = true;
    }
    else if (msg.indexOf("HELP") != -1) {
      reply = "COMMANDS:\n- STATUS (Full Report)\n- GRAIN (Check Moisture)\n- WEATHER (Env Data)";
      valid = true;
    }

    if (valid) {
      Serial.println("Command Received. Sending Reply...");
      delay(2000); // Wait for network
      sendSMS(reply);
    }
  }
}

// ================================================================
//   UTILITIES
// ================================================================

void sendSMS(String text) {
  Serial.println("TX SMS...");
  
  // 1. Prepare
  gsmSerial.println("AT+CMGF=1"); delay(200);
  while(gsmSerial.available()) gsmSerial.read(); // Flush

  // 2. Init Send
  gsmSerial.println("AT+CMGS=\"" + ADMIN_PHONE + "\"");
  delay(200);

  // 3. Wait for Prompt
  long t = millis();
  bool ready = false;
  while(millis() - t < 10000) {
    if(gsmSerial.available() && gsmSerial.read() == '>') {
      ready = true; break;
    }
  }

  // 4. Send Content
  if(ready) {
    gsmSerial.print(text);
    delay(200);
    gsmSerial.write(26); // Ctrl+Z
    Serial.println("Sent!");
  } else {
    Serial.println("Fail: Timeout");
    gsmSerial.write(27); // Escape
  }
}

void waitForSignal() {
  int retries = 0;
  bool connected = false;
  while(!connected && retries < 20) {
    gsmSerial.println("AT+CREG?");
    delay(1000);
    if(gsmSerial.available()) {
      String r = gsmSerial.readString();
      if(r.indexOf("0,1") != -1 || r.indexOf("0,5") != -1) connected = true;
    }
    retries++;
  }
}

void sendATCommand(String cmd, int wait) {
  gsmSerial.println(cmd);
  delay(wait);
  while(gsmSerial.available()) gsmSerial.read();
}

String cleanString(String input) {
  String output = "";
  for (int i = 0; i < input.length(); i++) {
    char c = input.charAt(i);
    if (isAlphaNumeric(c) || isPunct(c) || c == ' ' || c == '\n') output += c;
  }
  return output;
}
