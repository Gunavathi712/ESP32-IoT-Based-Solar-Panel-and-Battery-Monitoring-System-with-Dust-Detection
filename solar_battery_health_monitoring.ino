#include <WiFi.h>
#include <Wire.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include <BH1750.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

// ---------------- WIFI & THINGSPEAK ----------------
const char* WIFI_SSID     = "solar";
const char* WIFI_PASSWORD = "12345678";

unsigned long CHANNEL_ID      = 3557384UL;
const char*   WRITE_API_KEY   = "LCENL10W04PLV5XC";

WiFiClient client;

// ---------------- PIN DEFINITIONS ----------------
#define PIN_DHT             15
#define PIN_LDR             34   // LDR module AO pin
#define PIN_I2C_SDA         21   // Shared by BH1750 and 16x2 I2C LCD
#define PIN_I2C_SCL         22   // Shared by BH1750 and 16x2 I2C LCD
#define PIN_BAT_VOLTAGE     32   // Battery voltage sensor
#define PIN_BAT_CURRENT     35   // ACS712 current sensor
#define PIN_DS18B20         27   // Battery temp probe (4.7k pull-up to 3.3V)
#define PIN_BUZZER          23   // Buzzer pin

// ---------------- SENSOR & ALERT CONFIG ----------------
#define DHTTYPE             DHT11
#define LCD_I2C_ADDR        0x27  // Change to 0x3F if LCD stays blank

// Active-HIGH Buzzer: LOW = Silent (OFF), HIGH = Continuous ON
#define BUZZER_ACTIVE_HIGH  true

// High Temperature Alert Thresholds (in Celsius)
#define ENV_TEMP_ALERT_C    35.0  // Alert if environment temp >= 35 C
#define BAT_TEMP_ALERT_C    35.0  // Alert if battery temp >= 35 C

// Dust Detection calibrated for your LDR readings:
#define LDR_ROOM_DUST_THRESHOLD    1750
#define LDR_BRIGHT_DUST_THRESHOLD  400
#define BRIGHT_LUX_LEVEL           300.0
#define MIN_LUX_FOR_DUST           3.0

// Battery Voltage & Current calibration
#define VOLTAGE_DIVIDER_RATIO 5.0
#define ACS_SENSITIVITY_MV_A  100.0
#define ACS_DIVIDER_RATIO     1.0
float   acsZeroMilliVolts   = 2500.0;

// ---------------- OBJECTS ----------------
DHT dht(PIN_DHT, DHTTYPE);
BH1750 lightMeter(0x23);
OneWire oneWire(PIN_DS18B20);
DallasTemperature batteryTempSensor(&oneWire);
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2);

// ---------------- TIMING & STATE ----------------
unsigned long lastReadMs   = 0;
unsigned long lastScreenMs = 0;
unsigned long lastCloudMs  = 0;
uint8_t currentScreen      = 0;
bool    bh1750Ok           = false;

float envTempC = NAN, envHum = NAN, lux = -1, batVolt = 0, batCurr = 0, batTempC = DEVICE_DISCONNECTED_C;
int   ldrRaw = 0;
int   ldrBrightPct = 0;
bool  dustDetected = false;
bool  highEnvTemp  = false;
bool  highBatTemp  = false;

void setBuzzer(bool turnOn) {
  if (BUZZER_ACTIVE_HIGH) {
    digitalWrite(PIN_BUZZER, turnOn ? HIGH : LOW);
  } else {
    digitalWrite(PIN_BUZZER, turnOn ? LOW : HIGH);
  }
}

float readAvgMilliVolts(uint8_t pin, int samples = 30) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogReadMilliVolts(pin);
    delay(2);
  }
  return (float)sum / samples;
}

void connectWiFiStartup() {
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 10000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  lcd.setCursor(0, 1);
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi Connected! IP: ");
    Serial.println(WiFi.localIP());
    lcd.print("WiFi Connected!");
  } else {
    Serial.println("WiFi not connected (will retry in background).");
    lcd.print("Offline Mode");
  }
  delay(800);
}

void setup() {
  Serial.begin(115200);

  // Keep buzzer OFF from startup
  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_BAT_VOLTAGE, INPUT);
  pinMode(PIN_BAT_CURRENT, INPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Booting..");

  dht.begin();
  batteryTempSensor.begin();
  bh1750Ok = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  if (!bh1750Ok) {
    Serial.println("Warning: BH1750 not found at 0x23!");
  }

  ThingSpeak.begin(client);
  connectWiFiStartup();

  // Calibrate 0A AFTER Wi-Fi turns ON (keep load OFF during startup)
  delay(500);
  acsZeroMilliVolts = readAvgMilliVolts(PIN_BAT_CURRENT, 100) * ACS_DIVIDER_RATIO;

  readAllSensors();
  lastReadMs   = millis();
  lastScreenMs = millis();
  lastCloudMs  = millis();

  lcd.clear();
  Serial.println("ESP32 Monitoring Started");
}

void readAllSensors() {
  // 1. DHT11 Environment Temp & Humidity
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) envTempC = t;
  if (!isnan(h)) envHum   = h;

  // 2. BH1750 Light Intensity (Lux)
  if (bh1750Ok) {
    lux = lightMeter.readLightLevel();
  } else {
    lux = -1;
  }

  // 3. LDR Reading + Panel Dust Check
  ldrRaw = analogRead(PIN_LDR);
  ldrBrightPct = constrain(map(ldrRaw, 4095, 0, 0, 100), 0, 100);

  if (bh1750Ok && lux >= 0) {
    if (lux < MIN_LUX_FOR_DUST) {
      dustDetected = false;
    } else if (lux >= BRIGHT_LUX_LEVEL) {
      dustDetected = (ldrRaw > LDR_BRIGHT_DUST_THRESHOLD);
    } else {
      dustDetected = (ldrRaw > LDR_ROOM_DUST_THRESHOLD);
    }
  } else {
    dustDetected = (ldrRaw > LDR_ROOM_DUST_THRESHOLD);
  }

  // 4. Battery Voltage (GPIO 32)
  float pinMv = readAvgMilliVolts(PIN_BAT_VOLTAGE);
  batVolt = (pinMv / 1000.0) * VOLTAGE_DIVIDER_RATIO;

  // 5. Battery Current (GPIO 35) - clean 0.00A at no load, always positive
  float sensorMv = readAvgMilliVolts(PIN_BAT_CURRENT, 50) * ACS_DIVIDER_RATIO;
  batCurr = fabs((sensorMv - acsZeroMilliVolts) / ACS_SENSITIVITY_MV_A);
  if (batCurr < 0.20) batCurr = 0.0;

  // 6. DS18B20 Battery Temperature (GPIO 27) - ignore -127C and 85C startup error
  batteryTempSensor.requestTemperatures();
  float rawBatTemp = batteryTempSensor.getTempCByIndex(0);
  if (rawBatTemp == DEVICE_DISCONNECTED_C) {
    batTempC = DEVICE_DISCONNECTED_C;
  } else if (rawBatTemp != 85.0 && rawBatTemp > -55.0 && rawBatTemp < 125.0) {
    batTempC = rawBatTemp;
  }

  // 7. Check Alert Flags
  highEnvTemp = (!isnan(envTempC) && envTempC >= ENV_TEMP_ALERT_C);
  highBatTemp = (batTempC != DEVICE_DISCONNECTED_C && batTempC >= BAT_TEMP_ALERT_C);

  const char* alertReason = "OFF";
  if (dustDetected)     alertReason = "DUST ALERT";
  else if (highEnvTemp) alertReason = "ENV TEMP HIGH";
  else if (highBatTemp) alertReason = "BT TEMP HIGH";

  Serial.printf("Env: %.1fC (%.0f%%) | Lux: %.1f | LDR: %d (%d%%) -> Panel: %s | Bat: %.2fV, %.2fA, %.1fC | Buzzer: %s\n",
                envTempC, envHum, lux, ldrRaw, ldrBrightPct,
                dustDetected ? "DUST" : "CLEAN",
                batVolt, batCurr, batTempC, alertReason);
}

void updateLCD() {
  char line0[17], line1[17];

  if (currentScreen == 0) {
    // Page 1: Environment & Light
    if (isnan(envTempC)) {
      snprintf(line0, sizeof(line0), "Env: DHT Error");
    } else {
      snprintf(line0, sizeof(line0), "Env:%.1fC H:%.0f%%", envTempC, envHum);
    }
    snprintf(line1, sizeof(line1), "Lux:%.0f lx", (lux < 0) ? 0 : lux);
  } 
  else if (currentScreen == 1) {
    // Page 2: LDR & Panel Status
    snprintf(line0, sizeof(line0), "LDR:%d (%d%%)", ldrRaw, ldrBrightPct);
    snprintf(line1, sizeof(line1), "Panel: %s", dustDetected ? "DUST" : "CLEAN");
  } 
  else if (currentScreen == 2) {
    // Page 3: Battery Voltage, Current & Temp
    snprintf(line0, sizeof(line0), "B:%.1fV I:%.2fA", batVolt, batCurr);
    if (batTempC == DEVICE_DISCONNECTED_C) {
      snprintf(line1, sizeof(line1), "BatTemp: Sensor!");
    } else {
      snprintf(line1, sizeof(line1), "BatTemp: %.1fC", batTempC);
    }
  } 
  else {
    // Page 4 (Last Page): Buzzer Status & Alert Reason
    int alertCount = (dustDetected ? 1 : 0) + (highEnvTemp ? 1 : 0) + (highBatTemp ? 1 : 0);

    if (alertCount == 0) {
      snprintf(line0, sizeof(line0), "Buzzer: OFF");
      snprintf(line1, sizeof(line1), "Status: NORMAL");
    } else if (alertCount == 1) {
      snprintf(line0, sizeof(line0), "Buzzer: ON!");
      if (dustDetected)     snprintf(line1, sizeof(line1), "DUST ALERT");
      else if (highEnvTemp) snprintf(line1, sizeof(line1), "ENV TEMP HIGH");
      else                  snprintf(line1, sizeof(line1), "BT TEMP HIGH");
    } else if (alertCount == 2) {
      if (dustDetected && highEnvTemp) {
        snprintf(line0, sizeof(line0), "DUST ALERT");
        snprintf(line1, sizeof(line1), "ENV TEMP HIGH");
      } else if (dustDetected && highBatTemp) {
        snprintf(line0, sizeof(line0), "DUST ALERT");
        snprintf(line1, sizeof(line1), "BT TEMP HIGH");
      } else {
        snprintf(line0, sizeof(line0), "ENV TEMP HIGH");
        snprintf(line1, sizeof(line1), "BT TEMP HIGH");
      }
    } else {
      snprintf(line0, sizeof(line0), "DUST ALERT +");
      snprintf(line1, sizeof(line1), "ENV & BT TEMP HI");
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1);
}

void uploadThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    return;
  }

  if (!isnan(envTempC)) ThingSpeak.setField(1, envTempC);
  if (!isnan(envHum))   ThingSpeak.setField(2, envHum);
  if (lux >= 0)         ThingSpeak.setField(3, lux);
  ThingSpeak.setField(4, batVolt);
  ThingSpeak.setField(5, batCurr);
  if (batTempC != DEVICE_DISCONNECTED_C && batTempC > -55.0) {
    ThingSpeak.setField(6, batTempC);
  }
  ThingSpeak.setField(7, ldrRaw);
  ThingSpeak.setField(8, dustDetected ? 1 : 0);

  int statusCode = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);
  if (statusCode == 200) {
    Serial.println("ThingSpeak Upload: OK (200)");
  } else {
    Serial.printf("ThingSpeak Upload Failed! Error code: %d\n", statusCode);
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastReadMs >= 1500) {
    lastReadMs = now;
    readAllSensors();
  }

  if (now - lastScreenMs >= 2500) {
    lastScreenMs = now;
    updateLCD();
    currentScreen = (currentScreen + 1) % 4; // Cycles through all 4 LCD pages
  }

  if (now - lastCloudMs >= 20000) {
    lastCloudMs = now;
    uploadThingSpeak();
  }

  // Buzzer: Continuous ON when DUST or High Temp; OFF otherwise
  if (dustDetected || highEnvTemp || highBatTemp) {
    setBuzzer(true);
  } else {
    setBuzzer(false);
  }
}