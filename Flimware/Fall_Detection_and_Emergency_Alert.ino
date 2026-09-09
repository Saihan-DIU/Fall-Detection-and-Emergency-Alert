#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

#include "secrets.h"

// ============================================================
// CONFIGURATION
// ============================================================

// TextBEE API URL
const String TEXTBEE_API_URL =
    "https://api.textbee.dev/api/v1/gateway/devices/" +
    TEXTBEE_DEVICE_ID +
    "/send-sms";

// Hardware Pins
#define BUZZER_PIN 14
#define BUTTON_PIN 4
#define LED_PIN 15

// MPU6050 I2C
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU6050_ADDR 0x68

// GPS - UART2
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

HardwareSerial GPS_Serial(2);

// MAX30102 I2C Address
#define MAX30102_ADDR 0x57

// ============================================================
// FALL DETECTION PARAMETERS
// ============================================================

#define IMPACT_THRESHOLD 2.0
#define FREE_FALL_THRESHOLD 0.7
#define FREE_FALL_TIME 250
#define IMPACT_WINDOW 400
#define LIE_DOWN_TIME 5000

// ============================================================
// GPS PARAMETERS
// ============================================================

#define GPS_TIMEOUT 120000
#define GPS_UPDATE_INTERVAL 1000

// ============================================================
// SMS CONFIGURATION
// ============================================================

#define SMS_RETRY_COUNT 3
#define SMS_RETRY_DELAY 10000
#define HTTP_TIMEOUT 45000

// ============================================================
// MAX30102 CONFIGURATION
// ============================================================

#define SAMPLING_RATE 100
#define BUFFER_SIZE 100
#define IR_THRESHOLD 50000
#define MAX_FAILED_READINGS 10

// ============================================================
// SYSTEM STATES
// ============================================================

enum State {
  NORMAL,
  ALARM,
  ALERT_SENT,
  GPS_ACQUIRING
};

State currentState = NORMAL;

// ============================================================
// MPU6050 VARIABLES
// ============================================================

float accelX = 0;
float accelY = 0;
float accelZ = 0;

float totalAccel = 0;
float prevAccel = 1.0;

unsigned long fallTime = 0;
unsigned long freeFallStart = 0;

bool isFreeFalling = false;

int impactCount = 0;
unsigned long lastImpactTime = 0;

// ============================================================
// FALL IMPACT DATA
// ============================================================

float maxImpactForce = 0;
float impactAccelX = 0;
float impactAccelY = 0;
float impactAccelZ = 0;

unsigned long impactTime = 0;

// ============================================================
// MAX30102 VARIABLES
// ============================================================

MAX30105 particleSensor;

const byte RATE_SIZE = 4;

byte rates[RATE_SIZE];
byte rateSpot = 0;

long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

int failedReadings = 0;

// ============================================================
// SPO2 VARIABLES
// ============================================================

uint32_t bufferIR[BUFFER_SIZE];
uint32_t bufferRed[BUFFER_SIZE];

int32_t spo2 = 0;
int8_t validSPO2 = 0;

int32_t heartRate = 0;
int8_t validHeartRate = 0;

// ============================================================
// VITAL SIGNS
// ============================================================

float heartRateValue = 75.0;
float temperature = 36.5;
float spo2Value = 98.0;

bool max30102Connected = false;

unsigned long lastVitalUpdate = 0;

// ============================================================
// GPS VARIABLES
// ============================================================

TinyGPSPlus gps;

bool gpsAvailable = false;
bool gpsFix = false;

float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
float gpsSpeed = 0.0;

int gpsSatellites = 0;

String gpsDateTime = "";

unsigned long gpsStartTime = 0;
unsigned long lastGPSUpdate = 0;

// Replace this with an appropriate configured fallback location.
String defaultLocation = "23.793851,90.446956";

// ============================================================
// COMMUNICATION VARIABLES
// ============================================================

bool wifiConnected = false;

int alertRetryCount = 0;

unsigned long lastRetryTime = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  delay(2000);

  Serial.println();
  Serial.println("=== ADVANCED FALL DETECTION WITH MAX30102 ===");
  Serial.println("With real-time heart rate & SpO2 monitoring");

  // GPIO initialization
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize sensors
  initMPU6050();
  initMAX30102();
  initGPS();

  // Wi-Fi
  initWiFi();

  startupBeep();

  Serial.println();
  Serial.println("System Ready!");
  Serial.println("Real-time vital signs & GPS tracking active");
  Serial.println("========================================");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  unsigned long currentTime = millis();

  // ----------------------------------------------------------
  // 1. MPU6050 - every 20 ms
  // ----------------------------------------------------------

  static unsigned long lastMPURead = 0;

  if (currentTime - lastMPURead >= 20) {

    if (readMPU6050()) {
      detectFall(currentTime);
    }

    lastMPURead = currentTime;
  }

  // ----------------------------------------------------------
  // 2. GPS update
  // ----------------------------------------------------------

  static unsigned long lastGPSRead = 0;

  if (currentTime - lastGPSRead >= GPS_UPDATE_INTERVAL) {

    updateGPS();

    lastGPSRead = currentTime;
  }

  // ----------------------------------------------------------
  // 3. MAX30102
  // ----------------------------------------------------------

  updateMAX30102(currentTime);

  // ----------------------------------------------------------
  // 4. Button
  // ----------------------------------------------------------

  checkButton();

  // ----------------------------------------------------------
  // 5. SMS retry
  // ----------------------------------------------------------

  handleSMSRetry(currentTime);

  // ----------------------------------------------------------
  // 6. Status output every 3 seconds
  // ----------------------------------------------------------

  static unsigned long lastPrint = 0;

  if (currentTime - lastPrint >= 3000) {

    printStatus(currentTime);

    lastPrint = currentTime;
  }

  // ----------------------------------------------------------
  // 7. System state handling
  // ----------------------------------------------------------

  handleSystemState(currentTime);

  delay(1);
}

// ============================================================
// MPU6050 FUNCTIONS
// ============================================================

void initMPU6050() {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission();

  delay(100);

  // Accelerometer range: ±8g

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x1C);
  Wire.write(0x10);

  Wire.endTransmission();

  delay(100);

  Serial.println("MPU6050 initialized.");
}

// ------------------------------------------------------------

bool readMPU6050() {

  Wire.beginTransmission(MPU6050_ADDR);

  Wire.write(0x3B);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom(MPU6050_ADDR, 6);

  if (Wire.available() < 6) {
    return false;
  }

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  // ±8g = 4096 LSB/g

  accelX = ax / 4096.0;
  accelY = ay / 4096.0;
  accelZ = az / 4096.0;

  // Total acceleration magnitude

  totalAccel = sqrt(
      accelX * accelX +
      accelY * accelY +
      accelZ * accelZ
  );

  return true;
}

// ============================================================
// MAX30102 FUNCTIONS
// ============================================================

void initMAX30102() {

  Serial.println();
  Serial.println("Initializing MAX30102 Pulse Oximeter...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {

    Serial.println("MAX30102 not found. Check wiring/power!");

    max30102Connected = false;

    // Fallback values
    heartRateValue = 75.0;
    spo2Value = 98.0;
    temperature = 36.5;

    Serial.println("Using simulated vital signs.");

    return;
  }

  Serial.println("MAX30102 found!");

  max30102Connected = true;

  byte ledBrightness = 0x1F;
  byte sampleAverage = 4;
  byte ledMode = 2;

  int sampleRate = SAMPLING_RATE;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(
      ledBrightness,
      sampleAverage,
      ledMode,
      sampleRate,
      pulseWidth,
      adcRange
  );

  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // Clear SpO2 buffers

  for (int i = 0; i < BUFFER_SIZE; i++) {

    bufferIR[i] = 0;
    bufferRed[i] = 0;
  }

  Serial.println("MAX30102 configured for heart rate & SpO2 monitoring.");
}

// ------------------------------------------------------------

void updateMAX30102(unsigned long currentTime) {

  // ----------------------------------------------------------
  // Sensor not connected
  // ----------------------------------------------------------

  if (!max30102Connected) {

    if (currentTime - lastVitalUpdate >= 2000) {

      if (currentState == NORMAL) {

        heartRateValue = random(60, 100);
        spo2Value = random(96, 100);

      } else if (currentState == ALARM) {

        heartRateValue = random(100, 160);
        spo2Value = random(92, 98);
      }

      temperature = 36.5 + (random(-5, 5) / 10.0);

      lastVitalUpdate = currentTime;
    }

    return;
  }

  // ----------------------------------------------------------
  // Read MAX30102
  // ----------------------------------------------------------

  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // ----------------------------------------------------------
  // Finger detection
  // ----------------------------------------------------------

  if (irValue < IR_THRESHOLD) {

    failedReadings++;

    if (failedReadings >= MAX_FAILED_READINGS) {

      heartRateValue = 0;
      spo2Value = 0;

      if (currentTime - lastVitalUpdate >= 5000) {

        Serial.println("Please place finger on sensor.");

        lastVitalUpdate = currentTime;
      }
    }

    return;
  }

  failedReadings = 0;

  // ----------------------------------------------------------
  // Heart rate
  // ----------------------------------------------------------

  if (checkForBeat(irValue)) {

    long delta = currentTime - lastBeat;

    lastBeat = currentTime;

    if (delta > 0) {

      beatsPerMinute = 60.0 / (delta / 1000.0);

      if (beatsPerMinute < 255 &&
          beatsPerMinute > 20) {

        rates[rateSpot++] = (byte)beatsPerMinute;

        rateSpot %= RATE_SIZE;

        beatAvg = 0;

        for (byte x = 0; x < RATE_SIZE; x++) {
          beatAvg += rates[x];
        }

        beatAvg /= RATE_SIZE;

        heartRateValue = beatAvg;
      }
    }
  }

  // ----------------------------------------------------------
  // SpO2
  // ----------------------------------------------------------

  static unsigned long lastSpO2Time = 0;
  static int bufferIndex = 0;

  if (currentTime - lastSpO2Time >= 10) {

    bufferIR[bufferIndex] = (uint32_t)irValue;
    bufferRed[bufferIndex] = (uint32_t)redValue;

    bufferIndex++;

    if (bufferIndex >= BUFFER_SIZE) {

      maxim_heart_rate_and_oxygen_saturation(
          bufferIR,
          BUFFER_SIZE,
          bufferRed,
          &spo2,
          &validSPO2,
          &heartRate,
          &validHeartRate
      );

      if (validSPO2 == 1 && spo2 > 0) {
        spo2Value = spo2;
      }

      if (validHeartRate == 1 && heartRate > 0) {

        if (abs(heartRate - heartRateValue) > 20) {

          heartRateValue =
              (heartRateValue * 0.7) +
              (heartRate * 0.3);
        }
      }

      bufferIndex = 0;
      lastSpO2Time = currentTime;
    }
  }

  // ----------------------------------------------------------
  // Temperature
  // ----------------------------------------------------------

  static unsigned long lastTempTime = 0;

  if (currentTime - lastTempTime >= 10000) {

    temperature =
        particleSensor.readTemperature() + 0.5;

    lastTempTime = currentTime;
  }

  lastVitalUpdate = currentTime;
}

// ============================================================
// GPS FUNCTIONS
// ============================================================

void initGPS() {

  Serial.println("Initializing NEO-6M GPS Module on ESP32...");

  GPS_Serial.begin(
      9600,
      SERIAL_8N1,
      GPS_RX_PIN,
      GPS_TX_PIN
  );

  Serial.println("GPS Serial started at 9600 baud on UART2.");
  Serial.println("Waiting for GPS data...");

  gpsStartTime = millis();

  currentState = GPS_ACQUIRING;
}

// ------------------------------------------------------------

void updateGPS() {

  while (GPS_Serial.available() > 0) {

    gps.encode(GPS_Serial.read());
  }

  if (gps.location.isValid()) {

    if (!gpsFix) {

      gpsFix = true;
      gpsAvailable = true;

      currentState = NORMAL;

      Serial.println();
      Serial.println("GPS Fix Acquired!");
    }

    gpsLatitude = gps.location.lat();
    gpsLongitude = gps.location.lng();

    gpsAltitude = gps.altitude.meters();

    gpsSatellites = gps.satellites.value();

    gpsSpeed = gps.speed.kmph();

    if (gps.date.isValid() &&
        gps.time.isValid()) {

      gpsDateTime =
          String(gps.date.year()) + "-" +
          String(gps.date.month()) + "-" +
          String(gps.date.day()) + " " +
          String(gps.time.hour()) + ":" +
          String(gps.time.minute()) + ":" +
          String(gps.time.second());
    }

    lastGPSUpdate = millis();

  } else {

    if (millis() - gpsStartTime > GPS_TIMEOUT &&
        !gpsFix) {

      Serial.println();
      Serial.println("GPS not available after timeout.");
      Serial.println("Using default location for emergencies.");

      gpsAvailable = false;

      currentState = NORMAL;
    }
  }
}

// ------------------------------------------------------------

String getCurrentLocation() {

  if (gpsFix && gpsAvailable) {

    return String(gpsLatitude, 6) +
           "," +
           String(gpsLongitude, 6);

  } else {

    return defaultLocation;
  }
}

// ------------------------------------------------------------

String getGoogleMapsLink() {

  String location = getCurrentLocation();

  return "https://maps.google.com/?q=" + location;
}

// ============================================================
// FALL DETECTION
// ============================================================

void detectFall(unsigned long currentTime) {

  if (currentState != NORMAL) {
    return;
  }

  // ----------------------------------------------------------
  // Free-fall detection
  // ----------------------------------------------------------

  if (totalAccel < FREE_FALL_THRESHOLD) {

    if (!isFreeFalling) {

      isFreeFalling = true;
      freeFallStart = currentTime;

    } else if (currentTime - freeFallStart > FREE_FALL_TIME) {

      Serial.println();
      Serial.println("FREE FALL DETECTED!");

      triggerFallDetection(currentTime);

      isFreeFalling = false;

      return;
    }

  } else {

    isFreeFalling = false;
  }

  // ----------------------------------------------------------
  // Impact detection
  // ----------------------------------------------------------

  float accelChange =
      abs(totalAccel - prevAccel);

  if (accelChange > IMPACT_THRESHOLD) {

    impactCount++;

    lastImpactTime = currentTime;

    // Record maximum impact

    if (accelChange > maxImpactForce) {

      maxImpactForce = accelChange;

      impactAccelX = accelX;
      impactAccelY = accelY;
      impactAccelZ = accelZ;

      impactTime = currentTime;
    }

    Serial.print("Impact #");
    Serial.print(impactCount);

    Serial.print(" | Force: ");
    Serial.print(accelChange, 2);

    Serial.print("g | Total: ");
    Serial.print(totalAccel, 2);

    Serial.println("g");

    // Multiple impact detection

    if (impactCount >= 2 &&
        (currentTime - lastImpactTime) < IMPACT_WINDOW) {

      Serial.println();
      Serial.println("MULTIPLE IMPACTS DETECTED!");

      triggerFallDetection(currentTime);

      impactCount = 0;

      return;
    }
  }

  if (currentTime - lastImpactTime > 1000) {

    impactCount = 0;
  }

  prevAccel = totalAccel;
}

// ============================================================
// FALL TRIGGER
// ============================================================

void triggerFallDetection(unsigned long currentTime) {

  Serial.println();
  Serial.println("=================================");
  Serial.println("        FALL DETECTED!");
  Serial.println("=================================");

  Serial.print("Max Impact Force: ");
  Serial.print(maxImpactForce, 2);
  Serial.println("g");

  Serial.print("Impact Direction (X,Y,Z): ");

  Serial.print(impactAccelX, 2);
  Serial.print(", ");

  Serial.print(impactAccelY, 2);
  Serial.print(", ");

  Serial.println(impactAccelZ, 2);

  currentState = ALARM;

  fallTime = currentTime;

  impactCount = 0;

  if (max30102Connected) {

    Serial.println(
        "Monitoring vital signs for emergency response..."
    );

  } else {

    heartRateValue = random(120, 180);
    spo2Value = random(88, 95);
  }

  activateAlarm();

  Serial.println(
      "Alarm activated! Press button to cancel."
  );
}

// ============================================================
// WIFI
// ============================================================

void initWiFi() {

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(
      WIFI_SSID,
      WIFI_PASSWORD
  );

  int attempts = 0;

  while (WiFi.status() != WL_CONNECTED &&
         attempts < 20) {

    delay(500);

    Serial.print(".");

    attempts++;

    digitalWrite(
        LED_PIN,
        !digitalRead(LED_PIN)
    );
  }

  if (WiFi.status() == WL_CONNECTED) {

    wifiConnected = true;

    Serial.println();
    Serial.println("WiFi Connected!");

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED_PIN, HIGH);

    delay(1000);

    digitalWrite(LED_PIN, LOW);

  } else {

    wifiConnected = false;

    Serial.println();
    Serial.println("WiFi Connection Failed!");
  }
}

// ============================================================
// TEXTBEE SMS
// ============================================================

bool sendTextBEEAlert(String message) {

  if (!wifiConnected) {

    Serial.println(
        "WiFi not connected - cannot send alert."
    );

    return false;
  }

  Serial.println(
      "Sending SMS via TextBEE API..."
  );

  HTTPClient http;

  http.setTimeout(HTTP_TIMEOUT);
  http.setConnectTimeout(10000);
  http.setReuse(true);

  http.begin(TEXTBEE_API_URL);

  http.addHeader(
      "Content-Type",
      "application/json"
  );

  http.addHeader(
      "x-api-key",
      TEXTBEE_API_KEY
  );

  // JSON payload

  String jsonPayload = "{";

  jsonPayload +=
      "\"recipients\":[\"" +
      EMERGENCY_NUMBER +
      "\"],";

  jsonPayload +=
      "\"message\":\"" +
      escapeJSON(message) +
      "\"";

  jsonPayload += "}";

  Serial.println(
      "Payload prepared, sending..."
  );

  unsigned long startTime = millis();

  int httpResponseCode =
      http.POST(jsonPayload);

  unsigned long requestTime =
      millis() - startTime;

  Serial.print("Request completed in ");

  Serial.print(
      requestTime / 1000.0,
      1
  );

  Serial.print(" seconds. HTTP Code: ");

  Serial.println(httpResponseCode);

  if (httpResponseCode > 0) {

    String response = http.getString();

    Serial.print("API Response: ");
    Serial.println(response);

    http.end();

    if (httpResponseCode == 200 ||
        httpResponseCode == 201) {

      Serial.println(
          "SMS queued successfully!"
      );

      return true;

    } else {

      Serial.print(
          "API returned code: "
      );

      Serial.println(httpResponseCode);

      return false;
    }

  } else {

    Serial.print("HTTP Error: ");

    Serial.println(
        http.errorToString(
            httpResponseCode
        ).c_str()
    );

    if (httpResponseCode == -1 ||
        httpResponseCode == -11) {

      Serial.println(
          "Timeout occurred; SMS may have been sent."
      );
    }

    http.end();

    return false;
  }
}

// ============================================================
// JSON ESCAPE
// ============================================================

String escapeJSON(String input) {

  String output = "";

  for (unsigned int i = 0;
       i < input.length();
       i++) {

    char c = input.charAt(i);

    if (c == '"') {

      output += "\\\"";

    } else if (c == '\\') {

      output += "\\\\";

    } else if (c == '\n') {

      output += "\\n";

    } else if (c == '\r') {

      output += "\\r";

    } else if (c == '\t') {

      output += "\\t";

    } else {

      output += c;
    }
  }

  return output;
}

// ============================================================
// EMERGENCY ALERT
// ============================================================

void sendEmergencyAlert(unsigned long currentTime) {

  Serial.println();
  Serial.println("SENDING EMERGENCY SMS");

  // ----------------------------------------------------------
  // GPS acquisition
  // ----------------------------------------------------------

  if (!gpsFix) {

    Serial.println(
        "No GPS fix, trying to acquire location..."
    );

    currentState = GPS_ACQUIRING;

    gpsStartTime = currentTime;

    unsigned long gpsAttemptStart =
        currentTime;

    while (millis() - gpsAttemptStart < 30000 &&
           !gpsFix) {

      updateGPS();

      delay(1000);

      Serial.print(".");
    }

    if (gpsFix) {

      Serial.println();
      Serial.println(
          "GPS acquired during emergency!"
      );

    } else {

      Serial.println();
      Serial.println(
          "Could not get GPS. Using default location."
      );
    }

    currentState = ALARM;
  }

  // ----------------------------------------------------------
  // Create message
  // ----------------------------------------------------------

  String message =
      createAlertMessage(currentTime);

  Serial.println();
  Serial.println("=== SMS MESSAGE ===");

  Serial.println(message);

  Serial.println("===================");

  Serial.print("Sending to: ");
  Serial.println(EMERGENCY_NUMBER);

  // ----------------------------------------------------------
  // Send
  // ----------------------------------------------------------

  bool alertSent =
      sendTextBEEAlert(message);

  if (alertSent) {

    Serial.println(
        "SMS delivery confirmed!"
    );

    currentState = ALERT_SENT;

    alertRetryCount = 0;

    digitalWrite(
        BUZZER_PIN,
        LOW
    );

    beep(3, 80);

  } else {

    Serial.println(
        "SMS might have been sent."
    );

    if (alertRetryCount < SMS_RETRY_COUNT) {

      alertRetryCount++;

      lastRetryTime = currentTime;

      Serial.print("Retry ");
      Serial.print(alertRetryCount);

      Serial.print(" of ");
      Serial.print(SMS_RETRY_COUNT);

      Serial.println(
          " in 10 seconds..."
      );

    } else {

      Serial.println(
          "Maximum retries reached."
      );

      currentState = ALERT_SENT;

      alertRetryCount = 0;
    }
  }
}

// ============================================================
// CREATE ALERT MESSAGE
// ============================================================

String createAlertMessage(
    unsigned long currentTime
) {

  String message =
      "🚨 EMERGENCY ALERT 🚨\n";

  message +=
      "Name: " +
      PERSON_NAME +
      "\n";

  message += "Condition: ";

  if (currentState == ALARM) {

    message += "Fall Detected";

  } else {

    message += "Manual Alert";
  }

  // ----------------------------------------------------------
  // Time
  // ----------------------------------------------------------

  if (gpsDateTime.length() > 0) {

    message +=
        "\nTime: " +
        gpsDateTime;

  } else {

    message +=
        "\nTime: " +
        formatTime(currentTime);
  }

  // ----------------------------------------------------------
  // Impact
  // ----------------------------------------------------------

  if (maxImpactForce > 0) {

    message +=
        "\nImpact Force: " +
        String(maxImpactForce, 2) +
        "g";

    message +=
        "\nImpact Direction: ";

    message +=
        String(impactAccelX, 2) +
        ",";

    message +=
        String(impactAccelY, 2) +
        ",";

    message +=
        String(impactAccelZ, 2);
  }

  // ----------------------------------------------------------
  // Vital signs
  // ----------------------------------------------------------

  message +=
      "\nHeart Rate: " +
      String(heartRateValue, 0) +
      " BPM";

  if (!max30102Connected) {
    message += " (SIM)";
  }

  message +=
      "\nSpO2: " +
      String(spo2Value, 0) +
      "%";

  if (!max30102Connected) {
    message += " (SIM)";
  }

  message +=
      "\nTemperature: " +
      String(temperature, 1) +
      "°C";

  // ----------------------------------------------------------
  // Location
  // ----------------------------------------------------------

  message +=
      "\nLocation: " +
      getCurrentLocation();

  if (!gpsFix) {
    message += " (Default)";
  }

  message +=
      "\nMaps: " +
      getGoogleMapsLink();

  // ----------------------------------------------------------
  // GPS information
  // ----------------------------------------------------------

  if (gpsFix) {

    message +=
        "\nGPS: " +
        String(gpsSatellites) +
        " satellites";

    message +=
        "\nAltitude: " +
        String(gpsAltitude, 1) +
        "m";

  } else {

    message +=
        "\nGPS: Not available";
  }

  // ----------------------------------------------------------
  // Device
  // ----------------------------------------------------------

  message +=
      "\nDevice: ESP32 Fall Detection System";

  return message;
}

// ============================================================
// SMS RETRY
// ============================================================

void handleSMSRetry(
    unsigned long currentTime
) {

  if (currentState == ALARM &&
      alertRetryCount > 0) {

    if (currentTime - lastRetryTime >=
        SMS_RETRY_DELAY) {

      Serial.println(
          "Retrying SMS send..."
      );

      sendEmergencyAlert(currentTime);
    }
  }
}

// ============================================================
// FORMAT TIME
// ============================================================

String formatTime(
    unsigned long millisTime
) {

  unsigned long seconds =
      millisTime / 1000;

  unsigned long minutes =
      seconds / 60;

  unsigned long hours =
      minutes / 60;

  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  String timeStr =
      String(hours);

  timeStr += ":";

  if (minutes < 10) {
    timeStr += "0";
  }

  timeStr +=
      String(minutes);

  timeStr += ":";

  if (seconds < 10) {
    timeStr += "0";
  }

  timeStr +=
      String(seconds);

  return timeStr;
}

// ============================================================
// SYSTEM STATES
// ============================================================

void handleSystemState(
    unsigned long currentTime
) {

  switch (currentState) {

    // --------------------------------------------------------
    // NORMAL
    // --------------------------------------------------------

    case NORMAL:

      digitalWrite(
          LED_PIN,
          currentTime % 1000 < 500
      );

      break;

    // --------------------------------------------------------
    // GPS ACQUIRING
    // --------------------------------------------------------

    case GPS_ACQUIRING:

      digitalWrite(
          LED_PIN,
          currentTime % 200 < 100
      );

      break;

    // --------------------------------------------------------
    // ALARM
    // --------------------------------------------------------

    case ALARM:

      digitalWrite(
          LED_PIN,
          currentTime % 500 < 250
      );

      // Check whether device is stationary

      if (totalAccel > 0.8 &&
          totalAccel < 1.2) {

        if (currentTime - fallTime >
            LIE_DOWN_TIME) {

          Serial.println(
              "Device stationary - sending alert!"
          );

          sendEmergencyAlert(currentTime);
        }
      }

      // Auto cancel after 60 seconds

      if (currentTime - fallTime >
          60000) {

        Serial.println(
            "Auto-cancelling alarm."
        );

        cancelAlarm();
      }

      break;

    // --------------------------------------------------------
    // ALERT SENT
    // --------------------------------------------------------

    case ALERT_SENT:

      digitalWrite(
          LED_PIN,
          HIGH
      );

      digitalWrite(
          BUZZER_PIN,
          LOW
      );

      maxImpactForce = 0;

      // Auto reset after 2 minutes

      if (currentTime - fallTime >
          120000) {

        resetSystem();
      }

      break;
  }
}

// ============================================================
// BUTTON
// ============================================================

void checkButton() {

  static unsigned long lastPress = 0;

  static bool lastButtonState = HIGH;

  bool currentButtonState =
      digitalRead(BUTTON_PIN);

  unsigned long currentTime =
      millis();

  if (currentButtonState == LOW &&
      lastButtonState == HIGH) {

    if (currentTime - lastPress > 200) {

      handleButtonPress(currentTime);

      lastPress = currentTime;
    }
  }

  lastButtonState =
      currentButtonState;
}

// ------------------------------------------------------------

void handleButtonPress(
    unsigned long currentTime
) {

  Serial.println();
  Serial.println("[Button Pressed]");

  switch (currentState) {

    case NORMAL:

      Serial.println(
          "Manual emergency alert triggered."
      );

      sendEmergencyAlert(currentTime);

      break;

    case GPS_ACQUIRING:

      Serial.println(
          "GPS acquisition cancelled."
      );

      currentState = NORMAL;

      break;

    case ALARM:

      Serial.println(
          "Alarm cancelled by user."
      );

      cancelAlarm();

      break;

    case ALERT_SENT:

      Serial.println(
          "Resetting system."
      );

      resetSystem();

      break;
  }
}

// ============================================================
// ALARM FUNCTIONS
// ============================================================

void activateAlarm() {

  // Initial three-beep pattern

  for (int i = 0; i < 3; i++) {

    digitalWrite(
        BUZZER_PIN,
        HIGH
    );

    digitalWrite(
        LED_PIN,
        HIGH
    );

    delay(300);

    digitalWrite(
        BUZZER_PIN,
        LOW
    );

    digitalWrite(
        LED_PIN,
        LOW
    );

    if (i < 2) {
      delay(300);
    }
  }

  // Continuous alarm

  digitalWrite(
      BUZZER_PIN,
      HIGH
  );
}

// ------------------------------------------------------------

void cancelAlarm() {

  Serial.println(
      "Alarm cancelled."
  );

  currentState = NORMAL;

  impactCount = 0;

  alertRetryCount = 0;

  maxImpactForce = 0;

  digitalWrite(
      BUZZER_PIN,
      LOW
  );

  digitalWrite(
      LED_PIN,
      LOW
  );

  beep(1, 100);
}

// ------------------------------------------------------------

void resetSystem() {

  Serial.println(
      "System reset to normal state."
  );

  currentState = NORMAL;

  impactCount = 0;

  alertRetryCount = 0;

  maxImpactForce = 0;

  digitalWrite(
      BUZZER_PIN,
      LOW
  );

  digitalWrite(
      LED_PIN,
      LOW
  );
}

// ============================================================
// STATUS DISPLAY
// ============================================================

void printStatus(
    unsigned long currentTime
) {

  Serial.print("State: ");

  switch (currentState) {

    case NORMAL:
      Serial.print("NORMAL  ");
      break;

    case GPS_ACQUIRING:
      Serial.print("GPS_ACQ ");
      break;

    case ALARM:
      Serial.print("ALARM   ");
      break;

    case ALERT_SENT:
      Serial.print("ALERT   ");
      break;
  }

  // Acceleration

  Serial.print("| Accel: ");

  Serial.print(
      totalAccel,
      2
  );

  Serial.print("g");

  // Impact

  if (maxImpactForce > 0) {

    Serial.print(" | Impact: ");

    Serial.print(
        maxImpactForce,
        2
    );

    Serial.print("g");
  }

  // Heart rate

  Serial.print(" | HR: ");

  if (heartRateValue > 0) {

    Serial.print(
        heartRateValue,
        0
    );

    Serial.print("bpm");

  } else {

    Serial.print("---");
  }

  // SpO2

  Serial.print(" | SpO2: ");

  if (spo2Value > 0) {

    Serial.print(
        spo2Value,
        0
    );

    Serial.print("%");

  } else {

    Serial.print("---");
  }

  // Temperature

  Serial.print(" | Temp: ");

  Serial.print(
      temperature,
      1
  );

  Serial.print("°C");

  // MAX30102

  Serial.print(" | MAX: ");

  Serial.print(
      max30102Connected
          ? "ON"
          : "OFF"
  );

  // GPS

  Serial.print(" | GPS: ");

  Serial.print(
      gpsFix
          ? "FIX"
          : "NO"
  );

  // Wi-Fi

  Serial.print(" | WiFi: ");

  Serial.print(
      wifiConnected
          ? "ON"
          : "OFF"
  );

  // Alarm duration

  if (currentState == ALARM) {

    Serial.print(" | Alarm: ");

    Serial.print(
        (currentTime - fallTime) / 1000
    );

    Serial.print("s");
  }

  // GPS coordinates

  if (gpsFix) {

    Serial.println();

    Serial.print("GPS: ");

    Serial.print(
        gpsLatitude,
        6
    );

    Serial.print(",");

    Serial.print(
        gpsLongitude,
        6
    );

    Serial.print(" | Sat: ");

    Serial.print(
        gpsSatellites
    );

    Serial.print(" | Alt: ");

    Serial.print(
        gpsAltitude,
        1
    );

    Serial.print("m");
  }

  Serial.println();
}

// ============================================================
// UTILITY
// ============================================================

void startupBeep() {

  for (int i = 0; i < 2; i++) {

    digitalWrite(
        BUZZER_PIN,
        HIGH
    );

    digitalWrite(
        LED_PIN,
        HIGH
    );

    delay(100);

    digitalWrite(
        BUZZER_PIN,
        LOW
    );

    digitalWrite(
        LED_PIN,
        LOW
    );

    delay(100);
  }
}

// ------------------------------------------------------------

void beep(
    int times,
    int duration
) {

  for (int i = 0; i < times; i++) {

    digitalWrite(
        BUZZER_PIN,
        HIGH
    );

    delay(duration);

    digitalWrite(
        BUZZER_PIN,
        LOW
    );

    if (i < times - 1) {
      delay(duration);
    }
  }
}