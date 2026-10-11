#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "Green Pod Virtual"
#define BLYNK_AUTH_TOKEN    "YOUR_DEVICE_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// Wokwi virtual Wi-Fi
char ssid[] = "Wokwi-GUEST";
char pass[] = "";

BlynkTimer timer;
Servo capsuleGate;
Preferences prefs;

// Virtual wiring: motor indicators
const int LEFT_FWD  = 25;
const int LEFT_REV  = 26;
const int RIGHT_FWD = 16;
const int RIGHT_REV = 17;
const int DRILL_LED = 19;
const int DRIVER_LED = 13;
const int SERVO_PIN = 18;
const int ESTOP_PIN = 33;

// Dashboard commands
int driveCommand = 0;
bool drillRequested = false;
bool autoRequested = false;
bool estopActive = true;

// Adjustable virtual sensor inputs
float simMoisture = 45.0;
float simPH = 6.5;
float simEC = 1.2;
float simTemperature = 26.0;
float simHumidity = 60.0;

// Rover estimates: example values, not real measurements
float areaM2 = 0.0;
float roverSpeedMps = 0.15;
float workingWidthM = 0.30;

uint32_t capsulesPlanted = 0;
uint32_t capsulesRemaining = 100;

unsigned long lastAreaUpdate = 0;
bool releaseBusy = false;

// ---------- Safety and virtual outputs ----------

bool safeToActuate() {
  return Blynk.connected() && !estopActive;
}

void allIndicatorsOff() {
  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(LEFT_REV, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(RIGHT_REV, LOW);
}

void stopRover() {
  driveCommand = 0;
  allIndicatorsOff();
}

void stopAll() {
  stopRover();
  drillRequested = false;
  digitalWrite(DRILL_LED, LOW);
  capsuleGate.write(0);
}

void updateEmergencyStop() {
  // Virtual button connects GPIO33 to GND when pressed.
  // INPUT_PULLUP: HIGH = released, LOW = pressed.
  bool pressed = (digitalRead(ESTOP_PIN) == LOW);

  if (pressed) {
    estopActive = true;
    stopAll();
  } else {
    estopActive = false;
  }
}

void setDrive(int cmd) {
  updateEmergencyStop();

  if (!safeToActuate() || cmd < 0 || cmd > 4 || autoRequested) {
    stopRover();
    return;
  }

  driveCommand = cmd;
  allIndicatorsOff();

  switch (cmd) {
    case 0: // Stop
      break;

    case 1: // Forward: both wheels forward
      digitalWrite(LEFT_FWD, HIGH);
      digitalWrite(RIGHT_FWD, HIGH);
      break;

    case 2: // Reverse: both wheels reverse
      digitalWrite(LEFT_REV, HIGH);
      digitalWrite(RIGHT_REV, HIGH);
      break;

    case 3: // Left: right wheel forward
      digitalWrite(RIGHT_FWD, HIGH);
      break;

    case 4: // Right: left wheel forward
      digitalWrite(LEFT_FWD, HIGH);
      break;
  }
}

int getStatus() {
  if (estopActive) return 3;
  if (!Blynk.connected()) return 4;
  if (autoRequested) return 5;
  if (drillRequested) return 2;
  if (driveCommand != 0) return 1;
  return 0;
}

// ---------- Area and capsule tracking ----------

void updateArea() {
  unsigned long now = millis();

  if (lastAreaUpdate == 0) {
    lastAreaUpdate = now;
    return;
  }

  unsigned long elapsed = now - lastAreaUpdate;
  lastAreaUpdate = now;

  if (safeToActuate() &&
      (driveCommand == 1 || driveCommand == 2)) {
    areaM2 += roverSpeedMps * workingWidthM *
              (elapsed / 1000.0f);
  }
}

void releaseOneCapsule() {
  updateEmergencyStop();

  if (!safeToActuate() || driveCommand != 0 ||
      autoRequested || releaseBusy || capsulesRemaining == 0) {
    return;
  }

  releaseBusy = true;

  // Virtual gate movement. Adjust these angles for your model.
  capsuleGate.write(75);
  delay(400);
  capsuleGate.write(0);
  delay(300);

  capsulesRemaining--;
  capsulesPlanted++;

  prefs.putUInt("remaining", capsulesRemaining);
  prefs.putUInt("planted", capsulesPlanted);

  releaseBusy = false;
}

// ---------- Blynk commands ----------

BLYNK_WRITE(V0) {
  setDrive(param.asInt());
}

BLYNK_WRITE(V1) {
  updateEmergencyStop();

  bool requested = (param.asInt() == 1);

  if (safeToActuate() && requested && !autoRequested) {
    drillRequested = true;
    digitalWrite(DRILL_LED, HIGH);
  } else {
    drillRequested = false;
    digitalWrite(DRILL_LED, LOW);
  }
}

BLYNK_WRITE(V2) {
  if (param.asInt() == 1) {
    releaseOneCapsule();
    Blynk.virtualWrite(V2, 0);
  }
}

BLYNK_WRITE(V3) {
  autoRequested = (param.asInt() == 1);

  // This is only a request/status flag.
  // No autonomous planting algorithm is implemented.
  if (autoRequested) {
    stopAll();
  }
}

BLYNK_WRITE(V4) {
  if (param.asInt() != 1) return;

  updateEmergencyStop();
  if (!safeToActuate()) {
    Blynk.virtualWrite(V4, 0);
    return;
  }

  areaM2 = 0;
  capsulesPlanted = 0;
  prefs.putFloat("area", areaM2);
  prefs.putUInt("planted", capsulesPlanted);

  Blynk.virtualWrite(V4, 0);
}

BLYNK_WRITE(V5) {
  updateEmergencyStop();
  if (!safeToActuate()) return;

  areaM2 = constrain(param.asDouble(), 0.0, 1000.0);
  prefs.putFloat("area", areaM2);
}

// Dashboard simulation sliders
BLYNK_WRITE(V20) {
  simMoisture = constrain(param.asDouble(), 0.0, 100.0);
}
BLYNK_WRITE(V21) {
  simPH = constrain(param.asDouble(), 0.0, 14.0);
}
BLYNK_WRITE(V22) {
  simEC = constrain(param.asDouble(), 0.0, 20.0);
}
BLYNK_WRITE(V23) {
  simTemperature = constrain(param.asDouble(), -10.0, 80.0);
}
BLYNK_WRITE(V24) {
  simHumidity = constrain(param.asDouble(), 0.0, 100.0);
}

BLYNK_CONNECTED() {
  // Synchronize sensor-test slider settings, never actuator commands.
  Blynk.syncVirtual(V20, V21, V22, V23, V24);

  stopAll();
  Blynk.virtualWrite(V0, 0);
  Blynk.virtualWrite(V1, 0);
  Blynk.virtualWrite(V3, 0);
}

void publishDashboard() {
  updateEmergencyStop();
  updateArea();

  if (!Blynk.connected()) {
    stopAll();
    digitalWrite(DRIVER_LED, LOW);
    return;
  }

  digitalWrite(DRIVER_LED, estopActive ? LOW : HIGH);

  Blynk.virtualWrite(V10, simMoisture);
  Blynk.virtualWrite(V11, simPH);
  Blynk.virtualWrite(V12, simEC);
  Blynk.virtualWrite(V13, simTemperature);
  Blynk.virtualWrite(V14, areaM2);
  Blynk.virtualWrite(V15, capsulesRemaining);
  Blynk.virtualWrite(V16, capsulesPlanted);
  Blynk.virtualWrite(V17, getStatus());
  Blynk.virtualWrite(V18, estopActive ? 1 : 0);
  Blynk.virtualWrite(V19, simHumidity);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_REV, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_REV, OUTPUT);
  pinMode(DRILL_LED, OUTPUT);
  pinMode(DRIVER_LED, OUTPUT);

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  allIndicatorsOff();
  digitalWrite(DRILL_LED, LOW);
  digitalWrite(DRIVER_LED, LOW);

  capsuleGate.setPeriodHertz(50);
  capsuleGate.attach(SERVO_PIN, 500, 2400);
  capsuleGate.write(0);

  prefs.begin("greenpod", false);
  areaM2 = prefs.getFloat("area", 0.0);
  capsulesPlanted = prefs.getUInt("planted", 0);
  capsulesRemaining = prefs.getUInt("remaining", 100);

  updateEmergencyStop();
  lastAreaUpdate = millis();

  // Connect to the Blynk device using Wokwi Wi-Fi.
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(1000L, publishDashboard);

  Serial.println("Green Pod virtual controller started.");
}

void loop() {
  updateEmergencyStop();

  if (!Blynk.connected()) {
    stopAll();
  }

  Blynk.run();
  timer.run();
}
