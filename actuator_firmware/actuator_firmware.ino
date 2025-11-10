#include <Servo.h>

#define INTAKE_SERVO_PIN 3

// === Timing configuration ===
const int SERVO_HZ = 100;
const unsigned long SERVO_INTERVAL_US = 1000000UL / SERVO_HZ;

// === Servo ===
Servo intakeServo;
int intakeServoTargetDeg = 90;  // default position (degrees)

// === Timing trackers ===
unsigned long lastServo = 0;

// === Frequency monitoring ===
const bool printFrequencies = false; 
unsigned long servoCount = 0;


// === Setup ===
void setup() {
  Serial.begin(115200);

  intakeServo.attach(INTAKE_SERVO_PIN);
  intakeServo.write(intakeServoTargetDeg);
}

// === Main loop ===
void loop() {
  unsigned long now = micros();

  // Each of these functions has "catch-up" delay logic which maintains target frequencies


  // Servo Control @ 20 Hz 
  if (now - lastServo >= SERVO_INTERVAL_US) {
    servoCount++;
    updateServo();
    lastServo += SERVO_INTERVAL_US;
    if ((long)(now - lastServo) >= SERVO_INTERVAL_US)
      lastServo = now;
  }

  // Serial input (asynchronous) 
  processSerial();

  // Frequency monitor (1 Hz print) 
  if (printFrequencies) {
  static unsigned long lastFreqPrint = 0;
  unsigned long nowMs = millis();
    if (nowMs - lastFreqPrint >= 1000) {
      Serial.print("[FREQ] Servo: ");          Serial.print(servoCount);
      Serial.println(" Hz");

      servoCount = 0;
      lastFreqPrint = nowMs;
    }
  }
}


// =====================================================
// === Servo update (20 Hz) ============================
// =====================================================
void updateServo() {
  intakeServo.write(intakeServoTargetDeg);
}

// =====================================================
// === Serial message parsing ==========================
// =====================================================
void processSerial() {
  static String buffer = "";

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() == 0) continue;

      int sepIndex = buffer.indexOf(':');
      if (sepIndex > 0) {
        String key = buffer.substring(0, sepIndex);
        String valStr = buffer.substring(sepIndex + 1);

        if (key == "command.intake.position") {
          int val = valStr.toInt();
          intakeServoTargetDeg = constrain(val, 0, 180);
        } 
        else if (key == "command.intake.motor") {
          // handle intake speed control
        } 
        else if (key == "command.drive.left") {
          // handle left motor speed
        } 
        else if (key == "command.drive.right") {
          // handle right motor speed
        }
      }
      buffer = "";
    } 
    else {
      buffer += c;
    }
  }
}

