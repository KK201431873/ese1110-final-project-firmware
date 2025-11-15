#include <Servo.h>

#define INTAKE_SERVO_PIN 3


// === Timing configuration ===
const int INTAKE_SERVO_HZ = 100;
const int DRIVE_MOTORS_HZ = 100;
const unsigned long INTAKE_SERVO_INTERVAL_US = 1000000UL / INTAKE_SERVO_HZ;
const unsigned long DRIVE_MOTORS_INTERVAL_US = 1000000UL / DRIVE_MOTORS_HZ;

// === Timing trackers ===
unsigned long lastIntakeServo = 0;
unsigned long lastDriveMotors = 0;

// === Frequency monitoring ===
const bool printFrequencies = false; 
unsigned long intakeServoCount = 0;
unsigned long driveMotorsCount = 0;


// === Servo ===
Servo intakeServo;
int intakeServoTargetDeg = 90;  // default position (degrees)

// === Drive motors ===
#define LEFT_DRIVE_LPWM_PIN 9
#define LEFT_DRIVE_RPWM_PIN 10


// === Setup ===
void setup() {
  Serial.begin(115200);

  intakeServo.attach(INTAKE_SERVO_PIN);
  intakeServo.write(intakeServoTargetDeg);

  // Left drive motor
  pinMode(LEFT_DRIVE_LPWM_PIN, OUTPUT);
  pinMode(LEFT_DRIVE_RPWM_PIN, OUTPUT);
  setLeftDrivePower(0);
}

// === Main loop ===
void loop() {
  unsigned long now = micros();

  // Intake servo control
  if (now - lastIntakeServo >= INTAKE_SERVO_INTERVAL_US) {
    intakeServoCount++;
    updateIntakeServo();
    lastIntakeServo += INTAKE_SERVO_INTERVAL_US;
    if ((long)(now - lastIntakeServo) >= INTAKE_SERVO_INTERVAL_US)
      lastIntakeServo = now;
  }

  // Drive motor control
  if (now - lastDriveMotors >= DRIVE_MOTORS_INTERVAL_US) {
    driveMotorsCount++;
    updateDriveMotors();
    lastDriveMotors += DRIVE_MOTORS_INTERVAL_US;
    if ((long)(now - lastDriveMotors) >= DRIVE_MOTORS_INTERVAL_US)
      lastDriveMotors = now;
  }

  // Serial input (asynchronous) 
  processSerial();

  // Frequency monitor (1 Hz print) 
  if (printFrequencies) {
  static unsigned long lastFreqPrint = 0;
  unsigned long nowMs = millis();
    if (nowMs - lastFreqPrint >= 1000) {
      Serial.print("[FREQ] Servo: ");          Serial.print(intakeServoCount);
      Serial.println(" Hz");

      intakeServoCount = 0;
      lastFreqPrint = nowMs;
    }
  }
}


// =====================================================
// === Servo update ============================
// =====================================================
void updateIntakeServo() {
  intakeServo.write(intakeServoTargetDeg);
}


// =====================================================
// === Motor update ============================
// =====================================================
void updateDriveMotors() {
  double now = millis()/1.0e3;
  int power = (int) (255*sin(now));
  setLeftDrivePower(power);
}

void setLeftDrivePower(int leftPower) {
  leftPower = max(-255, min(255, leftPower));
  if (leftPower >= 0) {
    analogWrite(LEFT_DRIVE_RPWM_PIN, abs(leftPower));
    analogWrite(LEFT_DRIVE_LPWM_PIN, 0);
  } else {
    analogWrite(LEFT_DRIVE_RPWM_PIN, 0);
    analogWrite(LEFT_DRIVE_LPWM_PIN, abs(leftPower));
  }
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

