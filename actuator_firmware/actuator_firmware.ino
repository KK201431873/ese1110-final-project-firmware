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


// === Intake ===
Servo intakeServo;
int intakeServoTarget = 90;  // default position (degrees)

// === Drive motors ===
#define LEFT_DRIVE_LPWM_PIN 5
#define LEFT_DRIVE_RPWM_PIN 6
#define RIGHT_DRIVE_LPWM_PIN 9
#define RIGHT_DRIVE_RPWM_PIN 10
int leftDrivePower = 0;
int rightDrivePower = 0;

// === Motor run-on prevention ===
const unsigned long MOTOR_COMMAND_TIMEOUT = 1000; // 1sec without a drive command stops the motor
unsigned long lastLeftDriveCommand = 0;
unsigned long lastRightDriveCommand = 0;
unsigned long lastIntakeMotorCommand = 0;


// === Setup ===
void setup() {
  Serial.begin(115200);

  intakeServo.attach(INTAKE_SERVO_PIN);
  intakeServo.write(intakeServoTarget);

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
  intakeServo.write(intakeServoTarget);
}


// =====================================================
// === Motor update ============================
// =====================================================
void updateDriveMotors() {
  unsigned long nowMs = millis();

  // Left
  if (nowMs - lastLeftDriveCommand < MOTOR_COMMAND_TIMEOUT) {
    setLeftDrivePower(leftDrivePower);
  } else {
    setLeftDrivePower(0);
  }

  // Right
  if (nowMs - lastRightDriveCommand < MOTOR_COMMAND_TIMEOUT) {
    setRightDrivePower(rightDrivePower);
  } else {
    setRightDrivePower(0);
  }
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

void setRightDrivePower(int rightPower) {
  rightPower = max(-255, min(255, rightPower));
  if (rightPower >= 0) {
    analogWrite(RIGHT_DRIVE_RPWM_PIN, abs(rightPower));
    analogWrite(RIGHT_DRIVE_LPWM_PIN, 0);
  } else {
    analogWrite(RIGHT_DRIVE_RPWM_PIN, 0);
    analogWrite(RIGHT_DRIVE_LPWM_PIN, abs(rightPower));
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
          int val = (int)(valStr.toDouble() * 180.0);
          intakeServoTarget = constrain(val, 0, 180);
        } 
        else if (key == "command.intake.motor") {
          // handle intake speed control
          lastIntakeMotorCommand = millis();
        } 
        else if (key == "command.drive.left") {
          int val = (int)(valStr.toDouble() * 255.0);
          leftDrivePower = constrain(val, -255, 255);
          lastLeftDriveCommand = millis();
        } 
        else if (key == "command.drive.right") {
          int val = (int)(valStr.toDouble() * 255.0);
          rightDrivePower = constrain(val, -255, 255);
          lastRightDriveCommand = millis();
        }
      }
      buffer = "";
    } 
    else {
      buffer += c;
    }
  }
}

