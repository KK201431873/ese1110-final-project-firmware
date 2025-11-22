#include <Servo.h>


// === Timing configuration ===
const int SERVO_LOOP_HZ = 100;
const int MOTORS_LOOP_HZ = 100;
const unsigned long SERVO_LOOP_INTERVAL_US = 1000000UL / SERVO_LOOP_HZ;
const unsigned long MOTORS_LOOP_INTERVAL_US = 1000000UL / MOTORS_LOOP_HZ;

// === Timing trackers ===
unsigned long lastServoLoop = 0;
unsigned long lastMotorsLoop = 0;

// === Frequency monitoring ===
const bool printFrequencies = false; 
unsigned long servoLoopCount = 0;
unsigned long motorsLoopCount = 0;


// === Intake ===
#define INTAKE_SERVO_PIN A0
Servo intakeServo;
int intakeServoTarget = 90;  // default position (degrees)

// === Drive motors ===
#define LEFT_DRIVE_LPWM_PIN 3
#define LEFT_DRIVE_RPWM_PIN 5
#define RIGHT_DRIVE_LPWM_PIN 6
#define RIGHT_DRIVE_RPWM_PIN 9
#define INTAKE_LPWM_PIN 10
#define INTAKE_RPWM_PIN 11
const bool INVERT_LEFT_DRIVE = false;
const bool INVERT_RIGHT_DRIVE = true;
const bool INVERT_INTAKE = false;
const int MAXIMUM_INTAKE_PWM = 60;
int leftDrivePower = 0;
int rightDrivePower = 0;
int intakePower = 0;

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

  // Right drive motor
  pinMode(RIGHT_DRIVE_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_DRIVE_RPWM_PIN, OUTPUT);
  setRightDrivePower(0);

  // Intake motor
  pinMode(INTAKE_LPWM_PIN, OUTPUT);
  pinMode(INTAKE_RPWM_PIN, OUTPUT);
  setIntakePower(0);
}

// === Main loop ===
void loop() {
  unsigned long now = micros();

  // Servo control
  if (now - lastServoLoop >= SERVO_LOOP_INTERVAL_US) {
    servoLoopCount++;
    updateServo();
    lastServoLoop += SERVO_LOOP_INTERVAL_US;
    if ((long)(now - lastServoLoop) >= SERVO_LOOP_INTERVAL_US)
      lastServoLoop = now;
  }

  // Motor control
  if (now - lastMotorsLoop >= MOTORS_LOOP_INTERVAL_US) {
    motorsLoopCount++;
    updateMotors();
    lastMotorsLoop += MOTORS_LOOP_INTERVAL_US;
    if ((long)(now - lastMotorsLoop) >= MOTORS_LOOP_INTERVAL_US)
      lastMotorsLoop = now;
  }

  // Serial input (asynchronous) 
  processSerial();

  // Frequency monitor (1 Hz print) 
  if (printFrequencies) {
  static unsigned long lastFreqPrint = 0;
  unsigned long nowMs = millis();
    if (nowMs - lastFreqPrint >= 1000) {
      Serial.print("[FREQ] Servo: ");          Serial.print(servoLoopCount);
      Serial.println(" Hz");

      servoLoopCount = 0;
      lastFreqPrint = nowMs;
    }
  }
}


// =====================================================
// === Servo update ============================
// =====================================================
void updateServo() {
  intakeServo.write(intakeServoTarget);
}


// =====================================================
// === Motor update ============================
// =====================================================
void updateMotors() {
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

  // Intake
  if (nowMs - lastIntakeMotorCommand < MOTOR_COMMAND_TIMEOUT) {
    setIntakePower(intakePower);
  } else {
    setIntakePower(0);
  }
}

void setLeftDrivePower(int leftPower) {
  leftPower = max(-255, min(255, leftPower));
  if (INVERT_LEFT_DRIVE) leftPower *= -1;
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
  if (INVERT_RIGHT_DRIVE) rightPower *= -1;
  if (rightPower >= 0) {
    analogWrite(RIGHT_DRIVE_RPWM_PIN, abs(rightPower));
    analogWrite(RIGHT_DRIVE_LPWM_PIN, 0);
  } else {
    analogWrite(RIGHT_DRIVE_RPWM_PIN, 0);
    analogWrite(RIGHT_DRIVE_LPWM_PIN, abs(rightPower));
  }
}

void setIntakePower(int intakePower) {
  intakePower = max(-MAXIMUM_INTAKE_PWM, min(MAXIMUM_INTAKE_PWM, intakePower));
  if (INVERT_INTAKE) intakePower *= -1;
  if (intakePower >= 0) {
    analogWrite(INTAKE_RPWM_PIN, abs(intakePower));
    analogWrite(INTAKE_LPWM_PIN, 0);
  } else {
    analogWrite(INTAKE_RPWM_PIN, 0);
    analogWrite(INTAKE_LPWM_PIN, abs(intakePower));
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
          int val = (int)(valStr.toDouble() * MAXIMUM_INTAKE_PWM);
          intakePower = constrain(val, -MAXIMUM_INTAKE_PWM, MAXIMUM_INTAKE_PWM);
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

