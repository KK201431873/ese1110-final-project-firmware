#include <Wire.h>
#include <Servo.h>

#define MPU_ADDR 0x68
#define INTAKE_SERVO_PIN 3

// === Timing configuration ===
const int IMU_HZ = 100;
const int IMU_PRINT_HZ = 20;
const int ENCODER_HZ = 50;
const int ENCODER_PRINT_HZ = 20;
const int SERVO_HZ = 20;
const unsigned long IMU_INTERVAL_US = 1000000UL / IMU_HZ;
const unsigned long IMU_PRINT_INTERVAL_US = 1000000UL / IMU_PRINT_HZ;
const unsigned long ENCODER_INTERVAL_US = 1000000UL / ENCODER_HZ;
const unsigned long ENCODER_PRINT_INTERVAL_US = 1000000UL / ENCODER_PRINT_HZ;
const unsigned long SERVO_INTERVAL_US = 1000000UL / SERVO_HZ;

// === IMU offsets ===
const float gx_offset = 1.37;
const float gy_offset = 9.87;
const float gz_offset = -0.85;

// === Orientation state ===
float roll = 0, pitch = 0, yaw = 0;

// === Servo ===
Servo intakeServo;
int servoTargetDeg = 90;  // default position (degrees)

// === Motor encoder states ===
long leftEncoder = 0;
long rightEncoder = 0;

// === Timing trackers ===
unsigned long lastIMU = 0;
unsigned long lastIMUPrint = 0;
unsigned long lastEncoder = 0;
unsigned long lastEncoderPrint = 0;
unsigned long lastServo = 0;

// === Frequency monitoring ===
const bool printFrequencies = false; 
unsigned long lastFreqPrint = 0;
unsigned long imuCount = 0;
unsigned long imuPrintCount = 0;
unsigned long encoderCount = 0;
unsigned long encoderPrintCount = 0;
unsigned long servoCount = 0;


// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400 kHz Fast Mode I2C

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  intakeServo.attach(INTAKE_SERVO_PIN);
  intakeServo.write(servoTargetDeg);
}

// === Main loop ===
void loop() {
  unsigned long now = micros();

  // Each of these functions has "catch-up" delay logic which maintains target frequencies

  // IMU @ 100 Hz 
  if (now - lastIMU >= IMU_INTERVAL_US) {
    imuCount++;
    processIMU();
    lastIMU += IMU_INTERVAL_US;
    if ((long)(now - lastIMU) >= IMU_INTERVAL_US)
      lastIMU = now;
  }

  // IMU Print @ 20 Hz 
  if (now - lastIMUPrint >= IMU_PRINT_INTERVAL_US) {
    imuPrintCount++;
    printIMU();
    lastIMUPrint += IMU_PRINT_INTERVAL_US;
    if ((long)(now - lastIMUPrint) >= IMU_PRINT_INTERVAL_US)
      lastIMUPrint = now;
  }

  // Encoder @ 50 Hz 
  if (now - lastEncoder >= ENCODER_INTERVAL_US) {
    encoderCount++;
    processEncoder();
    lastEncoder += ENCODER_INTERVAL_US;
    if ((long)(now - lastEncoder) >= ENCODER_INTERVAL_US)
      lastEncoder = now;
  }

  // Encoder Print @ 20 Hz 
  if (now - lastEncoderPrint >= ENCODER_PRINT_INTERVAL_US) {
    encoderPrintCount++;
    printEncoder();
    lastEncoderPrint += ENCODER_PRINT_INTERVAL_US;
    if ((long)(now - lastEncoderPrint) >= ENCODER_PRINT_INTERVAL_US)
      lastEncoderPrint = now;
  }

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
      Serial.print("[FREQ] IMU: ");          Serial.print(imuCount);
      Serial.print(" Hz, IMU Print: ");      Serial.print(imuPrintCount);
      Serial.print(" Hz, Encoder: ");        Serial.print(encoderCount);
      Serial.print(" Hz, Encoder Print: ");  Serial.print(encoderPrintCount);
      Serial.print(" Hz, Servo: ");          Serial.print(servoCount);
      Serial.println(" Hz");

      imuCount = imuPrintCount = encoderCount = encoderPrintCount = servoCount = 0;
      lastFreqPrint = nowMs;
    }
  }
}


// =========================================================
// === IMU Processing with Complementary Filter (100 Hz) ===
// =========================================================
void processIMU() {
  // --- Read 14 bytes starting at ACCEL_XOUT_H ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t a_x = Wire.read() << 8 | Wire.read();
  int16_t a_y = Wire.read() << 8 | Wire.read();
  int16_t a_z = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  int16_t g_x = Wire.read() << 8 | Wire.read();
  int16_t g_y = Wire.read() << 8 | Wire.read();
  int16_t g_z = Wire.read() << 8 | Wire.read();

  // --- Convert to physical units ---
  const float gyro_sensitivity = 131.0;
  const float acc_sensitivity = 16384.0; // for ±2g

  float gx = (g_x / gyro_sensitivity - gx_offset) * DEG_TO_RAD;
  float gy = (g_y / gyro_sensitivity - gy_offset) * DEG_TO_RAD;
  float gz = (g_z / gyro_sensitivity - gz_offset) * DEG_TO_RAD;

  float ax = a_x / acc_sensitivity;
  float ay = a_y / acc_sensitivity;
  float az = a_z / acc_sensitivity;

  static unsigned long last_us = micros();
  unsigned long now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;

  // 1. Integrate gyro
  roll  += gx * dt * RAD_TO_DEG;
  pitch += gy * dt * RAD_TO_DEG;
  yaw   += gz * dt * RAD_TO_DEG;

  // 2. Compute accel-based roll/pitch
  float acc_roll  = atan2(ay, az) * RAD_TO_DEG;
  float acc_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

  // 3. Complementary filter
  const float alpha = 0.95f;
  roll  = alpha * roll  + (1 - alpha) * acc_roll;
  pitch = alpha * pitch + (1 - alpha) * acc_pitch;

  // 4. Normalize yaw
  if (yaw > 180.0f) yaw -= 360.0f;
  else if (yaw < -180.0f) yaw += 360.0f;
}

void printIMU() {
  Serial.print("sensor.imu.roll:");
  Serial.println(roll);
  Serial.print("sensor.imu.pitch:");
  Serial.println(pitch);
  Serial.print("sensor.imu.yaw:");
  Serial.println(yaw);
}

// =====================================================
// === Servo update (20 Hz) ============================
// =====================================================
void updateServo() {
  intakeServo.write(servoTargetDeg);
}

// =====================================================
// === Motor encoder processing (100 Hz) ================
// =====================================================
void processEncoder() {
  double now = millis()/1.0e3;
  leftEncoder += (int)(100 * (0.3*sin(0.103*now + 0.314*cos(0.063*now)) + 0.7*cos(0.1*tan(0.01*now) + 0.084*now + 0.143*cos(0.01*now))));
  rightEncoder += (int)(100 * (0.45*cos(0.03*now - 0.287*sin(0.021*now)) + 0.55*sin(0.153*now - 0.09*sin(0.061*now))));
}

void printEncoder() {
  Serial.print("sensor.encoder.left:");
  Serial.println(leftEncoder);
  Serial.print("sensor.encoder.right:");
  Serial.println(rightEncoder);
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
        int val = valStr.toInt();

        if (key == "command.intake.position") {
          servoTargetDeg = constrain(val, 0, 180);
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

