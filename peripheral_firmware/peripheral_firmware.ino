#include <Wire.h>
// #include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <EEPROM.h>

// =====================================================
// === Robot configuration ===
// =====================================================
const float WHEEL_DIAMETER = 0.065;   // m
const float WHEEL_BASE = 0.332;       // m
const float TICKS_PER_REV = 1024.0;
const float GEAR_RATIO = 1.0;
const float ALPHA = 1.00;             // IMU yaw fusion weight

// =====================================================
// === Timing configuration ===
// =====================================================
const int IMU_HZ = 150;
const int LOCALIZATION_HZ = 1000;     // Encoder localization at 1 kHz
const int PRINT_HZ = 30;             // Unified print frequency

const unsigned long IMU_INTERVAL_US = 1000000UL / IMU_HZ;
const unsigned long LOC_INTERVAL_US = 1000000UL / LOCALIZATION_HZ;
const unsigned long PRINT_INTERVAL_US = 1000000UL / PRINT_HZ;

// === IMU object ===
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float imuYawOffset = 0.0f;

// IMU watchdog
bool yawIsZero = false;
float lastNonzeroYaw = 0.0f;
float last_asum = 0.0;
int stillCount = 0;
const int BNO_TIMEOUT_FRAMES = 50;
const int BNO_RST_PIN = 2;


// =====================================================
// === State ===
// =====================================================
float roll = 0, pitch = 0, yaw = 0;   // IMU orientation (deg)
long leftEncoder = 0, rightEncoder = 0;

// Robot pose (meters, radians)
float poseX = 0.0, poseY = 0.0, poseH = 0.0;

// Timing trackers
unsigned long lastIMU = 0;
unsigned long lastLoc = 0;
unsigned long lastPrint = 0;

// === Frequency monitoring ===
const bool printFrequencies = true; 
unsigned long lastFreqPrint = 0;
unsigned long imuCount = 0;
unsigned long localizationCount = 0;
unsigned long printCount = 0;


// =====================================================
// === Setup ===
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  while (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected. Check wiring!");
    resetIMU();
  }
  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);

  // --- Load in calibration data ---
  int eeAddress = 0;
  long bnoID;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning of EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);
      bno.setSensorOffsets(calibrationData);
  }

  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized successfully!");
}


// =====================================================
// === Main loop ===
// =====================================================
void loop() {
  unsigned long now = micros();

  // --- IMU update (75 Hz) ---
  if (now - lastIMU >= IMU_INTERVAL_US) {
    imuCount++;
    processIMU();
    if (!yawIsZero) {
      fuseIMUYaw();
    }
    lastIMU += IMU_INTERVAL_US;
    if ((long)(now - lastIMU) >= IMU_INTERVAL_US)
      lastIMU = now;
  }

  // --- Encoder localization (1 kHz) ---
  if (now - lastLoc >= LOC_INTERVAL_US) {
    localizationCount++;
    processEncoderLocalization();
    lastLoc += LOC_INTERVAL_US;
    if ((long)(now - lastLoc) >= LOC_INTERVAL_US)
      lastLoc = now;
  }

  // --- Unified print (100 Hz) ---
  if (now - lastPrint >= PRINT_INTERVAL_US) {
    printCount++;
    printAll();
    lastPrint += PRINT_INTERVAL_US;
    if ((long)(now - lastPrint) >= PRINT_INTERVAL_US)
      lastPrint = now;
  }

  // Frequency monitor (1 Hz print) 
  if (printFrequencies) {
  static unsigned long lastFreqPrint = 0;
  unsigned long nowMs = millis();
    if (nowMs - lastFreqPrint >= 1000) {
      Serial.print("[FREQ] IMU: ");          Serial.print(imuCount);
      Serial.print(" Hz, Localization: ");   Serial.print(localizationCount);
      Serial.print(" Hz, Print: ");          Serial.print(printCount);
      Serial.println(" Hz");

      imuCount = localizationCount = printCount = 0;
      lastFreqPrint = nowMs;
    }
  }
}


// =====================================================
// === IMU Processing (BNO055 fused orientation) ===
// =====================================================
void processIMU() {
  // --- Watchdog logic ---
  sensors_event_t accel;
  bno.getEvent(&accel, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  float asum = fabs(accel.acceleration.x) + fabs(accel.acceleration.y) + fabs(accel.acceleration.z);
  
  float diff = fabs(asum - last_asum);
  if (diff < 1e-6) {
      stillCount++;
  } else {
      stillCount = 0;
  }

  last_asum = asum;
  if (stillCount > BNO_TIMEOUT_FRAMES) {
      Serial.println("IMU idle → resetting");
      resetIMU();
      stillCount = 0;
  }

  // --- Quaternion conversion ---
  imu::Quaternion quat = bno.getQuat();

  float w = quat.w();
  float x = quat.x();
  float y = quat.y();
  float z = quat.z();

  // Convert quaternion → Euler angles
  // BNO055 uses aerospace sequence (heading = yaw)
  float ysqr = y * y;

  // roll (x-axis rotation)
  float t0 = +2.0f * (w * x + y * z);
  float t1 = +1.0f - 2.0f * (x * x + ysqr);
  roll = atan2(t0, t1) * 180.0f / PI;

  // pitch (y-axis rotation)
  float t2 = +2.0f * (w * y - z * x);
  t2 = t2 >  1.0f ?  1.0f : t2;
  t2 = t2 < -1.0f ? -1.0f : t2;
  pitch = asin(t2) * 180.0f / PI;

  // yaw (z-axis rotation)
  float t3 = +2.0f * (w * z + x * y);
  float t4 = +1.0f - 2.0f * (ysqr + z * z);
  yaw = atan2(t3, t4) * 180.0f / PI;

  // --- Compute raw yaw BEFORE offset ---
  float rawYaw = atan2(t3, t4) * 180.0f / PI;

  // Determine if raw IMU yaw is essentially zero
  if (fabs(rawYaw) > 0.01f) {
      yawIsZero = false;
      lastNonzeroYaw = rawYaw;

      // Now apply offset
      yaw = rawYaw + (imuYawOffset * RAD_TO_DEG);
  } else {
      yawIsZero = true;

      // Keep yaw equal to poseH (valid number)
      yaw = poseH * RAD_TO_DEG;
  }

  // Wrap final yaw
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

bool resetIMU() {
  Serial.println("### IMU RESET TRIGGERED ###");

  // If hardware reset pin is available
  if (BNO_RST_PIN >= 0) {
    pinMode(BNO_RST_PIN, OUTPUT);
    digitalWrite(BNO_RST_PIN, LOW);
    delay(10);
    digitalWrite(BNO_RST_PIN, HIGH);
    delay(50);
  }

  // Send soft reset over I2C
  Wire.beginTransmission(0x28);
  Wire.write(0x3F);   // SYS_TRIGGER register
  Wire.write(0x20);   // Reset command
  Wire.endTransmission();

  delay(700);  // datasheet: ~650ms boot time

  if (!bno.begin()) {
    Serial.println("IMU begin() failed after reset");
    return false;
  }

  bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
  bno.setExtCrystalUse(true);

  // --- Restore IMU yaw ---
  imu::Quaternion q = bno.getQuat();

  float w = q.w(), x = q.x(), y = q.y(), z = q.z();
  float ysqr = y * y;

  // compute yaw
  float t3 = +2.0f * (w * z + x * y);
  float t4 = +1.0f - 2.0f * (ysqr + z * z);
  float imuYawAfterReset = atan2(t3, t4);

  // Compute offset to align IMU to stored heading poseH
  imuYawOffset = poseH - imuYawAfterReset;

  Serial.println("IMU reinitialized successfully");
  return true;
}


// =====================================================
// === Encoder-based localization (1 kHz) ===
// =====================================================
// Encoder leftEnc(2, 3);
// Encoder rightEnc(4, 5);

void processEncoderLocalization() {
  // long newLeft = leftEnc.read();
  // long newRight = rightEnc.read();

  // long deltaLeft = newLeft - leftEncoder;
  // long deltaRight = newRight - rightEncoder;

  // leftEncoder = newLeft;
  // rightEncoder = newRight;
  long deltaLeft = 10;
  long deltaRight = 10;

  float distLeft = (deltaLeft / TICKS_PER_REV) * (M_PI * WHEEL_DIAMETER) / GEAR_RATIO;
  float distRight = (deltaRight / TICKS_PER_REV) * (M_PI * WHEEL_DIAMETER) / GEAR_RATIO;

  float deltaDist = (distLeft + distRight) / 2.0;
  float deltaHeading = (distRight - distLeft) / WHEEL_BASE;

  poseX += deltaDist * cosf(poseH + deltaHeading / 2.0);
  poseY += deltaDist * sinf(poseH + deltaHeading / 2.0);
  poseH += deltaHeading;

  // Normalize heading to [-pi, pi]
  if (poseH > M_PI) poseH -= 2 * M_PI;
  else if (poseH < -M_PI) poseH += 2 * M_PI;
}


// =====================================================
// === Fuse IMU yaw with odometry heading (100 Hz) ===
// =====================================================
void fuseIMUYaw() {
  float imuYawRad = yaw * DEG_TO_RAD;

  float x_h = cosf(poseH), y_h = sinf(poseH);
  float x_imu = cosf(imuYawRad), y_imu = sinf(imuYawRad);

  float x_bar = ALPHA * x_imu + (1.0 - ALPHA) * x_h;
  float y_bar = ALPHA * y_imu + (1.0 - ALPHA) * y_h;

  poseH = atan2f(y_bar, x_bar);
}


// =====================================================
// === Unified print (100 Hz) ===
// =====================================================
void printAll() {
  Serial.print(F("encoder.left:")); Serial.print(leftEncoder);
  Serial.print(F(",encoder.right:")); Serial.print(rightEncoder);
  Serial.print(F(",imu.roll:")); Serial.print(roll);
  Serial.print(F(",imu.pitch:")); Serial.print(pitch);
  Serial.print(F(",imu.yaw:")); Serial.print(yaw);
  Serial.print(F(",localization.pose:")); Serial.print(poseX, 6); Serial.print(F(";")); Serial.print(poseY, 6); Serial.print(F(";")); Serial.print(poseH, 3);
  Serial.println();
}