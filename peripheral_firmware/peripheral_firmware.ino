#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// =====================================================
// === Robot configuration ===
// =====================================================
const float WHEEL_DIAMETER = 0.065;   // m
const float WHEEL_BASE = 0.332;       // m
const float TICKS_PER_REV = 1024.0;
const float GEAR_RATIO = 1.0;
const float ALPHA = 0.05;             // IMU yaw fusion weight

// =====================================================
// === Timing configuration ===
// =====================================================
const int IMU_HZ = 100;
const int LOCALIZATION_HZ = 1000;     // Encoder localization at 1 kHz
const int PRINT_HZ = 100;             // Unified print frequency

const unsigned long IMU_INTERVAL_US = 1000000UL / IMU_HZ;
const unsigned long LOC_INTERVAL_US = 1000000UL / LOCALIZATION_HZ;
const unsigned long PRINT_INTERVAL_US = 1000000UL / PRINT_HZ;

// === IMU object ===
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// =====================================================
// === State ===
// =====================================================
float roll = 0, pitch = 0, yaw = 0;   // IMU orientation (deg)
long leftEncoder = 0, rightEncoder = 0;
long lastLeft = 0, lastRight = 0;

// Robot pose (meters, radians)
float poseX = 0.0, poseY = 0.0, poseH = 0.0;

// Timing trackers
unsigned long lastIMU = 0;
unsigned long lastLoc = 0;
unsigned long lastPrint = 0;

// === Frequency monitoring ===
const bool printFrequencies = false; 
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
  Wire.setClock(400000);

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected. Check wiring!");
    while (1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("BNO055 initialized successfully!");
}


// =====================================================
// === Main loop ===
// =====================================================
void loop() {
  unsigned long now = micros();

  // --- IMU update (100 Hz) ---
  if (now - lastIMU >= IMU_INTERVAL_US) {
    imuCount++;
    processIMU();
    fuseIMUYaw();
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
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  roll  = orientationData.orientation.x;
  pitch = orientationData.orientation.y;
  yaw   = orientationData.orientation.z;
}


// =====================================================
// === Encoder-based localization (1 kHz) ===
// =====================================================
Encoder leftEnc(2, 3);
Encoder rightEnc(4, 5);

void processEncoderLocalization() {
  long newLeft = leftEnc.read();
  long newRight = rightEnc.read();

  long deltaLeft = newLeft - lastLeft;
  long deltaRight = newRight - lastRight;

  lastLeft = newLeft;
  lastRight = newRight;

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
vvoid printAll() {
  Serial.print(F("sensor.encoder.left:")); Serial.print(leftEnc.read());
  Serial.print(F(",sensor.encoder.right:")); Serial.print(rightEnc.read());
  Serial.print(F(",sensor.imu.roll:")); Serial.print(roll);
  Serial.print(F(",sensor.imu.pitch:")); Serial.print(pitch);
  Serial.print(F(",sensor.imu.yaw:")); Serial.print(yaw);
  Serial.print(F(",localization.x:")); Serial.print(poseX, 6);
  Serial.print(F(",localization.y:")); Serial.print(poseY, 6);
  Serial.print(F(",localization.h:")); Serial.print(poseH * RAD_TO_DEG, 3);
  Serial.println();
}