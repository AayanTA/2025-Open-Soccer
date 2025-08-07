#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "PowerfulBLDCdriver.h"
#include <math.h>

// ---------- Field Dimensions (in mm) -----------
#define FIELD_LEFT_MM    0
#define FIELD_RIGHT_MM   5000
#define FIELD_BOTTOM_MM  0
#define FIELD_TOP_MM     5000

// ---------- IMU Setup -----------
#define BNO08X_RESET -1
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool initialYawSet = false;
float initialYaw = 0.0;

// ---------- Heading Correction Parameters -----------
const float headingKp = 1.0; // Tune as needed
float getHeadingCorrection(float relativeYaw) {
  float corr = headingKp * (relativeYaw / 180.0);
  if (corr > 1.0) corr = 1.0;
  if (corr < -1.0) corr = -1.0;
  return corr;
}

// ---------- Utility IMU Functions -----------
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector report");
  }
}

void quaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw) {
  float sinr_cosp = 2 * (w * x + y * z);
  float cosr_cosp = 1 - 2 * (x * x + y * y);
  roll = atan2(sinr_cosp, cosr_cosp);
  float sinp = 2 * (w * y - z * x);
  if (fabs(sinp) >= 1)
    pitch = copysign(PI / 2, sinp);
  else
    pitch = asin(sinp);
  float siny_cosp = 2 * (w * z + x * y);
  float cosy_cosp = 1 - 2 * (y * y + z * z);
  yaw = atan2(siny_cosp, cosy_cosp);
  roll  = roll  * 180.0 / PI;
  pitch = pitch * 180.0 / PI;
  yaw   = yaw   * 180.0 / PI;
}

float normalizeAngle180(float angle) {
  while (angle >= 180.0) angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

// ---------- Motor Setup -----------
/*
  New motor configuration:
    Motor 1: Front Left
    Motor 2: Front Right
    Motor 3: Back Right
    Motor 4: Back Left
*/
PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;

// ---------- TOF Sensor Class (SteelBarToF) -----------
class SteelBarToF {
  private:
    uint8_t address;
    TwoWire* wire;
    static const int numReadings = 5; // Moving average window size
    int readings[numReadings];
    int sum;
    int index;
    bool filled;
  public:
    SteelBarToF(uint8_t _address, TwoWire* _wire)
      : address(_address), wire(_wire), sum(0), index(0), filled(false) {
        for (int i = 0; i < numReadings; i++) {
          readings[i] = 0;
        }
      }
    uint8_t getAddress() const { return address; }
    int readDistance() {
      if (!wire) return -1;
      wire->beginTransmission(address);
      wire->write(0x10);
      if (wire->endTransmission() != 0) return -1;
      if (wire->requestFrom(address, 5) == 5) {
        wire->read(); // ignore sequence byte
        int distance = (wire->read() | (wire->read() << 8) |
                        (wire->read() << 16) | (wire->read() << 24));
        sum -= readings[index];
        readings[index] = distance;
        sum += distance;
        index = (index + 1) % numReadings;
        if (index == 0) filled = true;
        return filled ? sum / numReadings : sum / (index + 1);
      }
      return -1;
    }
};

// ---------- Create Array of 8 TOF Sensors ----------
#define NUM_SENSORS 8
SteelBarToF tofSensors[NUM_SENSORS] = {
  SteelBarToF(0x50, &Wire),
  SteelBarToF(0x51, &Wire),
  SteelBarToF(0x52, &Wire),
  SteelBarToF(0x53, &Wire),
  SteelBarToF(0x54, &Wire),
  SteelBarToF(0x55, &Wire),
  SteelBarToF(0x56, &Wire),
  SteelBarToF(0x57, &Wire)
};

// ---------- Localization Function ----------
// Sensor mounting (per your description): Sensors increase counterclockwise from sensor 0 at the back.
// For example, assume:
//   Sensor 4: Front
//   Sensor 0: Back
//   Sensor 6: Left
//   Sensor 2: Right   <-- IGNORE sensor 2 per request
// We'll use sensor 4 (front) and sensor 0 (back) for Y,
// and only sensor 6 (left) for X. (You might add others if available.)
void computeEstimatedPosition(float &estX, float &estY) {
  // For Y estimate:
  int d_front = tofSensors[4].readDistance(); // front
  int d_back  = tofSensors[0].readDistance();  // back
  float yFromFront = (d_front >= 0) ? FIELD_TOP_MM - d_front : -1;
  float yFromBack  = (d_back  >= 0) ? FIELD_BOTTOM_MM + d_back : -1;
  if (yFromFront >= 0 && yFromBack >= 0)
    estY = (yFromFront + yFromBack) / 2.0;
  else if (yFromFront >= 0)
    estY = yFromFront;
  else if (yFromBack >= 0)
    estY = yFromBack;
  else
    estY = -1;
  
  // For X estimate, ignore sensor 2.
  // Use sensor 6 (left) only:
  int d_left = tofSensors[6].readDistance();
  estX = (d_left >= 0) ? FIELD_LEFT_MM + d_left : -1;
}

// ---------- Global Variables for Starting Position ----------
bool startPositionSet = false;
float startX = 0.0, startY = 0.0;

// ---------- Global Variables for Current Position & Heading ----------
float currentX = 0.0, currentY = 0.0;
float currentHeading = 0.0;  // relative yaw in degrees

// ---------- Movement Function ----------
// For our configuration:
//   Motor 1 (Front Left)  = - (drive + strafe + rotation)
//   Motor 2 (Front Right) =   (drive - strafe - rotation)
//   Motor 3 (Back Right)  =   (drive + strafe - rotation)
//   Motor 4 (Back Left)   = - (drive - strafe + rotation)
void setMotorSpeeds(float translationAngleDeg, float translationSpeedNormalized, float rotation) {
  float translationAngleRad = translationAngleDeg * PI / 180.0;
  float drive  = translationSpeedNormalized * cos(translationAngleRad);
  float strafe = translationSpeedNormalized * sin(translationAngleRad);
  
  float m1 = -(drive + strafe + rotation);  // Front Left
  float m2 =  (drive - strafe - rotation);    // Front Right
  float m3 =  (drive + strafe - rotation);    // Back Right
  float m4 = -(drive - strafe + rotation);    // Back Left
  
  float motorOutputs[4] = { m1, m2, m3, m4 };
  float maxVal = 0;
  for (int i = 0; i < 4; i++) {
    if (fabs(motorOutputs[i]) > maxVal) maxVal = fabs(motorOutputs[i]);
  }
  if (maxVal > 1.0) {
    for (int i = 0; i < 4; i++) {
      motorOutputs[i] /= maxVal;
    }
  }
  
  float maxMotorSpeed = 60000000.0;
  long scaledSpeeds[4];
  for (int i = 0; i < 4; i++) {
    scaledSpeeds[i] = (long)(motorOutputs[i] * maxMotorSpeed);
  }
  
  motor1.setSpeed(scaledSpeeds[0]);
  motor2.setSpeed(scaledSpeeds[1]);
  motor3.setSpeed(scaledSpeeds[2]);
  motor4.setSpeed(scaledSpeeds[3]);
  
  Serial.print("Motor speeds: ");
  Serial.print(scaledSpeeds[0]); Serial.print(" ");
  Serial.print(scaledSpeeds[1]); Serial.print(" ");
  Serial.print(scaledSpeeds[2]); Serial.print(" ");
  Serial.println(scaledSpeeds[3]);
}

// ---------- Spin Function (Optional) ----------
void spinAround(float speedNormalized) {
  float maxMotorSpeed = 45000000.0;
  long scaledSpeed = (long)(speedNormalized * maxMotorSpeed);
  motor1.setSpeed(scaledSpeed);
  motor2.setSpeed(scaledSpeed);
  motor3.setSpeed(-scaledSpeed);
  motor4.setSpeed(-scaledSpeed);
}

// ---------- Setup Function ----------
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Merged Localization & Return-To-Start Code");
  
  // Initialize IMU
  Wire.setSCL(9);
  Wire.setSDA(8);
  Wire.begin();
  Wire.setClock(1000000);
  if (!bno08x.begin_I2C(74, &Wire)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }
  Serial.println("BNO08x Found!");
  setReports();
  delay(100);
  
  // Initialize Motors
  motor1.begin(25, &Wire);
  motor1.setCurrentLimitFOC(65536 * 2);
  motor1.setIdPidConstants(1500, 200);
  motor1.setIqPidConstants(1500, 200);
  motor1.setSpeedPidConstants(4e-2, 4e-4, 3e-2);
  
  motor2.begin(26, &Wire);
  motor2.setCurrentLimitFOC(65536 * 2);
  motor2.setIdPidConstants(1500, 200);
  motor2.setIqPidConstants(1500, 200);
  motor2.setSpeedPidConstants(4e-2, 4e-4, 3e-2);
  
  motor3.begin(27, &Wire);
  motor3.setCurrentLimitFOC(65536 * 2);
  motor3.setIdPidConstants(1500, 200);
  motor3.setIqPidConstants(1500, 200);
  motor3.setSpeedPidConstants(4e-2, 4e-4, 3e-2);
  
  motor4.begin(28, &Wire);
  motor4.setCurrentLimitFOC(65536 * 2);
  motor4.setIdPidConstants(1500, 200);
  motor4.setIqPidConstants(1500, 200);
  motor4.setSpeedPidConstants(4e-2, 4e-4, 3e-2);
  
  motor1.setELECANGLEOFFSET(1443874048);
  motor1.setSINCOSCENTRE(1258);
  motor1.configureOperatingModeAndSensor(3, 1);
  motor1.configureCommandMode(12);
  
  motor2.setELECANGLEOFFSET(1803318016);
  motor2.setSINCOSCENTRE(1240);
  motor2.configureOperatingModeAndSensor(3, 1);
  motor2.configureCommandMode(12);
  
  motor3.setELECANGLEOFFSET(1316338944);
  motor3.setSINCOSCENTRE(1254);
  motor3.configureOperatingModeAndSensor(3, 1);
  motor3.configureCommandMode(12);
  
  motor4.setELECANGLEOFFSET(1345594368);
  motor4.setSINCOSCENTRE(1264);
  motor4.configureOperatingModeAndSensor(3, 1);
  motor4.configureCommandMode(12);
  
  delay(500);
  
  // TOF Sensors are already instantiated (addresses 0x50 to 0x57)
  
  Serial.println("Setup complete.");
}

// ---------- Global Variables for Starting Position ----------
bool startPositionSet = false;
float startX = 0.0, startY = 0.0;

// ---------- Global Variables for Current Position & Heading ----------
float currentX = 0.0, currentY = 0.0;
float currentHeading = 0.0;  // relative yaw in degrees

// ---------- Main Loop ----------
void loop() {
  delay(10);
  
  // --- IMU Reading ---
  if (bno08x.wasReset()) {
    Serial.println("IMU sensor was reset");
    setReports();
    initialYawSet = false;
  }
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  
  float imuRoll, imuPitch, imuYaw;
  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    float w = sensorValue.un.gameRotationVector.real;
    float ix = sensorValue.un.gameRotationVector.i;
    float iy = sensorValue.un.gameRotationVector.j;
    float iz = sensorValue.un.gameRotationVector.k;
    quaternionToEuler(w, ix, iy, iz, imuRoll, imuPitch, imuYaw);
    if (!initialYawSet) {
      initialYaw = imuYaw;
      initialYawSet = true;
    }
  }
  currentHeading = normalizeAngle180(imuYaw - initialYaw);
  Serial.print("Relative Yaw: ");
  Serial.println(currentHeading);
  
  // --- TOF Sensor Readings & Localization ---
  int tofReadings[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    tofReadings[i] = tofSensors[i].readDistance();
  }
  
  computeEstimatedPosition(currentX, currentY);
  Serial.print("Estimated Position (mm): X = ");
  Serial.print(currentX);
  Serial.print(" , Y = ");
  Serial.println(currentY);
  
  // --- Set Starting Position on First Valid Localization ---
  if (!startPositionSet && currentX >= 0 && currentY >= 0) {
    startX = currentX;
    startY = currentY;
    startPositionSet = true;
    Serial.print("Starting Position Set: X = ");
    Serial.print(startX);
    Serial.print(" , Y = ");
    Serial.println(startY);
  }
  
  // --- Compute Error from Starting Position ---
  float errorX = startX - currentX;
  float errorY = startY - currentY;
  float distanceError = sqrt(errorX * errorX + errorY * errorY);
  
  // Compute desired translation angle to move toward the starting position.
  // atan2(errorX, errorY) gives angle with 0° as forward.
  float desiredTranslationAngle = atan2(errorX, errorY) * 180.0 / PI;
  
  // Compute translation speed (full speed if error > threshold; scale down when close)
  float translationSpeedNormalized = (distanceError < 500) ? (distanceError / 500.0) : 1.0;
  
  // --- Compute Heading Correction ---
  float rotationCorrection = getHeadingCorrection(currentHeading);
  
  // --- Movement Command ---
  // Command the robot to move toward the starting position.
  setMotorSpeeds(desiredTranslationAngle, translationSpeedNormalized, rotationCorrection);
  
  // Debug prints:
  Serial.print("Error: dX = "); Serial.print(errorX);
  Serial.print(" dY = "); Serial.print(errorY);
  Serial.print(" | Distance Error = "); Serial.println(distanceError);
  
  delay(1000);
}
