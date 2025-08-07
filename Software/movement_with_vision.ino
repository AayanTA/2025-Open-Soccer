#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "PowerfulBLDCdriver.h"

#define BNO08X_RESET -1
#define SERIAL_BAUD 115200

// --- Motor Setup ---
// Motor 1: Front Left
// Motor 2: Front Right
// Motor 3: Back Right
// Motor 4: Back Left
PowerfulBLDCdriver motor1, motor2, motor3, motor4;

// --- IMU Setup ---
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
bool initialYawSet = false;
float initialYaw = 0.0;

// --- Heading Correction Constants ---
const float headingKp = 1.0;

// --- USB Serial Parsing State ---
String serialBuf = "";        // accumulate incoming chars
float desiredAngle = 0.0;     // from Pi

// --- Prototypes ---
void setReports();
void quaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw);
float normalizeAngle180(float angle);
void setMotorSpeeds(float translationAngleDeg, float translationSpeedNormalized, float rotation);
void spinAround(float speedNormalized);

void setup() {
  // USB‐Serial
  Serial.begin(SERIAL_BAUD);
  while(!Serial);

  // IMU
  Wire.setSCL(9);
  Wire.setSDA(8);
  Wire.begin();
  Wire.setClock(1000000);
  if (!bno08x.begin_I2C(74, &Wire)) {
    Serial.println("Failed to find BNO08x");
    while(1);
  }
  setReports();

  // Motors
  motor1.begin(25, &Wire);
  motor2.begin(26, &Wire);
  motor3.begin(27, &Wire);
  motor4.begin(28, &Wire);
  // (apply all your calibration and PID setup here as before...)

  Serial.println("Pico ready. Awaiting commands in format: \"XXX YYY ZZZ\" (angle=ZZZ)");
}

void loop() {
  // 1) Read and parse serial lines
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        int xVal, yVal, aVal;
        // parse three ints
        if (sscanf(serialBuf.c_str(), "%d %d %d", &xVal, &yVal, &aVal) == 3) {
          desiredAngle = (float)aVal;
          if (desiredAngle < 0) desiredAngle += 360.0;
          else if (desiredAngle >= 360) desiredAngle = fmod(desiredAngle, 360.0);
        } else {
          Serial.print("Parse error: "); Serial.println(serialBuf);
        }
        serialBuf = "";
      }
    } else {
      serialBuf += c;
    }
  }

  // 2) Read IMU for heading correction
  if (bno08x.wasReset()) {
    setReports();
    initialYawSet = false;
  }
  if (bno08x.getSensorEvent(&sensorValue)
      && sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    float w = sensorValue.un.gameRotationVector.real;
    float i = sensorValue.un.gameRotationVector.i;
    float j = sensorValue.un.gameRotationVector.j;
    float k = sensorValue.un.gameRotationVector.k;
    float roll, pitch, yaw;
    quaternionToEuler(w, i, j, k, roll, pitch, yaw);
    if (!initialYawSet) {
      initialYaw = yaw;
      initialYawSet = true;
    }
    float relativeYaw = normalizeAngle180(yaw - initialYaw);
    float correction = headingKp * (relativeYaw / 180.0);
    correction = constrain(correction, -1.0, 1.0);

    // 3) Move using desiredAngle from Pi, full speed (1.0) for now
    setMotorSpeeds(desiredAngle, 1.0, correction);
  }

  delay(10);
}

// ——— IMU & Utility Functions ———

void setReports() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR);
}

void quaternionToEuler(float w, float x, float y, float z,
                       float &roll, float &pitch, float &yaw) {
  float sinr = 2*(w*x + y*z), cosr = 1 - 2*(x*x + y*y);
  roll = atan2(sinr, cosr);
  float sinp = 2*(w*y - z*x);
  pitch = fabs(sinp)>=1 ? copysign(PI/2, sinp) : asin(sinp);
  float siny = 2*(w*z + x*y), cosy = 1 - 2*(y*y + z*z);
  yaw = atan2(siny, cosy);
  roll  = roll  * 180.0/PI;
  pitch = pitch * 180.0/PI;
  yaw   = yaw   * 180.0/PI;
}

float normalizeAngle180(float angle) {
  while (angle > 180) angle -= 360;
  while (angle <= -180) angle += 360;
  return angle;
}

// ——— Movement (as before) ———

void setMotorSpeeds(float translationAngleDeg,
                    float translationSpeedNormalized,
                    float rotation) {
  float rad = translationAngleDeg * PI/180.0;
  float drive  = translationSpeedNormalized * cos(rad);
  float strafe = translationSpeedNormalized * sin(rad);
  float m1 = -(drive + strafe + rotation);
  float m2 =  (drive - strafe - rotation);
  float m3 =  (drive + strafe - rotation);
  float m4 = -(drive - strafe + rotation);
  float arr[4]={m1,m2,m3,m4}, maxv=0;
  for(int i=0;i<4;i++) maxv = max(maxv, fabs(arr[i]));
  if (maxv>1) for(int i=0;i<4;i++) arr[i]/=maxv;
  long maxSpeed=60000000;
  motor1.setSpeed((long)(arr[0]*maxSpeed));
  motor2.setSpeed((long)(arr[1]*maxSpeed));
  motor3.setSpeed((long)(arr[2]*maxSpeed));
  motor4.setSpeed((long)(arr[3]*maxSpeed));
}
