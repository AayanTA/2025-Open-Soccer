#include <Adafruit_BNO08x.h>
#include <Wire.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Global variables to store the initial yaw reference
bool initialYawSet = false;
float initialYaw = 0.0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // wait for serial port to open

  Serial.println("Adafruit BNO08x test!");

  Wire.setSCL(9);
  Wire.setSDA(8);
  Wire.begin();
  
  // Try to initialize!
  if (!bno08x.begin_I2C(74, &Wire)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector report");
  }
}

void quaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    // Convert radians to degrees
    roll  = roll  * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw   = yaw   * 180.0 / M_PI;
}

float normalizeAngle180(float angle) {
  // Normalize angle to range [-180, 180)
  while (angle >= 180.0) {
    angle -= 360.0;
  }
  while (angle < -180.0) {
    angle += 360.0;
  }
  return angle;
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset");
    setReports();
    // Reset initial yaw if needed
    initialYawSet = false;
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
    float w = sensorValue.un.gameRotationVector.real;
    float x = sensorValue.un.gameRotationVector.i;
    float y = sensorValue.un.gameRotationVector.j;
    float z = sensorValue.un.gameRotationVector.k;

    float roll, pitch, yaw;
    quaternionToEuler(w, x, y, z, roll, pitch, yaw);

    // On first reading, store the initial yaw as reference.
    if (!initialYawSet) {
      initialYaw = yaw;
      initialYawSet = true;
    }

    // Compute relative yaw by subtracting the initial reference.
    float relativeYaw = yaw - initialYaw;

    // Normalize to range [-180, 180)
    relativeYaw = normalizeAngle180(relativeYaw);

    Serial.print("Yaw (-180 to 180, 0 is straight ahead): ");
    Serial.println(relativeYaw);
  }
}
