//yaw is the direction that we care abt
// this one uses game rotation vector, which doesnt use the magnetometer so slightly less accurate
// but it starts at 0 yaw so that is nice :)

#include <Adafruit_BNO08x.h>
#include <Wire.h>

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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
    Serial.println("Could not enable geomagnetic rotation vector");
  }
}

void quaternionToEuler(float w, float x, float y, float z, float &roll, float &pitch, float &yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    pitch = fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    // Convert radians to degrees
    roll *= 180 / M_PI;
    pitch *= 180 / M_PI;
    yaw *= 180 / M_PI;

    // yaw is reversed (back of robot is 0) so convert
    if (yaw <= 0) {
      yaw += 180;
    } else {
      yaw -= 180;
    }
    
}

void loop() {
  delay(10);

  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
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

    Serial.print("Direction - Roll: ");
    Serial.print(roll);
    Serial.print(" degrees, Pitch: ");
    Serial.print(pitch);
    Serial.print(" degrees, Yaw: ");
    Serial.println(yaw); // yaw is here!
  }
}
