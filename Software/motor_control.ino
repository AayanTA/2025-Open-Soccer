#include "Wire.h"
#include "PowerfulBLDCdriver.h"

PowerfulBLDCdriver motor1;
PowerfulBLDCdriver motor2;
PowerfulBLDCdriver motor3;
PowerfulBLDCdriver motor4;

// Stub for heading correction – modify to read your IMU and compute a rotational offset (normalized to roughly -1 to +1)
float correctHeading() {
  // For now, no correction is applied.
  return 0.0;
}

void setup() {
  Wire.setSCL(9);
  Wire.setSDA(8);
  Serial.begin(115200); // initialise serial
  Wire.begin();         // initialise I2C0
  Wire.setClock(1000000); // set I2C speed to 1MHz

  // Initialize motors with addresses 25,26,27,28:
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
  
  // Set each motor's calibration values (they are individual)
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
}

// This function sets the translation speeds (with optional heading correction)
// translationAngleDeg: desired direction of movement (0-360, with 0° = forward, 90° = right)
// translationSpeedNormalized: a value from 0 to 1 indicating how fast to move
void setMotorSpeeds(float translationAngleDeg, float translationSpeedNormalized) {
  // Convert desired translation angle to radians
  float cmdDirRad = translationAngleDeg * PI / 180.0;
  
  // For an X formation, assume the effective force directions (in degrees) are:
  // Motor1: 135°, Motor2: 225°, Motor3: 315°, Motor4: 45°
  float forceAngles[4] = {
    135.0 * PI / 180.0,
    225.0 * PI / 180.0,
    315.0 * PI / 180.0,
    45.0  * PI / 180.0
  };
  
  // Compute each motor's translation component using:
  //   component = translationSpeedNormalized * sin(cmdDirRad - forceAngle)
  float motorComponents[4];
  for (int i = 0; i < 4; i++) {
    motorComponents[i] = translationSpeedNormalized * sin(cmdDirRad - forceAngles[i]);
  }
  
  // Get a rotational (heading) correction (for example from a PID using an IMU)
  float rotationComponent = correctHeading();  // Expected to be in roughly [-1, 1]
  
  // Combine translation and rotation. For a basic approach, add the same rotational term to each motor.
  float motorSpeeds[4];
  for (int i = 0; i < 4; i++) {
    motorSpeeds[i] = motorComponents[i] + rotationComponent;
  }
  
  // Normalize the motor speeds if any exceed an absolute value of 1
  float maxVal = 0;
  for (int i = 0; i < 4; i++) {
    if (fabs(motorSpeeds[i]) > maxVal) {
      maxVal = fabs(motorSpeeds[i]);
    }
  }
  if (maxVal > 1.0) {
    for (int i = 0; i < 4; i++) {
      motorSpeeds[i] /= maxVal;
    }
  }
  
  // Scale normalized speeds to the motor's speed range.
  // For example, if max speed is 60000000:
  float maxMotorSpeed = 60000000.0;
  long scaledSpeeds[4];
  for (int i = 0; i < 4; i++) {
    scaledSpeeds[i] = (long)(motorSpeeds[i] * maxMotorSpeed);
  }
  
  // Set the motor speeds
  motor1.setSpeed(scaledSpeeds[0]);
  motor2.setSpeed(scaledSpeeds[1]);
  motor3.setSpeed(scaledSpeeds[2]);
  motor4.setSpeed(scaledSpeeds[3]);
  
  // For debugging:
  Serial.print("Motor speeds: ");
  Serial.print(scaledSpeeds[0]); Serial.print(" ");
  Serial.print(scaledSpeeds[1]); Serial.print(" ");
  Serial.print(scaledSpeeds[2]); Serial.print(" ");
  Serial.println(scaledSpeeds[3]);
}

// Example function to spin the robot in place.
// speedNormalized: a value from -1 to 1; positive for one direction, negative for the other.
void spinAround(float speedNormalized) {
  float maxMotorSpeed = 45000000.0;
  long scaledSpeed = (long)(speedNormalized * maxMotorSpeed);
  // For a simple spin in an X drive, one common pattern is to command:
  // motors 1 & 2: forward; motors 3 & 4: reverse (adjust signs as needed based on mounting)
  motor1.setSpeed(scaledSpeed);
  motor2.setSpeed(scaledSpeed);
  motor3.setSpeed(-scaledSpeed);
  motor4.setSpeed(-scaledSpeed);
}

void loop() {
  // Set the desired translation direction and speed.
  // For example, to move in a specified angle (0 to 360 degrees; here 90 means right):
  float desiredAngle = 90;      // degrees
  float desiredSpeed = 1.0;     // normalized speed (0 to 1)
  
  // To move in translation:
  setMotorSpeeds(desiredAngle, desiredSpeed);
  
  // Alternatively, to spin in place, you can call:
  // spinAround(1.0);
  
  delay(1000); // Adjust delay as needed
}
