#include <Wire.h>

const int numSensors = 8;
const int numReadings = 5; // Moving average window size

class SteelBarToF {
  private:
    uint8_t address;
    TwoWire* wire;
    int readings[numReadings]; // Array to store sensor readings
    int sum;                   // Sum of the readings
    int index;                 // Current index in the array
    bool filled;               // Flag to check if array is filled

  public:
    SteelBarToF(uint8_t _address, TwoWire* _wire)
      : address(_address), wire(_wire), sum(0), index(0), filled(false) {
        // Initialize readings array
        for (int i = 0; i < numReadings; i++) {
          readings[i] = 0;
        }
      }

    uint8_t getAddress() const {
      return address;
    }

    int readDistance() {
      if (!wire) return -1;

      wire->beginTransmission(address);
      wire->write(0x10); // Command to request distance
      if (wire->endTransmission() != 0) {
        return -1; // Sensor not responding
      }

      if (wire->requestFrom(address, 5) == 5) {
        wire->read(); // Sequence byte (not used)
        int distance = (wire->read() | (wire->read() << 8) | (wire->read() << 16) | (wire->read() << 24));

        // Update the moving average
        sum -= readings[index];     // Subtract the oldest reading
        readings[index] = distance; // Add the new reading
        sum += distance;            // Update the sum

        index = (index + 1) % numReadings; // Move to the next index

        if (index == 0) filled = true; // Array is filled after the first cycle

        return filled ? sum / numReadings : sum / (index + 1); // Return average
      }

      return -1; // Read failed
    }
};

// Declare sensors with unique I2C addresses
SteelBarToF sensors[numSensors] = {
  SteelBarToF(0x50, &Wire),
  SteelBarToF(0x51, &Wire),
  SteelBarToF(0x52, &Wire),
  SteelBarToF(0x53, &Wire),
  SteelBarToF(0x54, &Wire),
  SteelBarToF(0x55, &Wire),
  SteelBarToF(0x56, &Wire),
  SteelBarToF(0x57, &Wire)
};

void setup() {
  Serial.begin(115200);
  Wire.setSDA(8); // Set your SDA pin
  Wire.setSCL(9); // Set your SCL pin
  Wire.begin();

  Serial.println("Setup complete. Starting measurements...");
}

void loop() {
  String output = "Averaged Sensor Readings: ";
  for (int i = 0; i < numSensors; i++) {
    int avgDistance = sensors[i].readDistance();
    output += "S";
    output += i;
    output += "(0x";
    output += String(sensors[i].getAddress(), HEX);
    output += "): ";
    if (avgDistance >= 0) {
      output += String(avgDistance);
      output += " mm";
    } else {
      output += "Invalid";
    }
    if (i < numSensors - 1) {
      output += " | ";
    }
  }
  Serial.println(output);
  delay(1000); // Delay between readings
}
