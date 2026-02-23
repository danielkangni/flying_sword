/*
 Flying Sword - Glove Firmware
 ESP32 + MPU6050 + HC-05
 
 Wiring:
 MPU6050 -> ESP32: VCC(3.3V), GND, SDA(GPIO 21), SCL(GPIO 22)
 HC-05 -> ESP32: VCC(5V), GND, TX -> RX2(GPIO 16), RX -> TX2(GPIO 17)
 Flex Sensors -> Analog Pins (TBD)
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <MahonyAHRS.h> // Ensure this library is installed

Adafruit_MPU6050 mpu;
Mahony filter;

// Serial for HC-05 (using UART2)
#define RXD2 16
#define TXD2 17

unsigned long lastUpdate = 0;
const float sampleFreq = 100.0f; // 100Hz

struct FlightCommand {
  int8_t throttle;
  int8_t pitch;
  int8_t roll;
  int8_t yaw;
  uint8_t grip;
};

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // HC-05 default baud rate is usually 9600

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  filter.begin(sampleFreq);
  
  Serial.println("Flying Sword Glove Initialized");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Update Mahony filter
  // Convert gyro from rad/s to deg/s if your Mahony lib requires it
  filter.updateIMU(g.gyro.x * 57.2958f, g.gyro.y * 57.2958f, g.gyro.z * 57.2958f, 
                   a.acceleration.x, a.acceleration.y, a.acceleration.z);

  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();

  // Mapping logic (Example)
  FlightCommand cmd;
  cmd.roll = (int8_t)constrain(roll, -45, 45);   // Map degree to command byte
  cmd.pitch = (int8_t)constrain(pitch, -45, 45);
  cmd.yaw = 0; // TBD via rotation or separate sensor
  cmd.throttle = 50; // TBD via flex sensor or vertical movement
  cmd.grip = 0;

  // Send packet to HC-05
  sendPacket(cmd);

  // Debug Print
  if (millis() - lastUpdate > 100) {
    Serial.printf("Roll: %.2f | Pitch: %.2f\n", roll, pitch);
    lastUpdate = millis();
  }

  delay(10); // Sync with 100Hz
}

void sendPacket(FlightCommand cmd) {
  // Simple binary protocol for efficiency
  // [Header] [T] [P] [R] [Y] [G] [Checksum]
  uint8_t packet[7];
  packet[0] = 0xAA; // Start Byte
  packet[1] = (uint8_t)cmd.throttle;
  packet[2] = (uint8_t)cmd.pitch;
  packet[3] = (uint8_t)cmd.roll;
  packet[4] = (uint8_t)cmd.yaw;
  packet[5] = cmd.grip;
  packet[6] = packet[1] ^ packet[2] ^ packet[3] ^ packet[4] ^ packet[5]; // Simple XOR Checksum

  Serial2.write(packet, 7);
}
