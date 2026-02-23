/*
 Flying Sword - Drone Receiver Firmware
 ESP32 + HC-05 -> Pixhawk/Flight Controller
 
 Wiring:
 HC-05 -> ESP32: VCC(5V), GND, TX -> RX2(GPIO 16), RX -> TX2(GPIO 17)
 ESP32 -> FC: TX1(GPIO 1) -> FC RX, RX1(GPIO 3) -> FC TX (MAVLink Port)
*/

#include <MAVLink.h> // Ensure MAVLink library is installed

// Hardware Serial for Bluetooth (HC-05)
#define RXD2 16
#define TXD2 17

struct FlightCommand {
  int8_t throttle;
  int8_t pitch;
  int8_t roll;
  int8_t yaw;
  uint8_t grip;
};

void setup() {
  Serial.begin(115200); // USB Debug
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Bluetooth (HC-05)
  
  // Serial1 for MAVLink (to Flight Controller)
  Serial1.begin(57600, SERIAL_8N1, 12, 13); // Adjust pins based on availability
  
  Serial.println("Flying Sword Drone Receiver Initialized");
}

void loop() {
  if (Serial2.available() >= 7) {
    if (Serial2.read() == 0xAA) {
      uint8_t buffer[6];
      Serial2.readBytes(buffer, 6);
      
      uint8_t checksum = buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3] ^ buffer[4];
      
      if (checksum == buffer[5]) {
        FlightCommand cmd;
        cmd.throttle = (int8_t)buffer[0];
        cmd.pitch    = (int8_t)buffer[1];
        cmd.roll     = (int8_t)buffer[2];
        cmd.yaw      = (int8_t)buffer[3];
        cmd.grip     = buffer[4];
        
        sendMavlinkOverride(cmd);
        
        // Debug Print
        Serial.printf("T:%d P:%d R:%d Y:%d G:%d\n", cmd.throttle, cmd.pitch, cmd.roll, cmd.yaw, cmd.grip);
      }
    }
  }
}

void sendMavlinkOverride(FlightCommand cmd) {
  // Map -45/45 degree commands to 1000-2000ms PWM for MAVLink override
  uint16_t ch_roll = map(cmd.roll, -45, 45, 1000, 2000);
  uint16_t ch_pitch = map(cmd.pitch, -45, 45, 1000, 2000);
  uint16_t ch_throttle = map(cmd.throttle, 0, 100, 1000, 2000);
  uint16_t ch_yaw = 1500; // Neutral

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // RC_CHANNELS_OVERRIDE (Channel 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw)
  mavlink_msg_rc_channels_override_pack(1, 200, &msg, 1, 1, 
                                        ch_roll, ch_pitch, ch_throttle, ch_yaw, 
                                        0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}
