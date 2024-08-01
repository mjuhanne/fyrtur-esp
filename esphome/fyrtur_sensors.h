#include "esphome.h"

class Blindsstatus : public Component, public UARTDevice {
 public:
  Blindsstatus(UARTComponent *parent) : UARTDevice(parent) {}
  Sensor *battery_sensor = new Sensor();
  Sensor *voltage_sensor = new Sensor();
  Sensor *speed_sensor = new Sensor();
  Sensor *position_sensor = new Sensor();
  byte buffer[8];
  int i = 0;

  void setup() override {
    // nothing to do here
  }
  void loop() override {
   while (available()) {
    byte b = read();
//  ESP_LOGD("DEBUG", "Incoming byte is: %02x", b);
    buffer[i] = b;
    if (i == 7) {
     i = 0;
     if (buffer[0] == 0x00 && buffer[1] == 0xff && buffer[2] == 0xd8) { // Check Header
//    ESP_LOGD("DEBUG", "Received Header");
      byte checksum = buffer[3] ^ buffer[4] ^ buffer[5] ^ buffer[6];
      if (buffer[7] == checksum) { // Check Checksum
//     ESP_LOGD("DEBUG", "Checksum OK");
       int battery_level = int(buffer[3]);
       int voltage = int(buffer[4]);
       int speed = int(buffer[5]);
       int position = int(buffer[6]);
       battery_sensor->publish_state(battery_level);
//     ESP_LOGD("DEBUG", "Battery is: %i", battery_level);
       voltage_sensor->publish_state(voltage);
//     ESP_LOGD("DEBUG", "Voltage is: %i", voltage);
       speed_sensor->publish_state(speed);
//     ESP_LOGD("DEBUG", "Speed is: %i", speed);
       position_sensor->publish_state(position);
//     ESP_LOGD("DEBUG", "Position is: %i", position);
      }
     }
    } else {
     i++;
    }
   }
  }
};
