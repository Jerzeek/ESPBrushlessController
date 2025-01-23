#include "Wire.h"

#define I2C_DEV_ADDR 0x04

uint32_t i = 0;

void setup() {
  Serial.begin(115200);
  while( ! Serial ) {}
  Serial.setDebugOutput(true);
  Serial.println("Started!");
  Wire.begin();
  Serial.println("Wire Started");
}

void loop() {
  if( Serial.available() ) {
    char command = Serial.read();
    uint16_t b1,b2 = 0;
    if( command == 'a' || command == 'A' || command == 'd' || command == 'D' )
      b1 = Serial.parseInt();
    if( command == 's' || command == 'S' ) {
      b1 = Serial.parseInt();
      b2 = Serial.parseInt();
    }
    Serial.readStringUntil('\n');
    Serial.println("Sending Command: " + String(command) + ", " + String(b1) + ", " + String(b2));
    Wire.beginTransmission(I2C_DEV_ADDR);
    Wire.write(command);
    Wire.write((byte *)&b1, sizeof(uint16_t));
    Wire.write((byte *)&b2, sizeof(uint16_t));
    uint8_t error = Wire.endTransmission(true);
    Serial.printf("endTransmission: %u\n", error);
  }
  delay(50);

}