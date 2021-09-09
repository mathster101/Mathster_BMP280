#include "Mathster_BMP280.h"
#include "Wire.h"


uint8_t BMP280_Mathew::i2c_read_byte(const uint8_t addr) {
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(device_address, 1);
  while (Wire.available() != 1)
    ; // wait till device is actually available
  return (uint8_t)Wire.read();
}

uint8_t* BMP280_Mathew::i2c_read_bytes(const uint8_t addr, uint8_t* buffer_to_fill, int num_bytes) {
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(device_address, num_bytes);
  while (Wire.available() != num_bytes)
      ; // wait till device is actually available
  for (int i = 0; i < num_bytes; i++) {
    buffer_to_fill[i] = Wire.read();
  }
  return buffer_to_fill;
}

uint8_t BMP280_Mathew::i2c_write_byte(const uint8_t addr, const uint8_t data_byte) {
  Wire.beginTransmission(device_address);
  Wire.write(addr);
  Wire.write(data_byte);
  Wire.endTransmission();
  if (i2c_read_byte(addr) == data_byte)
    return true;
  else
    return false;
}

