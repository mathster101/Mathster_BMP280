#include "Arduino.h"
#include "Wire.h"


class BMP280_Mathster {
public:
  void initialize();
  double get_pressure();
  float get_temperature();

private:
  uint8_t device_address = 0x76;
  uint8_t ctrl_meas = 0xF4;
  uint8_t config = 0xF5;
  uint8_t temp_reg_start = 0xFA;                                                           //till and including 0xFC
  uint8_t press_reg_start = 0xF7;                                                          //till and including 0xF9;
  uint16_t dig_T1, dig_P1;                                                                 //calibration
  uint8_t calibration_reg_start = 0x88;                                                    //till and including 0x9F ,i.e. 24 bytes
  int32_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;  //calibration
  int32_t t_fine = -100000;                                                                          //internal use only

  uint8_t i2c_read_byte(const uint8_t addr);
  uint8_t* i2c_read_bytes(const uint8_t addr, uint8_t* buffer_to_fill, int num_bytes);
  uint8_t i2c_write_byte(const uint8_t addr, const uint8_t data_byte);
};