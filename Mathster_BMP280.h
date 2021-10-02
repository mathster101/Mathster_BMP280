/*
* Library for the BMP280 Pressure and Temperature Sensor
* Uses I2C
* Developed by Mathew P.
* Datasheet can be found at [shorturl.at/alzDG] (Bosch website)
* Thanks to Bosch SensorTec for calculation details [https://github.com/BoschSensortec/BMP280_driver]
*/


#include "Arduino.h"
#include "Wire.h"

#define DEVICE_ADDRESS 0x76
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5
#define TEMP_REG_START 0xFA
#define PRESS_REG_START 0xF7
#define CALIBRATION_REG_START 0x88

class BMP280_Mathster {
public:
  void initialize();
  float get_pressure();
  float get_temperature();
  float get_altitude();
  void set_temperature_oversampling(uint8_t option);
  void set_pressure_oversampling(uint8_t option);
  void set_iir_coefficients(uint8_t option);
  void sleep();
  void calibration_dump();

private:
  uint16_t dig_T1, dig_P1;                                                                 //calibration
  int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;  //calibration
  int32_t t_fine;                                                                          //internal use only
  int temp_internal;                                                                       //internal use only
  float barometric_constant;                                                               //for altitude calculations;
  uint8_t i2c_read_byte(const uint8_t addr);
  void i2c_read_bytes(const uint8_t addr, uint8_t* buffer_to_fill, int num_bytes);
  bool i2c_write_byte(const uint8_t addr, const uint8_t data_byte);
};

