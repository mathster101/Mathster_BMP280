#include <Mathster_BMP280.h>

/*
* Library for the BMP280 Pressure and Temperature Sensor
* Uses I2C
* Developed by Mathew P.
*/




BMP280_Mathster BMP280;

void setup() {
  BMP280.initialize();
  BMP280.set_temperature_oversampling(2);
  BMP280.set_pressure_oversampling(4);
  BMP280.set_iir_coefficients(4);
  Serial.begin(115200);
  //BMP280.register_dump();
  Serial.println("_________________________________________");
}

void loop() {
  // put your main code here, to run repeatedly:
  float Temperature, Pressure, Altitude;
  Temperature = BMP280.get_temperature();
  Pressure = BMP280.get_pressure();
  Altitude = BMP280.get_altitude();
  Serial.print("Temperature (C) = ");
  Serial.println(Temperature);
  Serial.print("Pressure (Pa) = ");
  Serial.println(Pressure);
  Serial.print("Altitude (m) = ");
  Serial.println(Altitude);
  Serial.println("_________________________________________");
  delay(500);
}
