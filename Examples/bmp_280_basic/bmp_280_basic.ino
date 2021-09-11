#include <Mathster_BMP280.h>

BMP280_Mathster BMP280;

void setup() {
  BMP280.initialize();
  Serial.begin(115200);
  Serial.println("_________________________________________");
}

void loop() {
  // put your main code here, to run repeatedly:
  float Temperature, Pressure, Altitude;
  Temperature = BMP280.get_temperature();
  Pressure = BMP280.get_pressure();
  Altitude = BMP280.get_altitude();
  Serial.print("Temperature (C) = ");Serial.println(Temperature);
  Serial.print("Pressure (Pa) = ");Serial.println(Pressure);
  Serial.print("Altitude (m) = ");Serial.println(Altitude);
  Serial.println("_________________________________________");
  delay(5000);
}
