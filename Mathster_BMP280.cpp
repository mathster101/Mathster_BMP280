#include "Mathster_BMP280.h"
#include "Wire.h"


uint8_t BMP280_Mathster::i2c_read_byte(const uint8_t addr)
{
	uint8_t data_byte;
	Wire.beginTransmission(device_address);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(device_address, 1);
	while (Wire.available() != 1);// wait till device is actually available
	data_byte =  (uint8_t)Wire.read();
	return data_byte;
}

uint8_t* BMP280_Mathster::i2c_read_bytes(const uint8_t addr, uint8_t* buffer_to_fill, int num_bytes)
{
	Wire.beginTransmission(device_address);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(device_address, num_bytes);
	while (Wire.available() != num_bytes);// wait till device is actually available
	for (int i = 0; i < num_bytes; i++)
	{
		buffer_to_fill[i] = Wire.read();
	}
	return buffer_to_fill;
}

uint8_t BMP280_Mathster::i2c_write_byte(const uint8_t addr, const uint8_t data_byte)
{
	Wire.beginTransmission(device_address);
	Wire.write(addr);
	Wire.write(data_byte);
	Wire.endTransmission();
	if (i2c_read_byte(addr) == data_byte)
		return true;
	
	return false;
}

void BMP280_Mathster::initialize()
{
	uint8_t buffer[24];		
	Wire.begin();			// no need to re init wire in main code
	Wire.setClock(400000);	// BMP280 supports 400kHz i2c
	uint8_t default_control = i2c_read_byte(ctrl_meas);
	default_control = 0b01001011;
	i2c_write_byte(ctrl_meas, default_control);
	i2c_read_bytes(calibration_reg_start, buffer, 24);
	
	dig_T1 = (buffer[1] << 8  | buffer[0]);
	dig_T2 = (buffer[3] << 8  | buffer[2]);
	dig_T3 = (buffer[5] << 8  | buffer[4]);
	dig_P1 = (buffer[7] << 8  | buffer[6]);
	dig_P2 = (buffer[9] << 8  | buffer[8]);
	dig_P3 = (buffer[11] << 8 | buffer[10]);
	dig_P4 = (buffer[13] << 8 | buffer[12]);
	dig_P5 = (buffer[15] << 8 | buffer[14]);
	dig_P6 = (buffer[17] << 8 | buffer[16]);
	dig_P7 = (buffer[19] << 8 | buffer[18]);
	dig_P8 = (buffer[21] << 8 | buffer[20]);
	dig_P9 = (buffer[23] << 8 | buffer[22]);
	delay(50);
	get_temperature();// initializes t_fine

}

float BMP280_Mathster::get_temperature()
{
	uint8_t buffer[3];
	int32_t var1, var2;
	int32_t raw_temperature, calibrated_temperature;
	i2c_read_bytes(temp_reg_start, buffer, 3);

	raw_temperature = (int32_t)((((int32_t)(buffer[0])) << 12) | (((int32_t)(buffer[1])) << 4) | (((int32_t)(buffer[2])) >> 4));
	var1 = ((((raw_temperature >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((raw_temperature >> 4) - ((int32_t)dig_T1)) * ((raw_temperature >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	calibrated_temperature = (t_fine * 5 + 128) >> 8;
	return (float)calibrated_temperature / 100;
}

double BMP280_Mathster::get_pressure()
{
	uint8_t buffer[3];
	int32_t raw_pressure;
	int32_t var1, var2;
	uint32_t calibrated_pressure;
	get_temperature();//change later
	i2c_read_bytes(press_reg_start, buffer, 3);
	
	raw_pressure = (int32_t)((((int32_t)(buffer[0])) << 12) | (((int32_t)(buffer[1])) << 4) | (((int32_t)(buffer[2])) >> 4));
	var1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dig_P6);
	var2 = var2 + ((var1 * ((int32_t)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)dig_P4) << 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)dig_P1)) >> 15);
	if (var1 == 0) {
	return 0;	// error case
	}
	
	calibrated_pressure = (((uint32_t)(((int32_t)1048576) - raw_pressure) - (var2 >> 12))) * 3125;
	if (calibrated_pressure < 0x80000000)
	{
	calibrated_pressure = (calibrated_pressure << 1) / ((uint32_t)var1);
	}
	else
	{
	calibrated_pressure = (calibrated_pressure / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((calibrated_pressure >> 3) * (calibrated_pressure >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(calibrated_pressure >> 2)) * ((int32_t)dig_P8)) >> 13;
	calibrated_pressure = (uint32_t)((int32_t)calibrated_pressure + ((var1 + var2 + dig_P7) >> 4));
	return (double)calibrated_pressure; // pascals
}

void BMP280_Mathster::set_temperature_oversampling(uint8_t option)
{
	uint8_t current_val;
	current_val = i2c_read_byte(ctrl_meas);
	switch (option)
	{
	case 1:
		current_val = 0b00011111 & current_val; // temp sensor off
		break;
	case 2:
		current_val = (0b00011111 & current_val) | 0b00100000; //x1
		break;
	case 3:
		current_val = (0b00011111 & current_val) | 0b01000000;//x2
		break;
	case 4:
		current_val = (0b00011111 & current_val) | 0b01100000;//x4
		break;
	case 5:
		current_val = (0b00011111 & current_val) | 0b10000000;//x8
		break;
	default:
		current_val = (0b00011111 & current_val) | 0b11100000;//x16
		break;
	}
	i2c_write_byte(ctrl_meas, current_val);
}

void BMP280_Mathster::set_pressure_oversampling(uint8_t option)
{
	uint8_t current_val;
	current_val = i2c_read_byte(ctrl_meas);
	switch (option)
	{
	case 1:
		current_val = 0b11100011 & current_val; // press sensor off
		break;
	case 2:
		current_val = (0b11100011 & current_val) | 0b00000100; //x1
		break;
	case 3:
		current_val = (0b11100011 & current_val) | 0b00001000;//x2
		break;
	case 4:
		current_val = (0b11100011 & current_val) | 0b00001100;//x4
		break;
	case 5:
		current_val = (0b11100011 & current_val) | 0b00010000;//x8
		break;
	default:
		current_val = (0b11100011 & current_val) | 0b00011100;//x16
		break;
	}
	i2c_write_byte(ctrl_meas, current_val);
}

void BMP280_Mathster::set_iir_coefficients(uint8_t option)
{
	uint8_t current_val;
	current_val = i2c_read_byte(config);
	switch (option)
	{
	case 1:
		current_val = 0b11100011 & current_val; // iir filter off
		break;
	case 2:
		current_val = (0b11100011 & current_val) | 0b00000100; //2
		break;
	case 3:
		current_val = (0b11100011 & current_val) | 0b00001000;//4
		break;
	case 4:
		current_val = (0b11100011 & current_val) | 0b00001100;//8
		break;
	case 5:
		current_val = (0b11100011 & current_val) | 0b00010000;//16
		break;
	default:
		current_val = (0b11100011 & current_val) | 0b00011100;//x16
		break;
	}
	i2c_write_byte(config, current_val);
}

void BMP280_Mathster::sleep()
{
	uint8_t current_val;
	current_val = i2c_read_byte(ctrl_meas);
	current_val &= 0b11111100;
	Serial.println(current_val,BIN);
	i2c_write_byte(ctrl_meas, current_val);
}

void BMP280_Mathster::register_dump()
{
	uint8_t buffer[100];
	uint8_t start = 0x88, bytes = 100;
	i2c_read_bytes(start, buffer, bytes);
	for (int i = 0; i < bytes; i++)
	{
		Serial.print("Register : ");Serial.print(start + i,HEX);Serial.print("    val : ");Serial.println(buffer[i],HEX);
	}
}