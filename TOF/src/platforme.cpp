/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "platforme.hpp"

//Communication I2C :
// Ecriture :
// 	- write {index[15-8], index[7-0], data1[7-0], data2[7-0], ...}
// Lecture :
// 	- write {index[15-8], index[7-0]}
// 	- read	{data1[7-0], data2[7-0], ...}

uint8_t RdByte(
		VL53L5CX_Platform *p_platform,
		uint16_t registerAddress,
		uint8_t *p_value)
{
	int status = 255;

	//Conversion de l'index en 2x 8bits
	uint8_t regAddress[2] = {(uint8_t)(registerAddress >> 8), (uint8_t)(registerAddress & 0x00ff)};

	//Lecture
	i2c_write_blocking(i2c_default, p_platform->address, regAddress, 2, true);
	status = i2c_read_blocking(i2c_default, p_platform->address, p_value, 1, false);

	if (status == PICO_ERROR_GENERIC)		status = 255;
	else if (status ==  PICO_ERROR_TIMEOUT)	status = 254;
	else	status = 0;

	return (uint8_t)status;
}

uint8_t WrByte(
		VL53L5CX_Platform *p_platform,
		uint16_t registerAddress,
		uint8_t value)
{
	int status = 255;

	// Conversion registre 8bits + value
	uint8_t data[3] = { (uint8_t)(registerAddress >> 8), (uint8_t)(registerAddress & 0x00ff), value };
	
	//Tente d'écrire un octet
	status = i2c_write_blocking(i2c_default, p_platform->address, data, 3, false);

	if (status == PICO_ERROR_GENERIC)		status = 255;
	else if (status ==  PICO_ERROR_TIMEOUT)	status = 254;
	else	status = 0;

	return (uint8_t)status;
}

uint8_t WrMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t registerAddress,
		uint8_t *p_values,
		uint32_t size)
{
	int status = 255;

	// Conversion registre 8bits + donnée1 + donnée2 + ...
	uint8_t *data = (uint8_t*)malloc((size + 2) * sizeof(uint8_t));	
	data[0] = (uint8_t)(registerAddress >> 8);
	data[1] = (uint8_t)(registerAddress & 0x00ff);
 	for (int i = 0; i < size; i++)		data[i + 2] = p_values[i];

	//Tente d'écrire plusieurs octets
	status = i2c_write_blocking(i2c_default, p_platform->address, data, size + 2, false);

	if (status == PICO_ERROR_GENERIC)		status = 255;
	else if (status ==  PICO_ERROR_TIMEOUT)	status = 254;
	else	status = 0;

	free(data);

	return (uint8_t)status;
}

uint8_t RdMulti(
		VL53L5CX_Platform *p_platform,
		uint16_t registerAddress,
		uint8_t *p_values,
		uint32_t size)
{
	int status = 255;
	
	uint8_t regAddress[2] = { (uint8_t)(registerAddress >> 8), (uint8_t)(registerAddress & 0x00ff) };
	uint8_t *buffer = (uint8_t*)malloc(size * sizeof(uint8_t));

	//Lecture
	i2c_write_blocking(i2c_default, p_platform->address, regAddress, 2, true);
	status = i2c_read_blocking(i2c_default, p_platform->address, buffer, size, false);

	for (int i = 0; i < size; i++)	p_values[i] = buffer[i];
	
	if (status == PICO_ERROR_GENERIC)		status = 255;
	else if (status ==  PICO_ERROR_TIMEOUT)	status = 254;
	else	status = 0;

	free(buffer);
	
	return (uint8_t)status;
}

uint8_t Reset_Sensor(
		VL53L5CX_Platform *p_platform)
{
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	WaitMs(p_platform, 100);

	return status;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	uint8_t status = 255;

	sleep_ms(TimeMs);
	status = 0;

	return status;
}
