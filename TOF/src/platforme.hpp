// /**
//   *
//   * Copyright (c) 2021 STMicroelectronics.
//   * All rights reserved.
//   *
//   * This software is licensed under terms that can be found in the LICENSE file
//   * in the root directory of this software component.
//   * If no LICENSE file comes with this software, it is provided AS-IS.
//   *
//   ******************************************************************************
//   */

<<<<<<< Updated upstream:TOF/src/platforme.hpp

#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>
=======
// #pragma once

// #include "picoIncludes.hpp"
// #include <stdint.h>
// #include <string.h>
// #include <stdio.h>

>>>>>>> Stashed changes:tof_test_and_control/platforme.hpp

// /**
//  * @brief Structure VL53L5CX_Platform needs to be filled by the customer,
//  * depending on his platform. At least, it contains the VL53L5CX I2C address.
//  * Some additional fields can be added, as descriptors, or platform
//  * dependencies. Anything added into this structure is visible into the platform
//  * layer.
//  */

// typedef struct
// {
// 	/* To be filled with customer's platform. At least an I2C address/descriptor
// 	 * needs to be added */
// 	/* Example for most standard platform : I2C address of sensor */
//     uint8_t  			address;
// 	i2c_inst_t* 		i2c;		//I2C instance

// } VL53L5CX_Platform;

// /*
//  * @brief The macro below is used to define the number of target per zone sent
//  * through I2C. This value can be changed by user, in order to tune I2C
//  * transaction, and also the total memory size (a lower number of target per
//  * zone means a lower RAM). The value must be between 1 and 4.
//  */

// #define 	VL53L5CX_NB_TARGET_PER_ZONE		1U

// /*
//  * @brief The macro below can be used to avoid data conversion into the driver.
//  * By default there is a conversion between firmware and user data. Using this macro
//  * allows to use the firmware format instead of user format. The firmware format allows
//  * an increased precision.
//  */

// // #define 	VL53L5CX_USE_RAW_FORMAT

// /*
//  * @brief All macro below are used to configure the sensor output. User can
//  * define some macros if he wants to disable selected output, in order to reduce
//  * I2C access.
//  */

// // #define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
// // #define VL53L5CX_DISABLE_NB_SPADS_ENABLED
// // #define VL53L5CX_DISABLE_NB_TARGET_DETECTED
// // #define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
// // #define VL53L5CX_DISABLE_RANGE_SIGMA_MM
// // #define VL53L5CX_DISABLE_DISTANCE_MM
// // #define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
// // #define VL53L5CX_DISABLE_TARGET_STATUS
// // #define VL53L5CX_DISABLE_MOTION_INDICATOR

// /**
//  * @brief Lit un octets depuis une adresse I2C
//  * @param p_platform pointeur vers les paramètres I2C
//  * @param Address adresse I2C du module
//  * @param p_values pointeur vers la valeur à lire
//  * @return status : 0 si OK
//  */
// uint8_t RdByte(
// 		VL53L5CX_Platform *p_platform,
// 		uint16_t registerAddress,
// 		uint8_t *p_value);

// /**
//  * @brief Ecrit un octet depuis une adresse I2C
//  * @param p_platform pointeur vers les paramètres I2C
//  * @param Address adresse I2C du module
//  * @param value valeur à écrire
//  * @return status : 0 si OK
//  */
// uint8_t WrByte(
// 		VL53L5CX_Platform *p_platform,
// 		uint16_t registerAddress,
// 		uint8_t value);

// /**
//  * @brief Lit plusieurs octets depuis une adresse I2C
//  * @param p_platform pointeur vers les paramètres I2C
//  * @param Address adresse I2C du module
//  * @param p_values pointeur vers le tableau des valeurs lues
//  * @param size taille du tableau
//  * @return status : 0 si OK
//  */
// uint8_t RdMulti(
// 		VL53L5CX_Platform *p_platform,
// 		uint16_t registerAddress,
// 		uint8_t *p_values,
// 		uint32_t size);

// /**
//  * @brief Ecrit plusieurs octets depuis une adresse I2C
//  * @param p_platform pointeur vers les paramètres I2C
//  * @param Address adresse I2C du module
//  * @param p_values pointeur vers le tableau des valeurs à écrire
//  * @param size taille du tableau
//  * @return status : 0 si OK
//  */
// uint8_t WrMulti(
// 		VL53L5CX_Platform *p_platform,
// 		uint16_t registerAddress,
// 		uint8_t *p_values,
// 		uint32_t size);

// /**
//  * @brief Reset le capteur
//  * @param p_platform pointeur vers les paramètres I2C
//  * @return status : 0 si OK
//  */
// uint8_t Reset_Sensor(
// 		VL53L5CX_Platform *p_platform);

<<<<<<< Updated upstream:TOF/src/platforme.hpp
/**
 * @brief Mandatory function, used to swap a buffer. The buffer size is always a
 * multiple of 4 (4, 8, 12, 16, ...).
 * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
 * @param (uint16_t) size : Buffer size to swap
 */
void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size);
/**
 * @brief Mandatory function, used to wait during an amount of time. It must be
 * filled as it's used into the API.
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint32_t) TimeMs : Time to wait in ms.
 * @return (uint8_t) status : 0 if wait is finished.
 */
uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs);

#endif	// _PLATFORM_H_
=======
// /**
//  * @brief Mandatory function, used to swap a buffer. The buffer size is always a
//  * multiple of 4 (4, 8, 12, 16, ...).
//  * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
//  * @param (uint16_t) size : Buffer size to swap
//  */
// void SwapBuffer(
// 		uint8_t 		*buffer,
// 		uint16_t 	 	 size);
// /**
//  * @brief Mandatory function, used to wait during an amount of time. It must be
//  * filled as it's used into the API.
//  * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
//  * structure.
//  * @param (uint32_t) TimeMs : Time to wait in ms.
//  * @return (uint8_t) status : 0 if wait is finished.
//  */
// uint8_t WaitMs(
// 		VL53L5CX_Platform *p_platform,
// 		uint32_t TimeMs);
>>>>>>> Stashed changes:tof_test_and_control/platforme.hpp
