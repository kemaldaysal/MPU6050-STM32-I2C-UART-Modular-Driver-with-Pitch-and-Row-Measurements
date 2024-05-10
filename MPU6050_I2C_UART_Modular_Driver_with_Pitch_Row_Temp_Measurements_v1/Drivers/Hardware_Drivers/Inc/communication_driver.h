/*
 * sensordriver.h
 *
 *  Created on: Apr 30, 2024
 *      Author: Kemal
 */

#ifndef HARDWARE_DRIVERS_INC_COMMUNICATION_LAYER_DRIVER_H_
#define HARDWARE_DRIVERS_INC_COMMUNICATION_LAYER_DRIVER_H_

#include "stdint.h"
#include "stm32f0xx_hal.h"

#define I2C_TIMEOUT_DURATION ( HAL_MAX_DELAY )
#define I2C_NUMBER_OF_TRIALS ((uint8_t) 4 )
#define I2C_REG_ADD_SIZE_1_BYTE ((uint8_t) 1)
#define I2C_DATA_SIZE_1_BYTE ((uint8_t) 1)
#define I2C_DATA_SIZE_2_BYTES ((uint8_t) 2)

// Sensor feedback options
typedef enum {
	SENSOR_OK,
	SENSOR_ERROR
}sensor_status_e;

sensor_status_e sensor_check_with_HAL(uint8_t chip_address);
uint8_t sensor_read_register8 (uint8_t chip_address, uint8_t reg_address);
uint16_t sensor_read_register16 (uint8_t chip_address, uint8_t reg_address);
sensor_status_e sensor_read_bytes(uint8_t chip_address, uint8_t reg_address, uint8_t *pBuffer, uint8_t size);
sensor_status_e sensor_write_register8 (uint8_t chip_address, uint8_t reg_address, uint8_t value);
sensor_status_e sensor_write_register16 (uint8_t chip_address, uint8_t reg_address, uint16_t value);
sensor_status_e sensor_write_bytes(uint8_t chip_address, uint8_t reg_address, uint8_t *pBuffer, uint8_t size);

#endif /* HARDWARE_DRIVERS_INC_COMMUNICATION_LAYER_DRIVER_H_ */
