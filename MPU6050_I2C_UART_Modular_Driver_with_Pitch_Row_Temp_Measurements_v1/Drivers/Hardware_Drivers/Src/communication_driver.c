#include <communication_driver.h>
#include "stm32f0xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

sensor_status_e sensor_check_with_HAL(uint8_t chip_address) {

	HAL_StatusTypeDef status;

	status = HAL_I2C_IsDeviceReady(&hi2c1, chip_address, I2C_NUMBER_OF_TRIALS,
	I2C_TIMEOUT_DURATION);

	if (HAL_OK == status) {

		return SENSOR_OK;

	} else {

		return SENSOR_ERROR;

	}
}

uint8_t sensor_read_register8(uint8_t chip_address, uint8_t reg_address) {

	HAL_StatusTypeDef status;

	uint8_t data;
	status = HAL_I2C_Mem_Read(&hi2c1, chip_address, reg_address,
	I2C_REG_ADD_SIZE_1_BYTE, &data, I2C_DATA_SIZE_1_BYTE,
	I2C_TIMEOUT_DURATION);

	if (HAL_OK != status) {

		return SENSOR_ERROR;

		// to do
	} else {
		return data;
	}
}

uint16_t sensor_read_register16(uint8_t chip_address, uint8_t reg_address) {

	HAL_StatusTypeDef status;

	uint8_t data[2];
	uint16_t retVal;

	status = HAL_I2C_Mem_Read(&hi2c1, chip_address, reg_address,
	I2C_REG_ADD_SIZE_1_BYTE, &data, I2C_DATA_SIZE_2_BYTES,
	I2C_TIMEOUT_DURATION);

	if (HAL_OK != status) {

		return SENSOR_ERROR;

		// to do
	} else {
		retVal = (data[1] << 8) | data[0];
		return retVal;

	}

}

sensor_status_e sensor_read_bytes(uint8_t chip_address, uint8_t reg_address, uint8_t *pBuffer, uint8_t size)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Read(&hi2c1, chip_address, reg_address, I2C_REG_ADD_SIZE_1_BYTE, pBuffer, size, I2C_TIMEOUT_DURATION);

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}

sensor_status_e sensor_write_register8(uint8_t chip_address,
		uint8_t reg_address, uint8_t value) {

	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(&hi2c1, chip_address, reg_address,
	I2C_REG_ADD_SIZE_1_BYTE, &value, I2C_DATA_SIZE_1_BYTE,
	I2C_TIMEOUT_DURATION);

	if (HAL_OK != status) {

		return SENSOR_ERROR;

		// to do
	} else {

		return SENSOR_OK;

	}

}
sensor_status_e sensor_write_register16(uint8_t chip_address,
		uint8_t reg_address, uint16_t value) {

	HAL_StatusTypeDef status;

	uint8_t data[2];
	data[0] = (uint8_t) ((value >> 8) & 0xFF); // High Byte
	data[1] = (uint8_t) (value & 0xFF); // Low Byte

	status = HAL_I2C_Mem_Write(&hi2c1, chip_address, reg_address,
	I2C_REG_ADD_SIZE_1_BYTE, data, I2C_DATA_SIZE_2_BYTES,
	I2C_TIMEOUT_DURATION);

	if (HAL_OK != status) {

		return SENSOR_ERROR;

		// to do
	} else {

		return SENSOR_OK;

	}

}


sensor_status_e sensor_write_bytes(uint8_t chip_address, uint8_t reg_address, uint8_t *pBuffer, uint8_t size)
{
	HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(&hi2c1, chip_address, reg_address, I2C_REG_ADD_SIZE_1_BYTE, pBuffer, size, I2C_TIMEOUT_DURATION );

	if (HAL_OK != status) {
		return SENSOR_ERROR;
	} else {
		return SENSOR_OK;
	}
}
