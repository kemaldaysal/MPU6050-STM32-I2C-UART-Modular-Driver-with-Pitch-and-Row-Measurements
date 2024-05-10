
#include <communication_driver.h>
#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c1;


sensor_status_e MPU6050_test_sensor_by_checking_ID_on_register(void) {

	uint8_t id = sensor_read_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_WHO_AM_I);

	if (MPU6050_I2C_ADDRESS_AD0_NOT_SHIFTED == id) {
		return SENSOR_OK;
	} else {
		return SENSOR_ERROR;
	}

}


sensor_status_e MPU6050_test_sensor_with_HAL(void) {

	sensor_status_e IsSensorAlive;

	IsSensorAlive = sensor_check_with_HAL(MPU6050_I2C_ADDRESS_AD0_SHIFTED);
	return IsSensorAlive;

}


sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange) {

	sensor_status_e status;

	// set gyro's range by changing register values for gyro ranges
	// give a feedback (return) about success or fail

	uint8_t gyro_config_register_data = sensor_read_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_GYRO_CONFIG);

	if (gyro_config_register_data != SENSOR_ERROR) {

		uint8_t new_register_data_to_be_written = (gyro_config_register_data
				| (gyroRange << MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION));

		status = sensor_write_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED,
				MPU_REG_GYRO_CONFIG, new_register_data_to_be_written);

		switch (gyroRange) {

		case FS_250:
			pSensor->gyro_co = 131.0;
			break;
		case FS_500:
			pSensor->gyro_co = 65.5;
			break;
		case FS_1000:
			pSensor->gyro_co = 32.8;
			break;
		case FS_2000:
			pSensor->gyro_co = 16.4;
			break;
		default:
			status = SENSOR_ERROR;
			break;

		}

		return status;

	} else {

		return SENSOR_ERROR;

	}


}

sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange) {

	sensor_status_e status;

	uint8_t acc_config_register_data = sensor_read_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_ACCEL_CONFIG);

	if (acc_config_register_data != SENSOR_ERROR) {

		uint8_t new_register_data_to_be_written = (acc_config_register_data | (accRange << MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION));

		sensor_write_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_ACCEL_CONFIG, new_register_data_to_be_written);

		switch (accRange) {

		case AFS_2G: pSensor->acc_co = 16384; break;
		case AFS_4G: pSensor->acc_co = 8192; break;
		case AFS_8G: pSensor->acc_co = 4096; break;
		case AFS_16G: pSensor->acc_co = 2048; break;
		default: status = SENSOR_ERROR; break;
		}

		return status;


	} else {

		return SENSOR_ERROR;

	}
}

sensor_status_e MPU6050_enable_temp_sensor(tempsensormode_e tempSensorCommand) {

	sensor_status_e temp_sensor_status;

	uint8_t pwrmgmt_config_register_data = sensor_read_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_PWR_MGMT_1);

	if (pwrmgmt_config_register_data != SENSOR_ERROR) {

		if (TEMPSENSOR_ON == tempSensorCommand) {

			// making it 1 disables the temp sensor
			pwrmgmt_config_register_data = (pwrmgmt_config_register_data & ~(1<<MPU_REG_TEMP_CONFIG_TEMP_ON_OFF_BITS_POSITION));

		} else if (TEMPSENSOR_OFF == tempSensorCommand) {

			// making it 0 enables the temp sensor
			pwrmgmt_config_register_data = (pwrmgmt_config_register_data | (1<<MPU_REG_TEMP_CONFIG_TEMP_ON_OFF_BITS_POSITION));

		}

		temp_sensor_status = sensor_write_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_PWR_MGMT_1, pwrmgmt_config_register_data);
		return temp_sensor_status;

	} else {

		return SENSOR_ERROR;
	}


}


sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode) {

	// setting SLEEP = 1 means SLEEPMODE_ON, SLEEP = 0 means SLEEPMODE_OFF

	sensor_status_e final_status;

	uint8_t pwr_mgmt_reg_data = sensor_read_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_PWR_MGMT_1);

	if (pwr_mgmt_reg_data != SENSOR_ERROR) {

		if (SLEEPMODE_ON == sleepmode) {

		pwr_mgmt_reg_data = (pwr_mgmt_reg_data | (1 << MPU_BIT_PWR_MGMT_1_SLEEP_MODE));

		} else if (SLEEPMODE_OFF == sleepmode) {

		pwr_mgmt_reg_data = (pwr_mgmt_reg_data & ~(1 << MPU_BIT_PWR_MGMT_1_SLEEP_MODE));

		}

		final_status = sensor_write_register8(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_PWR_MGMT_1, pwr_mgmt_reg_data);
		return final_status;

	} else {

		return SENSOR_ERROR;
	}

}

sensor_status_e MPU6050_initialize(SensorData_t *pSensor, fs_sel_e gyro_config, afs_sel_e acc_config, tempsensormode_e tempSensorMode) {

	sensor_status_e init_status;

	init_status = MPU6050_set_gyro_range(pSensor, gyro_config);

	if (init_status == SENSOR_OK) {

		init_status = MPU6050_set_acc_range(pSensor, acc_config);

		if (init_status == SENSOR_OK) {

			init_status = MPU6050_enable_temp_sensor(tempSensorMode);

			if (init_status == SENSOR_OK) {

				init_status = MPU6050_set_sleep_mode(SLEEPMODE_OFF);
			}
		}

	}

	return init_status;
}

sensor_status_e MPU6050_read_all_sensor_data_at_the_same_time_basic(SensorData_t *pSensorData) {

	sensor_status_e Final_Read_Status;

	// if all the data wanted to be read at once, for the sake of sampling at the same time,
	// 14 bytes of data will be needed to read. which is starting from register ACCEL_XOUT_H(59) to GYRO_ZOUT_L (72, including this)

	uint8_t data_buffer[14];

	Final_Read_Status = sensor_read_bytes(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_ACCEL_XOUT_H, data_buffer, 14);

	if (Final_Read_Status != SENSOR_ERROR) {

		pSensorData->accRaw.X = (int16_t) ((data_buffer[0] << 8) | (data_buffer[1]));
		pSensorData->accRaw.Y = (int16_t) ((data_buffer[2] << 8) | (data_buffer[3]));
		pSensorData->accRaw.Z = (int16_t) ((data_buffer[4] << 8) | (data_buffer[5]));

		pSensorData->tempRaw = (int16_t) ((data_buffer[6] << 8) | (data_buffer[7]));

		pSensorData->gyroRaw.X = (int16_t) ((data_buffer[8] << 8) | (data_buffer[9]));
		pSensorData->gyroRaw.Y = (int16_t) ((data_buffer[10] << 8) | (data_buffer[11]));
		pSensorData->gyroRaw.Z = (int16_t) ((data_buffer[12] << 8) | (data_buffer[13]));

		pSensorData->acc.X = (pSensorData->accRaw.X) / (pSensorData->acc_co);
		pSensorData->acc.Y = (pSensorData->accRaw.Y) / (pSensorData->acc_co);
		pSensorData->acc.Z = (pSensorData->accRaw.Z) / (pSensorData->acc_co);

		pSensorData->temp = (((pSensorData->tempRaw) / TEMP_CO)+ TEMP_OFFSET);

		pSensorData->gyro.X = (pSensorData->gyroRaw.X) / (pSensorData->gyro_co);
		pSensorData->gyro.Y = (pSensorData->gyroRaw.Y) / (pSensorData->gyro_co);
		pSensorData->gyro.Z = (pSensorData->gyroRaw.Z) / (pSensorData->gyro_co);


	}

	return Final_Read_Status;

}


sensor_status_e MPU6050_read_all_sensor_data_at_the_same_time_calibrated(SensorData_t *pSensorData) {



	static float setGyro = 0;

	static uint64_t time_start_2 = 0;
	static uint8_t time_elapsed_in_one_loop_2 = 0;

	time_start_2 = HAL_GetTick();

	sensor_status_e Final_Read_Status;

	// if all the data wanted to be read at once, for the sake of sampling at the same time,
	// 14 bytes of data will be needed to read. which is starting from register ACCEL_XOUT_H(59) to GYRO_ZOUT_L (72, including this)

	static uint8_t data_buffer[14];

	Final_Read_Status = sensor_read_bytes(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_ACCEL_XOUT_H, data_buffer, 14);

	if (Final_Read_Status != SENSOR_ERROR) {

		// Store the read data from registers into struct datatypes

		pSensorData->accRaw.X = (int16_t) ((data_buffer[0] << 8) | (data_buffer[1]));
		pSensorData->accRaw.Y = (int16_t) ((data_buffer[2] << 8) | (data_buffer[3]));
		pSensorData->accRaw.Z = (int16_t) ((data_buffer[4] << 8) | (data_buffer[5]));

		pSensorData->tempRaw = (int16_t) ((data_buffer[6] << 8) | (data_buffer[7]));

		pSensorData->gyroRaw.X = (int16_t) ((data_buffer[8] << 8) | (data_buffer[9]));
		pSensorData->gyroRaw.Y = (int16_t) ((data_buffer[10] << 8) | (data_buffer[11]));
		pSensorData->gyroRaw.Z = (int16_t) ((data_buffer[12] << 8) | (data_buffer[13]));

		/// Calibrating the data by subtracting the offsets

		pSensorData->accRawCalibrated.X = pSensorData->accRaw.X; // UNCALIBRATED, FIND SOMEWAY TO CALIBRATE !!
		pSensorData->accRawCalibrated.Y = pSensorData->accRaw.Y; // UNCALIBRATED, FIND SOMEWAY TO CALIBRATE !!
		pSensorData->accRawCalibrated.Z = pSensorData->accRaw.Z; // UNCALIBRATED, FIND SOMEWAY TO CALIBRATE !!


		pSensorData->gyroRawCalibrated.X = pSensorData->gyroRaw.X - pSensorData->gyroCalOffset.X;
		pSensorData->gyroRawCalibrated.Y = pSensorData->gyroRaw.Y - pSensorData->gyroCalOffset.Y;
		pSensorData->gyroRawCalibrated.Z = pSensorData->gyroRaw.Z - pSensorData->gyroCalOffset.Z;

		// Creating a complementary filter by taking advantage of the datasheet and putting the gyro and acc data into the filter to get meaningful values

		/// Interpreting the data

		pSensorData->acc.X = (pSensorData->accRawCalibrated.X) / (pSensorData->acc_co);
		pSensorData->acc.Y = (pSensorData->accRawCalibrated.Y) / (pSensorData->acc_co);
		pSensorData->acc.Z = (pSensorData->accRawCalibrated.Z) / (pSensorData->acc_co);

		pSensorData->temp = (((pSensorData->tempRaw) / TEMP_CO) + TEMP_OFFSET);

		// to convert it into angle, multiply it with 0.004 seconds, (degrees * seconds) / seconds = degrees
		pSensorData->gyro.X = ((pSensorData->gyroRawCalibrated.X) / (pSensorData->gyro_co)) * FIXED_CALCULATION_TIME_IN_SECONDS;
		pSensorData->gyro.Y = ((pSensorData->gyroRawCalibrated.Y) / (pSensorData->gyro_co)) * FIXED_CALCULATION_TIME_IN_SECONDS;
		pSensorData->gyro.Z = ((pSensorData->gyroRawCalibrated.Z) / (pSensorData->gyro_co)) * FIXED_CALCULATION_TIME_IN_SECONDS;


		pSensorData->gyroAnglePitch += pSensorData->gyro.X; // add each value found (integral) to final value to find x axis
		pSensorData->gyroAngleRoll += pSensorData->gyro.Y; // add each value found (integral) to final value to find y axis

		/// sin functions to prevent the axis shift between x and y axis,
		/// by using z axis data as a reference

		pSensorData->gyroAxisShiftRadianCoefficient = (FIXED_CALCULATION_TIME_IN_SECONDS / pSensorData->gyro_co) / (1/(M_PI/180));

		pSensorData->gyroAnglePitch += pSensorData->gyroAngleRoll * sin((pSensorData->gyroRawCalibrated.Z)*(pSensorData->gyroAxisShiftRadianCoefficient));
		pSensorData->gyroAngleRoll -= pSensorData->gyroAnglePitch * sin((pSensorData->gyroRawCalibrated.Z)*(pSensorData->gyroAxisShiftRadianCoefficient));


		/// ACC's values
		pSensorData->accTotalVector = sqrt(((pSensorData->accRawCalibrated.X)*(pSensorData->accRawCalibrated.X)) + ((pSensorData->accRawCalibrated.Y)*(pSensorData->accRawCalibrated.Y)) + ((pSensorData->accRawCalibrated.Z)*(pSensorData->accRawCalibrated.Z)));

		// convert these into degrees from radian
		pSensorData->accAnglePitch = asin((double) (pSensorData->accRawCalibrated.Y)/pSensorData->accTotalVector) * (1/(M_PI/180));
		pSensorData->accAngleRoll = asin((double) (pSensorData->accRawCalibrated.X)/pSensorData->accTotalVector) * -(1/(M_PI/180));


		// For belov values, compare the values with mobile phone's accelerometer and fine tune the values with these offsets defined at MPU6050.h
		pSensorData->accAnglePitch = (pSensorData->accAnglePitch) - (ACC_OFFSET_EXPERIMENTAL);
		pSensorData->accAngleRoll = (pSensorData->accAngleRoll) - (ACC_OFFSET_EXPERIMENTAL);

		/// Implementing Low-Pass and High-Pass filters

		// Because of gyroscopes may not be collecting new data immediately at startup while accelerometer can, we can skip the values for gyro for once and take only the acc.'s values.
		// In the next loop, both values will be taken.

		if (setGyro == 0) {

			pSensorData->final_AnglePitch = pSensorData->accAnglePitch; // use only acc's pitch value for now
			setGyro = 1;


		} else if (setGyro == 1) { // the low pass filter

			pSensorData->final_AnglePitch = ((pSensorData->gyroAnglePitch)*0.9996) + ((pSensorData->accAnglePitch)*0.0004);
			pSensorData->final_AngleRoll = ((pSensorData->gyroAngleRoll)*0.9996) + ((pSensorData->accAngleRoll)*0.0004);

		}

		// to fix and ensure the calculating period to 4 ms


		while((HAL_GetTick() - time_start_2) < FIXED_CALCULATION_TIME_IN_MILLISECONDS);
		time_elapsed_in_one_loop_2 = (HAL_GetTick() - time_start_2);

	}

	return Final_Read_Status;

}

sensor_status_e MPU6050_find_the_initial_calibration_offsets(SensorData_t *pSensorData) {

	sensor_status_e Calibration_Read_Status;

	// if all the data wanted to be read at once, for the sake of sampling at the same time,
	// 14 bytes of data will be needed to read. which is starting from register ACCEL_XOUT_H(59) to GYRO_ZOUT_L (72, including this)

	uint8_t data_buffer[14];

	static uint16_t time_start_1 = 0;
	static uint8_t time_elapsed_in_one_loop_1 = 0;

	for (uint16_t i=0; i<=NUM_OF_CALC_REPEATS_FOR_CAL; i++) {

		time_start_1 = HAL_GetTick();

		Calibration_Read_Status = sensor_read_bytes(MPU6050_I2C_ADDRESS_AD0_SHIFTED, MPU_REG_ACCEL_XOUT_H, data_buffer, 14);

		if (Calibration_Read_Status != SENSOR_ERROR) {

			pSensorData->accRaw.X = (int16_t) ((data_buffer[0] << 8) | (data_buffer[1]));
			pSensorData->accRaw.Y = (int16_t) ((data_buffer[2] << 8) | (data_buffer[3]));
			pSensorData->accRaw.Z = (int16_t) ((data_buffer[4] << 8) | (data_buffer[5]));

			pSensorData->tempRaw = (int16_t) ((data_buffer[6] << 8) | (data_buffer[7]));

			pSensorData->gyroRaw.X = (int16_t) ((data_buffer[8] << 8) | (data_buffer[9]));
			pSensorData->gyroRaw.Y = (int16_t) ((data_buffer[10] << 8) | (data_buffer[11]));
			pSensorData->gyroRaw.Z = (int16_t) ((data_buffer[12] << 8) | (data_buffer[13]));

			pSensorData->accCalOffset.X += pSensorData->accRaw.X;
			pSensorData->accCalOffset.Y += pSensorData->accRaw.Y;
			pSensorData->accCalOffset.Z += pSensorData->accRaw.Z;

			pSensorData->gyroCalOffset.X += pSensorData->gyroRaw.X;
			pSensorData->gyroCalOffset.Y += pSensorData->gyroRaw.Y;
			pSensorData->gyroCalOffset.Z += pSensorData->gyroRaw.Z;

			// to fix the calculating period to 4 ms
			while((HAL_GetTick() - time_start_1) < FIXED_CALCULATION_TIME_IN_MILLISECONDS);
			time_elapsed_in_one_loop_1 = (HAL_GetTick() - time_start_1);

		}

		else {
			return Calibration_Read_Status;
		}

	}

	if (Calibration_Read_Status != SENSOR_ERROR) {

		// Calibration process for acc may not be needed, check later!!
		pSensorData->accCalOffset.X /= NUM_OF_CALC_REPEATS_FOR_CAL;
		pSensorData->accCalOffset.Y /= NUM_OF_CALC_REPEATS_FOR_CAL;
		pSensorData->accCalOffset.Z /= NUM_OF_CALC_REPEATS_FOR_CAL;

		pSensorData->gyroCalOffset.X /= NUM_OF_CALC_REPEATS_FOR_CAL;
		pSensorData->gyroCalOffset.Y /= NUM_OF_CALC_REPEATS_FOR_CAL;
		pSensorData->gyroCalOffset.Z /= NUM_OF_CALC_REPEATS_FOR_CAL;
	}

	return Calibration_Read_Status;

}

