/*
 * mpu6050.h
 *
 *  Created on: Apr 30, 2024
 *      Author: Kemal
 */

#ifndef HARDWARE_DRIVERS_INC_MPU6050_H_
#define HARDWARE_DRIVERS_INC_MPU6050_H_

#include <communication_driver.h>
#include "stm32f0xx_hal.h"
#include "stdint.h"
#include "math.h"

#define MPU6050_I2C_ADDRESS_AD0_SHIFTED ( 0x68<<1 )
#define MPU6050_I2C_ADDRESS_AD1_SHIFTED ( 0x69<<1 )
#define MPU6050_I2C_ADDRESS_AD0_NOT_SHIFTED ( 0x68 )
#define MPU6050_I2C_ADDRESS_AD1_NOT_SHIFTED ( 0x69 )
#define MPU_REG_SELF_TEST_X	( 13 )
#define MPU_REG_SELF_TEST_Y	( 14 )
#define MPU_REG_SELF_TEST_Z	( 15 )
#define MPU_REG_SELF_TEST_A	( 16 )
#define MPU_REG_SMPLRT_DIV	( 25 )
#define MPU_REG_CONFIG	( 26 )
#define MPU_REG_GYRO_CONFIG	( 27 )
#define MPU_REG_ACCEL_CONFIG	( 28 )
#define MPU_REG_ACCEL_XOUT_H	( 59 )
#define MPU_REG_ACCEL_XOUT_L	( 60 )
#define MPU_REG_ACCEL_YOUT_H	( 61 )
#define MPU_REG_ACCEL_YOUT_L	( 62 )
#define MPU_REG_ACCEL_ZOUT_H	( 63 )
#define MPU_REG_ACCEL_ZOUT_L	( 64 )
#define MPU_REG_ACCEL_TEMP_OUT_H ( 65 )
#define MPU_REG_ACCEL_TEMP_OUT_L	( 66 )
#define MPU_REG_GYRO_XOUT_H	( 67 )
#define MPU_REG_GYRO_XOUT_L	( 68 )
#define MPU_REG_GYRO_YOUT_H	( 69 )
#define MPU_REG_GYRO_YOUT_L	( 70 )
#define MPU_REG_GYRO_ZOUT_H	( 71 )
#define MPU_REG_GYRO_ZOUT_L	( 72 )
#define MPU_REG_USER_CTRL	( 106)
#define MPU_REG_PWR_MGMT_1	( 107 )
#define MPU_REG_PWR_MGMT_2	( 108 )
#define MPU_REG_FIFO_COUNTH ( 114 )
#define MPU_REG_FIFO_COUNTL ( 115 )
#define MPU_REG_FIFO_R_W	( 116 )
#define MPU_REG_WHO_AM_I	( 0x75 )

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

#define MPU_BIT_PWR_MGMT_1_SLEEP_MODE ((uint8_t) 6 )
#define MPU_REG_GYRO_CONFIG_GYRO_RANGE_BITS_POSITION ((uint8_t) 3 )
#define MPU_REG_ACC_CONFIG_ACC_RANGE_BITS_POSITION ((uint8_t) 3 )
#define MPU_REG_TEMP_CONFIG_TEMP_ON_OFF_BITS_POSITION ((uint8_t) 3 )

#define TEMP_CO (340.0f)
#define TEMP_OFFSET (36.53f)

#define NUM_OF_CALC_REPEATS_FOR_CAL ((uint16_t) 2000)
#define FIXED_CALCULATION_TIME_IN_SECONDS (0.004f)
#define FIXED_CALCULATION_TIME_IN_MILLISECONDS ((uint8_t) 4)
#define FIXED_CALCULATION_TIME_IN_MICROSECONDS ((uint16_t) 4000)


#define ACC_OFFSET_EXPERIMENTAL (0.00f)

// Gyro's measurement frequency options (obtained from datasheet)
typedef enum {
	FS_250, // 0 (250 degrees / seconds)
	FS_500, // 1
	FS_1000, // 2
	FS_2000, // 3
}fs_sel_e;

// Accelerometer's measurement range // datasheet 1 / page 13, 6.2
typedef enum {
	AFS_2G, // +-2G, 2G MIN, 2G MAX
	AFS_4G, // 1
	AFS_8G, // 2
	AFS_16G, // 3
}afs_sel_e;

typedef struct {
	int16_t X;
	int16_t Y;
	int16_t Z;
}AxisRawVal_t;

typedef struct {
	double X;
	double Y;
	double Z;
}AxisValCal_t;

typedef struct {
	double X;
	double Y;
	double Z;
}AxisVal_t;


typedef struct {
	AxisRawVal_t accRaw;
	AxisRawVal_t gyroRaw;
	AxisRawVal_t accRawCalibrated;
	AxisRawVal_t gyroRawCalibrated;
	AxisValCal_t accCalOffset;
	AxisValCal_t gyroCalOffset;
	AxisVal_t acc;
	AxisVal_t gyro;
	double gyroAnglePitch;
	double gyroAngleRoll;
	double accAnglePitch;
	double accAngleRoll;
	double final_AnglePitch;
	double final_AngleRoll;
	uint16_t acc_co;
	float gyro_co;
	double gyroAxisShiftRadianCoefficient;
	int16_t accTotalVector;
	int16_t tempRaw;
	float temp;

}SensorData_t;

typedef enum {
	SLEEPMODE_OFF,
	SLEEPMODE_ON
}sleepmode_e;

typedef enum {
	TEMPSENSOR_ON,
	TEMPSENSOR_OFF
}tempsensormode_e;

sensor_status_e MPU6050_test_sensor_with_HAL(void);
sensor_status_e MPU6050_test_sensor_by_checking_ID_on_register(void);
sensor_status_e MPU6050_set_gyro_range(SensorData_t *pSensor, fs_sel_e gyroRange);
sensor_status_e MPU6050_set_acc_range(SensorData_t *pSensor, afs_sel_e accRange);
sensor_status_e MPU6050_enable_temp_sensor(tempsensormode_e tempSensorCommand);
sensor_status_e MPU6050_set_sleep_mode(sleepmode_e sleepmode);
sensor_status_e MPU6050_initialize(SensorData_t *pSensor, fs_sel_e gyro_config, afs_sel_e acc_config, tempsensormode_e tempSensorMode);
sensor_status_e MPU6050_read_all_sensor_data_at_the_same_time_basic(SensorData_t *pSensorData);
sensor_status_e MPU6050_find_the_initial_calibration_offsets(SensorData_t *pSensorData);
sensor_status_e MPU6050_read_all_sensor_data_at_the_same_time_calibrated(SensorData_t *pSensorData);


// Don't use statics here because these function must be reached from outside. If we use static, it prevents this availability.

#endif /* HARDWARE_DRIVERS_INC_MPU6050_H_ */
