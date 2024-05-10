#include "mpu6050.h"
#include "uart_driver.h"
#include "application_layer_drivers.h"

static SensorData_t sensorData;

connection_and_rw_test_status_e applayer_sensor_test_connectivity(void) {

	char buffer[200];
	sprintf((char*)buffer, "Starting connection and r/w test.\r\n");
	UART_send_byte_array(buffer, strlen((char*)buffer));

	sensor_status_e connectivity_status = MPU6050_test_sensor_by_checking_ID_on_register() ;

	if (connectivity_status == SENSOR_OK) {

		sprintf((char*)buffer, "CRW test is successful.\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return CRW_OK;

	} else {

		sprintf((char*)buffer, "CRW test failed, check connections, chip and register addresses !!\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return CRW_ERROR;

	}

}


main_init_status_e applayer_sensor_and_uart_init(void)
{

	char buffer[200];
	sprintf((char*)buffer, "\nStarting to initialization of MPU6050 and UART.\r\n");
	UART_send_byte_array(buffer, strlen((char*)buffer));


	// MPU6050 INIT OF GYRO, ACC AND TEMP SENSOR SETTINGS AND DISABLING THE SLEEP MODE

	sensor_status_e mpu6050_init_status = MPU6050_initialize(&sensorData, FS_1000, AFS_4G, TEMPSENSOR_ON);

	if (mpu6050_init_status == SENSOR_OK) {

		sprintf((char*)buffer, "MPU6050 initialization successful.\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));

	} else {

		sprintf((char*)buffer, "MPU6050 initialization failed, failed to read from/write to sensor registers, please debug to find the exact error location.\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));

	}

	// UART INIT

	HAL_StatusTypeDef UART_init_status = UART_init(115200);

	if (UART_init_status == HAL_OK) {

		sprintf((char*)buffer, "UART initialization successful.\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));


	} else {

		sprintf((char*)buffer, "UART initialization failed.\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));

	}

	// RESULT FEEDBACK

	if ((mpu6050_init_status == SENSOR_OK) && (UART_init_status == HAL_OK)) {

		sprintf((char*)buffer, "All initializations are successful.\r\n\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return INIT_OK;
	} else {

		sprintf((char*)buffer, "Main initializations failed.\r\n\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return INIT_ERROR;
	}

}


read_write_status_e applayer_sensor_read_acc_data(void)
{

	sensor_status_e acc_data_rw_status;
	char buffer[100];

	acc_data_rw_status = MPU6050_read_all_sensor_data_at_the_same_time_basic(&sensorData);

	if (acc_data_rw_status == SENSOR_OK) {

		sprintf((char*)buffer, "Acc X:%.2f Y:%.2f Z:%.2f\r\n",sensorData.acc.X, sensorData.acc.Y, sensorData.acc.Z);
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_OK;

	} else {

		sprintf((char*)buffer, "Read / write operations from sensor's data registers failed!! \r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_ERROR;
	}
	// to do

}

read_write_status_e applayer_sensor_read_gyro_data(void)
{

	sensor_status_e gyro_data_rw_status;

	char buffer[100];

	gyro_data_rw_status = MPU6050_read_all_sensor_data_at_the_same_time_basic(&sensorData);

	if (gyro_data_rw_status == SENSOR_OK) {

		sprintf((char*)buffer, "Gyro X:%.2f Y:%.2f Z:%.2f\r\n",sensorData.gyro.X, sensorData.gyro.Y, sensorData.gyro.Z);
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_OK;

	} else {

		sprintf((char*)buffer, "Read / write operations from sensor's data registers failed!! \r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_ERROR;
	}
}

read_write_status_e applayer_sensor_read_temp_data(void) {

	sensor_status_e temp_data_rw_status;

	char buffer[100];
	temp_data_rw_status = MPU6050_read_all_sensor_data_at_the_same_time_basic(&sensorData);

	if (temp_data_rw_status == SENSOR_OK) {

		sprintf((char*)buffer, "Temp: %.2f *C\r\n",sensorData.temp);
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_OK;

	} else {

		sprintf((char*)buffer, "Read / write operations from sensor's data registers failed!! \r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_ERROR;
	}
}

read_write_status_e applayer_sensor_read_all_data_at_once(void) {

	sensor_status_e all_data_rw_status;

	char buffer[100];
	all_data_rw_status = MPU6050_read_all_sensor_data_at_the_same_time_calibrated(&sensorData);

	if (all_data_rw_status == SENSOR_OK) {

		sprintf((char*)buffer, "Pitch Angle: %.2f | Roll Angle: %.2f | Temp: %.2f *C\r\n", sensorData.final_AnglePitch, sensorData.final_AngleRoll, sensorData.temp);
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_OK;

	} else {

		sprintf((char*)buffer, "Read / write operations from sensor's data registers failed!! \r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_ERROR;
	}



}

read_write_status_e applayer_sensor_find_initial_calibration_error_offsets(void) {

	sensor_status_e calibration_read_status;

	char buffer[150];
	sprintf((char*)buffer, "Starting to calculate gyro calibration offsets in approx 10 seconds.\r\n");
	UART_send_byte_array(buffer, strlen((char*)buffer));

	calibration_read_status = MPU6050_find_the_initial_calibration_offsets(&sensorData);//	sprintf((char*)buffer, "Acc X:%.2f Y:%.2f Z:%.2f | Gyro X:%.2f Y:%.2f Z:%.2f | Temp: %.2f *C\r\n",sensorData.acc.X, sensorData.acc.Y, sensorData.acc.Z, sensorData.gyro.X, sensorData.gyro.Y, sensorData.gyro.Z, sensorData.temp);

	if (calibration_read_status == SENSOR_OK) {

		sprintf((char*)buffer, "Calibration is successful.\r\n\nStarting to read sensor data.\r\n\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_OK;

	} else {

		sprintf((char*)buffer, "Calibration failed, error when reading the sensor datas from the sensor, check connections and timings !!\r\n");
		UART_send_byte_array(buffer, strlen((char*)buffer));
		return RW_ERROR;
	}

}

