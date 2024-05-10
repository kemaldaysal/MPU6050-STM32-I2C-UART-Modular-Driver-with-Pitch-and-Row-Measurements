/*
 * application_layer_drivers.h
 *
 *  Created on: May 4, 2024
 *      Author: Kemal
 */

#ifndef INC_APPLICATION_LAYER_DRIVERS_H_
#define INC_APPLICATION_LAYER_DRIVERS_H_

#include <stdio.h>

typedef enum{
	INIT_OK,
	INIT_ERROR
}main_init_status_e;

typedef enum{
	CRW_OK,
	CRW_ERROR
}connection_and_rw_test_status_e;

typedef enum{
	RW_OK,
	RW_ERROR

}read_write_status_e;



connection_and_rw_test_status_e applayer_sensor_test_connectivity(void);
main_init_status_e applayer_sensor_and_uart_init(void);
read_write_status_e applayer_sensor_read_acc_data(void);
read_write_status_e applayer_sensor_read_gyro_data(void);
read_write_status_e applayer_sensor_read_temp_data(void);
read_write_status_e applayer_sensor_read_all_data_at_once(void);
read_write_status_e applayer_sensor_find_initial_calibration_error_offsets(void);

#endif /* INC_APPLICATION_LAYER_DRIVERS_H_ */
