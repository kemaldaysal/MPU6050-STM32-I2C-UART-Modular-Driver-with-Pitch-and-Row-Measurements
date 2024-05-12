**General Information and References**
* In this project, STM32F070RB is used. When implementing this code to other projects with different MCUs, the pin configurations (especially in uart_driver.c and uart_driver.h) will need to be changed.
* The communication_layer and MPU_6050 header files and codes are generated with the reference of MPU6050's datasheet and register map documents, which can be reachable in TDK-Invensense's webpage.   
* To improve the complimentary filter, calculation methods and hence the sensor's outputs, Joop Brokking's MPU6050 sensor's math guides and Euler Angles video references can be used.  

**Things to do before trying the project codes**
1) I2C (communication layer), UART and MPU6050's drivers header files and source codes are located inside of /Drivers/Hardware_Drivers/Inc and /Src files. Because STM32CubeIDE doesn't have default paths to these folders, user need to add these folders to to project's path preferences, in order to resolve the "header files and source codes can't be found" error. To do so, user should right-click to the project, then Properties -> C/C++ Build -> Settings -> MCU GCC Compiler -> Include Paths, then add the /Hardware_Drivers/Inc folder to the path.
2) In order to use UART's TX and console write functions without errors and to display the float variables succesfully on console, user should enable the "Use float with printf from newlib-nano (-u _print_float)" setting. To do so, user should right-click to the project, then Properties -> C/C++ Build -> Settings -> MCU Settings and then enable the tickbox for "Use float with printf from newlib-nano (-u _print_float)".

**Known Bugs**
1) As similar to some other IMU sensors, the sensor's pitch and row outputs can increase and decrease overtime when the sensor is stationary. And this also causes an reference point shifting. Using complimentary integral filters didn't fix it, a Kalman based filter may be needed to stabilize the outputs. 
2) Adding HAL delays to the main.c function, especially inside of while loop, causes incorrect calibrations, calculations and data readings. Which is probably because of the delays interrupting 4ms limited sensor reading time.
3) Temperature sensor may display +2 *C higher values when the formula on MPU6050's datasheet is used to calculate the temperature from it's raw value. User may need to change it's offset and find another setting rather than fixed offset (specified in the datasheet) used in the code.
