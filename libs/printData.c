//-------------------------------------------------------Libraries-------------------------------------------------------//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mpl3115a2.h"
#include "mpu9150.h"
#include "ak8975.h"

//-------------------------------------------------------Subroutines-----------------------------------------------------//

// print out Processed sensor data in 6-decimal format, used for analysis/debugging
void printProcessedData(void) {
	/*---------------------------------------------------------//
		PROCESSED SENSOR DATA:
		Accelorometer:  X: ......[G],   Y: ......[G],   Z: ......[G]
		Gyroscope:      X: ......[°/s], Y: ......[°/s], Z: ......[°/s]
		Magnetometer:   X: ......[µT],  Y: ......[µT],  Z: ......[µT]
		Temperature:	......[°C]
		Altitude:       ......[m]
	//---------------------------------------------------------*/
	printf("%s\n", "PROCESSED SENSOR DATA:");
	printf("%s", "Accelorometer: [G]   ");
	printf("X: %06.4f,", get_AccelX());
	printf("Y: %06.4f,", get_AccelY());
	printf("Z: %06.4f\n", get_AccelZ());
	printf("%s", "Gyroscope: [deg/s]   ");
	printf("X: %03.0f, ", get_GyroX());
	printf("Y: %03.0f, ", get_GyroY());
	printf("Z: %03.0f\n", get_GyroZ());
	printf("%s", "Magnetomter: [uT]    ");
	printf("X: %04f,  ", get_MagX());
	printf("Y: %04f,  ", get_MagY());
	printf("Z: %04f\n", get_MagZ());
	printf("IMU/Mag Temp.:       %3.1f [Celcuis]\n", get_Temp());
	printf("Altitude:    %6.1f [meters]\n\n", get_Altitude());
}

// print out raw sensor data in 6-decimal format, used for analysis/debugging
void printRawData(void) {
	/*---------------------------------------------------------//
		RAW SENSOR DATA:
		Accelorometer:  X: ......, Y: ......, Z: ......
		Gyroscope:      X: ......, Y: ......, Z: ......
		Magnetometer:   X: ......, Y: ......, Z: ......
		Temperature:	......
		Altitude:       ......
	//---------------------------------------------------------*/
	printf("%s\n", "RAW SENSOR DATA:");
	printf("%s", "Accelorometer:  ");
	printf("X: %05d, ", get_AccelX_Raw());
	printf("Y: %05d, ", get_AccelY_Raw());
	printf("Z: %05d\n", get_AccelZ_Raw());
	printf("%s", "Gyroscope:      ");
	printf("X: %05d, ", get_GyroX_Raw());
	printf("Y: %05d, ", get_GyroY_Raw());
	printf("Z: %05d\n", get_GyroZ_Raw());
	printf("%s", "Magnetomter:    ");
	printf("X: %05d, ", get_MagX_Raw());
	printf("Y: %05d, ", get_MagY_Raw());
	printf("Z: %05d\n", get_MagZ_Raw());
	printf("%s", "Temperature:    ");
	printf("%06d\n", get_Temp_Raw());
	printf("%s", "Altitude:       ");
	printf("%08d\n\n", get_Altitude_Raw());
}

