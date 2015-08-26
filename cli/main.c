/* Author: Callum Nunes-Vaz
 *
 * Non-Standard (for C) Software Dependencies: 
 *	- C library for Broadcom BCM 2835 as used in Raspberry Pi [V1.42] {http://www.airspayce.com/mikem/bcm2835/}
 *
 * IMPORTANT NOTES:
 * To compile using gcc run  "gcc -o piCopter.prog piCopter.c -l bcm2835" in target dir (make sure bcm2835 libs installed)
 * MUST BE RUN AS SUPER USER TO ACCESS GPIO e.g. "sudo ./piCopterprogram.run"
 * Debugging is enabled at compile time by setting debug macro as defined e.g. "gcc -o piCopter.prog piCopter.c -DDEBUG -l bcm2835"
 */

//-------------------------------------------------------Libraries-------------------------------------------------------//
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <bcm2835.h>   // bcm2835 library to use chip pins and comms interfaces, needs to be installed (see top of document)
#include "../libs/mpu9150.h"
#include "../libs/mpl3115a2.h"
#include "../libs/ak8975.h"
#include "../libs/easySerialComms.h"

//--------------------------------------------------- Global Variables -------------------------------------------------//


//----------------------------------------------- Global Program Modifiers ----------------------------------------------//

const int LOOP_CNT_DIV = 256; // (DEBUGGING) set how many times the main loop will iterate before printing, reccomended 2^power value for consistent results

//------------------------------------------------------Main Routine-----------------------------------------------------//

int main (int argc, char *argv[]) {
	uint32_t loop_Counter; 
	init(); 			  // run all initialisation code for Pi and external devices
	if (!bcm2835_init()) return 1;    // if BCM not initialised then exit program
	while (1) {
		// CONTROLLER INPUT
		//readController();       // get data from user's controller over xbee

		// MPU9150
		//if (bcm2835_gpio_eds(imuIntrptPin)) { 	// poll for past edge detection
			readIMU();
			readMag(); //NACK
		//	bcm2835_gpio_set_eds(imuIntrptPin); // clear pins GPIO flag
		//}

		/*		
		// MPL3115A2
		if (bcm2835_gpio_eds(altiIntrptPin1)) { // check for past edge detection
			readAlti(); // Check altimeter interrupts, if high access and store raw altimeter data
			bcm2835_gpio_set_eds(imuIntrptPin); // clear pins GPIO flag
		} */

		// DEBUGGING
		loop_Counter++;
		if (loop_Counter%LOOP_CNT_DIV) {
			printf("\f");	// clear screen using form-feed for printRawData() and printProcessedData()
			printRawData(); 	// print out all raw data from i2c sensors
		}

		// KALMAN Filter
		//runKalman(); 		// Smooth sensor input data (produce 'processed' sensor data) using quaternion based kalman filter

		// PID Filter
		//runPID(); 		// ...smooth PWM ouputs with respect to time through PID control
		
		// SEND PWM Values
		//writePWM();  		// finally send PWM values to PIC

		// Accel data to [G]'s, Gyro to [degrees/s], Temp to [Celcius], Mag to [uT], Alti to [meters]
		//formatData();

		// DEBUGGING
		if (loop_Counter%LOOP_CNT_DIV) 
			printProcessedData(); // print out all processed data from i2c sensors
	}
	bcm2835_close();
	return 0;
}


//-------------------------------------------------------Subroutines-----------------------------------------------------//
// main initialisation routine, called from main()
void init(void) {
	printf("%s\n", "Entering initialisation..."); 

	printf("%s\n", "	Initialising BCM2835"); 
	bcm2835_init(); // inistialise gpio (mainly comms)

	printf("%s\n", "	Initialising Comms");  
	init_Comms(); // initialise commuications interfaces

	printf("%s\n", "	Initialising GPIO pins"); 
	init_PinDir(); // initialise GPIO pin directions and state-formats using bcm2835 function libraries

	printf("%s\n", "	Initialising Sensors"); 
	init_Sensors(); // initialise i2c altitude and imu sensors

	printf("%s\n%s", "Initialisation Complete.", "Entering Main Loop..."); 
}

// initialise GPIO pin directions and state-formats using bcm2835 function libraries
// SOME OF THESE BCM2835 FUNCTIONS CAUSE THE CPU TO HANG :((((((((((((((((((((((
void init_PinDir(void) {
	// imu interrupt
	//bcm2835_gpio_fsel(imuIntrptPin, BCM2835_GPIO_FSEL_INPT); // input
	//bcm2835_gpio_set_pud(imuIntrptPin, BCM2835_GPIO_PUD_DOWN);  // pull-down
	//bcm2835_gpio_hen(imuIntrptPin); // enable high interrupt enable
	// altimeter interrupts
	//bcm2835_gpio_fsel(altiIntrptPin1, BCM2835_GPIO_FSEL_INPT); // input
	//bcm2835_gpio_set_pud(altiIntrptPin1, BCM2835_GPIO_PUD_DOWN);  // pull-down
	//bcm2835_gpio_hen(altiIntrptPin1); // enable high interrupt enable

	// ALTI INTERRUPT 2 AND GPS NAV LOCK CURRENTLY UNUSED
	//bcm2835_gpio_fsel(altiIntrptPin2, BCM2835_GPIO_FSEL_INPT); // input
	//bcm2835_gpio_set_pud(altiIntrptPin2, BCM2835_GPIO_PUD_DOWN);  // pull-down
	//bcm2835_gpio_hen(altiIntrptPin2); // enable high interrupt enable
	// gps nav lock indicator
	//bcm2835_gpio_fsel(gpsNavLock, BCM2835_GPIO_FSEL_INPT); // input
	//bcm2835_gpio_set_pud(gpsNavLock, BCM2835_GPIO_PUD_DOWN);  // pull-down
	//bcm2835_gpio_hen(gpsNavLock); // enable high interrupt enable
}

// initialise i2c altitude and imu sensors
void init_Sensors(void) {
	init_IMU();
	init_MAG();
	init_ALTI();
} 

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
	printf("%s", "Accelorometer:  ");
	printf("X: %06.4f [G],   ", get_AccelX());
	printf("Y: %06.4f [G],   ", get_AccelY());
	printf("Z: %06.4f [G]\n", get_AccelZ());
	printf("%s", "Gyroscope:      ");
	printf("X: %03.0f [%c/s], ", get_GyroX(), 176);
	printf("Y: %03.0f [%c/s], ", get_GyroY(), 176);
	printf("Z: %03.0f [%c/s]\n", get_GyroZ(), 176);
	printf("%s", "Magnetomter:    ");
	printf("Z: %04d [%cT],  ", get_MagX(), 181);
	printf("Z: %04d [%cT],  ", get_MagY(), 181);
	printf("Z: %04d [%cT]\n", get_MagZ(), 181);
	printf("IMU/Mag Temp.:     %06.1f [%cC]\n", get_Temp(), 176);
	printf("Altitude:    %06.1f [m]\n\n", get_Altitude());
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
	printf("X: %05d , ", get_AccelX_Raw());
	printf("Y: %05d , ", get_AccelY_Raw());
	printf("Z: %05d \n", get_AccelZ_Raw());
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











