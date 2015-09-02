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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <bcm2835.h>   // bcm2835 library to use chip pins and comms interfaces, needs to be installed (see top of document)
#include "main.h"
#include "../libs/mpu9150.h"
#include "../libs/mpl3115a2.h"
#include "../libs/ak8975.h"
#include "../libs/easySerialComms.h"
#include "../libs/printData.h"

//--------------------------------------------------- Global Variables -------------------------------------------------//


//----------------------------------------------- Global Program Modifiers ----------------------------------------------//

const int LOOP_CNT_DIV = 64; // (DEBUGGING) set how many times the main loop will iterate before printing, reccomended 2^power value for consistent results

//------------------------------------------------------Main Routine-----------------------------------------------------//

int main (int argc, char *argv[]) {
	uint32_t loop_Counter;
	uint16_t g;
	byte i, a, d;

	// Check if the program has been run with superuser permissions
	if (getuid()) {
		printf("%s\n", "Please run program with superuser permissions!\nThe program will now exit"); exit(0);}

	// check if enough arguments have been supplied for configuration
	if (argc != 7) {
		printf("%s\n", "Incorrect Number of Configuration Parameters!\nThe program will now exit"); exit(0); }

	// apply actions of arguments (acceloromter range, gyroscope range and imu digital low pass filter setting)
	printf("%s %d %s\n", "Applying", (argc-1)/2, "Launch Parameters:");
	for (i=1; i<argc; i++) {
		if (!strcmp("-a", argv[i])) a = (byte)atoi(argv[++i]);
		if (!strcmp("-g", argv[i])) g = (uint16_t)atoi(argv[++i]);
		if (!strcmp("-d", argv[i])) d = (byte)atoi(argv[++i]);
	}

	init(a,g,d);   // run all initialisation code for Program and external devices

	// MAIN LOOP
	while (1) {
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
		if (!(loop_Counter%LOOP_CNT_DIV)) {
			system("clear");	// clear screen using form-feed for printRawData() and printProcessedData()
			printRawData(); 	// print out all raw data from i2c sensors
		}
		if (!(loop_Counter%LOOP_CNT_DIV)) 
			printProcessedData(); // print out all processed data from i2c sensors
	}
	bcm2835_close();
	return 0;
}


//-------------------------------------------------------Subroutines-----------------------------------------------------//
// main initialisation routine, called from main()
void init(byte a, uint16_t g, byte d) {
	printf("%s\n", "Entering Main initialisation..."); 

	printf("%s\n", "Initialising BCM2835");
	bcm2835_init(); // inistialise gpio 

	printf("%s\n", "	Initialising Comms");  
	init_Comms(); // initialise commuications interfaces

	printf("%s\n", "	Initialising GPIO pins"); 
	init_PinDir(); // initialise GPIO pin directions and state-formats using bcm2835 function libraries

	printf("%s\n", "	Initialising Sensors"); 
	init_Sensors(a, g, d); // initialise i2c altitude and imu sensors

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
void init_Sensors(byte a, uint16_t g, byte d) {
	init_IMU(a, g, d);
	init_MAG();
	init_ALTI();
} 













