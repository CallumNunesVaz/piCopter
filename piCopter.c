/*
 * Author: Callum Nunes-Vaz
 *
 * Date of last modification: 5th May 2015
 *
 * Current Problems: Currently investigating SPI comms (pic won't seem to talk back, most probably problem with PIC).
 *	   	     Magnetomer gives NACKS, must have incorrectly configured aux mode on mpu9150
 *	   	     Entire program contained in single .c file for dev purposes, will be reformatted for 1.0 version
 *		     Some BCM2835 functions cause CPU hangs, must find and test alternative method. see init_PinDir().
 *
 * Non-Standard (for C) Software Dependencies: 
 *	- C library for Broadcom BCM 2835 as used in Raspberry Pi [V1.42] {http://www.airspayce.com/mikem/bcm2835/}
 *
 * Description: This program drives a sensor board attached to the 40-pin GPIO in order to fly a quadcopter. Input is
 *		taken serially via an Xbee unit which is combined with data read from i2c sensors. The desired speed 
 *	        of the propellors is calculated and the data for the PWM controlling PIC is sent through SPI to a 
 *		PIC18F1828 which drives 4 speed controllers with 1kHz PWM. 
 * 		This program is intentionally overcomplicated in terms of devices and their communication interfaces, 
 * 		and is intended to be used as a learning tool.
 *
 * Communication scheme diagram:

                             -------------------
                             | Raspberry Pi A+ |
                             -------------------
                               |      |      |              --------------------                ----------------------
     -------------------i2c----'      |      '-------SPI----|    PIC16F1828    |------USART------| VENUS638FLPx (GPS) |
     |         |                    USART                   --------------------                ----------------------
     |   -------------                |                      |     |      |     |
     |	 | MPL3115A2 |                |  		   PWM    PWM    PWM    PWM			
     |	 -------------	     	  --------	            |      |      |      |
 -----------			  | XBEE |	        |ESC1|  |ESC2|  |ESC3|  |ESC4|
 | MPU9150 |                      --------
 -----------
 
 * IMPORTANT NOTES:
 * To compile using gcc run  "gcc -o piCopter.prog piCopter.c -l bcm2835" in target dir (make sure bcm2835 libs installed)
 * MUST BE RUN AS SUPER USER TO ACCESS GPIO e.g. "sudo ./piCopterprogram.run"
 * Debugging is enabled at compile time by setting debug macro as defined e.g. "gcc -o piCopter.prog piCopter.c -DDEBUG -l bcm2835"
 */

//-------------------------------------------------------Libraries-------------------------------------------------------//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "bcm2835.h"   // bcm2835 library to use chip pins and comms interfaces, needs to be installed (see top of document)
#include "piCopter_MAIN.h"  // header file directly related to this .c file




//--------------------------------------------------- Global Variables -------------------------------------------------//

// pwm values to go to pwm output
byte pwmOUT[4];

// sensor data storage
uint32_t altiRawData; // raw altitude data is 24 bits

float altiProcessedData;

unsigned int magRawData[6]; // magnetometer x y z

int magProcessedData[3]; // magnetometer x y z

unsigned int imuRawData[7]; // accel x y z, temp, gyro x y z

float imuProcessedData[7]; // accel x y z, temp, gyro x y z



//----------------------------------------------- Global Program Modifiers ----------------------------------------------//

ACCEL_RANGE = 8; // 2g's, 4g's 8g's or 16g's

GYRO_RANGE = 1000; // 250 degrees/s, 500 degrees/s, 1000 degrees/s, or 2000 degrees/s

DLPF_FREQ = 0; // choose value from 0 to 6 as value for imu low pass filter, 0=~256Hz, 1=~185Hz, 2=~95Hz, 3=~44Hz, 4=~22Hz, 5=~10Hz, 6=~5Hz

LOOP_CNT_DIV = 256; // (DEBUGGING) set how many times the main loop will iterate before printing, reccomended 2^power value for consistent results

//------------------------------------------------------Main Routine-----------------------------------------------------//

int main (int argc, char *argv[]) {
	uint32_t loop_Counter; 
	init(); 			  // run all initialisation code for Pi and external devices
	if (!bcm2835_init()) return 1;    // if BCM not initialised then exit program
	while (1) {
		// CONTROLLER INPUT
		readController();       // get data from user's controller over xbee

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
			system("clear");	// clear screen for printRawData() and printProcessedData()
			printRawData(); 	// print out all raw data from i2c sensors
		}

		// KALMAN Filter
		runKalman(); 		// Smooth sensor input data (produce 'processed' sensor data) using quaternion based kalman filter

		// PID Filter
		runPID(); 		// ...smooth PWM ouputs with respect to time through PID control
		
		// SEND PWM Values
		writePWM();  		// finally send PWM values to PIC

		// Accel data to [G]'s, Gyro to [degrees/s], Temp to [Celcius], Mag to [uT], Alti to [meters]
		formatData();

		// DEBUGGING
		if (loop_Counter%LOOP_CNT_DIV) 
			printProcessedData(); // print out all processed data from i2c sensors
	}
	bcm2835_close();
	return 0;
}


//-------------------------------------------------------Subroutines-----------------------------------------------------//
// functions in alphabetical order

// Format data from raw sensor values into SI units
void formatData() {
	byte i;

	// -------------- Accelorometer --------------- //
	for (i = 0; i < 3; i++) {
		if (imuRawData[i] >= 32767) { // if negative (because is unsigned int)
			imuProcessedData[i] = 65536 - imuRawData[i];  // invert
			imuProcessedData[i] *= -1;  // make float negative
		} else  // if positive
			imuProcessedData[i] = imuRawData[i];
		imuProcessedData[i] /= (32768/ACCEL_RANGE); // format to G's (9.8m/s)
	}


	// ----------------- Gyroscope ---------------- //
	for (i = 4; i < 7; i++) {
		if (imuRawData[i] >= 32767) { // if negative (because 2c unsigned int)
			imuProcessedData[i] = 65536 - imuRawData[i]; // invert
			imuProcessedData[i] *= -1; // make float negative
		} else // if positive
			imuProcessedData[i] = imuRawData[i];
		imuProcessedData[i] /= (32768/GYRO_RANGE); // format to degrees per second
	}


	// ----- IMU & MAG Die Temperature Sensor ----- //
	if (imuRawData[3] >= 32767) { // if negative (because is unsigned int and 2c)
		imuProcessedData[3] = 65536 - imuRawData[3]; // invert
		imuProcessedData[3] *= -1;  // make float negative
	} else  // if positive
		imuProcessedData[3] = imuRawData[3];
	imuProcessedData[3] = (imuProcessedData[3]/340) + 35; // format to Celcius (formula from datasheet)
	

	// -------------- Magnetometer --------------- //
	/* Magnetometer has internal adjustment using the raw xxyyzz data magRawData[0-5]
	   and the internal xyz adjustment values magRawData[6-8]. Use the following formula 
	   as specified by the AK8975 datasheet:

                            (ASA - 128)*0.5
		Hadj = H * (---------------  +  1 )
                                  128
	*/
	//for (i = 0; i < 3; i++) {
	//	magProcessedData[i] = (magRawData[i])*(((magRawData[i+3] - 128)/256) + 1);
	//}
	// Measuremnet data is stored in two’s complement and Little Endian format
	// four sign bits, then 12 value bits as increments of 0.3[uT]
	for (i = 0; i < 3; i++) {
		if (imuRawData[i] >= 61440) { // if negative (because is unsigned int and 2c)
			magProcessedData[i] = 65536 - (imuRawData[i] - 61440); // remove sign bits and invert
			magProcessedData[i] *= -1; // make negative
		} else  // if positive
			imuProcessedData[i] = imuRawData[i];
		imuProcessedData[i] = (int)(((float)imuProcessedData[i])*0.3); // format to uT
	}


	// --------------- Altimeter ---------------- //
	/* The Altitude data is arranged as 20-bit 2’s complement value in meters. Stored as meters 
	   with the 16 bits of OUT_P_DELTA_MSB and OUT_P_DELTA_CSB and with fractions of a meter 
	   stored in 4 bits in position 7-4 of OUT_P_DELTA_LSB. */
	if (altiRawData >= 524288) { // if negative (because is unsigned int and 2c)
		altiProcessedData = ((int)(1048576 - altiRawData)); // invert
		altiProcessedData *= -1;
	} else  // if positive
		altiProcessedData = altiRawData;
	altiProcessedData /= (16*65536); // format to meters (lower 4 bits were after decimal point)
}

// read a single value from one of the sensors registers (repeated-start format) used by mpu9150 and mpl3115a2
byte i2cRead_RS(byte slaveAddress, byte regAddress) {
	char reg[] = {regAddress};
	char buffer[] = {0};
	byte condition;
	bcm2835_i2c_setSlaveAddress(slaveAddress); // set Slave address
	condition = bcm2835_i2c_read_register_rs(reg, buffer, 1); //
	#ifdef DEBUG 
		printI2CReasonCode(condition); // if debugging enabled, print out Reason code 
	#endif
	return buffer[0];
}

// read a single value from one of the sensors registers (single-start format) used by ak8975
byte i2cRead_SS(byte slaveAddress, byte regAddress) {
	char buffer[2] = {slaveAddress, regAddress};
	byte condition;
	bcm2835_i2c_setSlaveAddress(slaveAddress); // set Slave address
	bcm2835_i2c_write(buffer, 2); // write slave address, then write register value
	condition = bcm2835_i2c_read(buffer, 1); // listen for data
	#ifdef DEBUG 
		printI2CReasonCode(condition); // if debugging enabled, print out Reason code 
	#endif
	return buffer[0];
}

// write a value to one of the i2c sensor's registers
void i2cWrite(byte slaveAddress, byte regAddress, byte val) {
	byte condition;                                     // returned i2c reason code storage
	char buffer[] = {regAddress, val}; 		    // buffer of bytes to send
	uint32_t num = 2; 				    // number of bytes in buffer / number of bytes to send
	bcm2835_i2c_setSlaveAddress(slaveAddress);          // set Slave address
	condition = bcm2835_i2c_write(buffer, num); 	    // Transfers any number of bytes to the currently selected I2C slave
	#ifdef DEBUG 
		printI2CReasonCode(condition); // if debugging enabled, print out Reason code  
	#endif
}

// main initialisation routine, called from main()
void init(void) {
	#ifdef DEBUG 
		printf("%s\n", "Entering initialisation..."); 
	#endif
	bcm2835_init(); // inistialise gpio (mainly comms) 
	init_Comms(); // initialise commuications interfaces
	init_PinDir(); // initialise GPIO pin directions and state-formats using bcm2835 function libraries
	init_Sensors(); // initialise i2c altitude and imu sensors
	init_DataStorage();	// set data to zero
	#ifdef DEBUG 
		printf("%s\n%s", "Initialisation Complete.", "Entering Main Loop..."); 
	#endif
}

// comms initialisation
void init_Comms(void) {
	#ifdef DEBUG 
		printf("%s\n", "	-Initialising Communications Interfaces..."); 
	#endif
	init_I2C();       	// establish i2c comms with sensors
	init_SPI();       	// establish SPI comms with PIC
	init_Serial();    	// establish Serial comms with xbee
}

// set data to zero
void init_DataStorage(void) {
	byte i;
	for (i = 0; i < 7; i++) // clear raw imu accelorometer, gyro and temp data
		imuRawData[i] = 0;
	for (i = 0; i < 6; i++) // clear raw magnetometer data
		magRawData[i] = 0;
	altiRawData = 0;	// clear raw altimeter data
}	

// I2C comms initialisation
void init_I2C(void) {
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising I2C..."); 
	#endif
	bcm2835_i2c_begin(); // enables i2c module
	bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626); // speed set to 400kHz ("fast mode")
}

// initialise GPIO pin directions and state-formats using bcm2835 function libraries
// SOME OF THESE BCM2835 FUNCTIONS CAUSE THE CPU TO HANG :((((((((((((((((((((((
void init_PinDir(void) {
	#ifdef DEBUG 
		printf("%s\n", "	-Initialising non-comms-GPIO pin direction and function... *Skipped in current version. See code"); 
	#endif
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
	byte i; // variable for for-loops
	
	//-----------------------------IMU CONFIGURATION----------------------------//
	#ifdef DEBUG 
		printf("%s\n%s\n", "	-Initialising Sensors...","		-Initialising IMU..."); 
	#endif

	// sample rate = (gyro output rate)/(1 + SMPRT_DIV)
	// gyro output rate =8KHz w/o DLPF and 1kHz with
	i2cWrite(imuI2CAddress, IMU_RA_SMPRT_DIV, 0); 

	// set value of digital low-pass filter (DLPF) for Accel and Gyro (samples out vibrations)
	// 0=~256Hz, 1=~185Hz, 2=~95Hz, 3=~44Hz, 4=~22Hz, 5=~10Hz, 6=~5Hz, 7=RESERVED
	// FSYNC not used, DLPF value held in 3 lowest bits so can have value written directly
	i2cWrite(imuI2CAddress, IMU_RA_IMU_CONFIG, DLPF_FREQ); 

	// set full scale range of gyroscope output (at bits <4:3>) 
	// 0 = 250 degrees/s, 1 = 500 degrees/s, 2 = 1000 degrees/s, 3 = 2000 degrees/s
	switch (GYRO_RANGE) {
		case 250: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 0); break;
		case 500: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 8); break;
		case 1000: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 16); break;
		case 2000: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 24); break;
		case default: printf("%s\n", "ERROR: invalid gyroscope range selected"); exit(1); break;
	}

	// set full scale range of accelorometer output (at bits <4:3>) 
	// 0 = 2g's, 1 = 4g's, 2 = 8g's, 3 =  16g's
	switch (ACCEL_RANGE) {
		case 2: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 0); break;
		case 4: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 8); break;
		case 8: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 16); break;
		case 16: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 24); break;
		case default: printf("%s\n", "ERROR: invalid accelerometer range selected"); exit(1); break;
	}
	// <7> 	temp H & L to fifo enable bit      <6> GyroX H & L to fifo enable bit 
	// <5>  GyroY H & L to fifo enable bit     <4> GyroZ H & L to fifo enable bit 
	// <3>  AccelXYZ H & L to fifo enable bit  <2> Slave 2 H & L to fifo enable bit 
        // <1>  Slave 1 H & L to fifo enable bit   <0> Slave 0 H & L to fifo enable bit 
	// Result = 0b00000000 = decimal 0
	i2cWrite(imuI2CAddress, IMU_RA_FIFO_EN, 0);

	// <7> int active high (0) or low (1),  		 <6> int pin as push-pull (0) or open drain (1),  
	// <5> int as 50us pulse (0) or held till cleared (1),   <4> flag cleared when INT_STATUS read (0) or any read (1),  
	// <3> FSYNC active high (0) or active low (1),  	 <2> FSYNC_INT enable bit, (0) for disable, 
        // <1> host can (1) or cant (0) access aux i2c bus       <0> UNIMPLEMENTED
	// Result = 0b00100010 = decimal 34
	i2cWrite(imuI2CAddress, IMU_RA_INT_PIN_CFG, 34);

	// <4> FIFO overflow enable,  	 <3> I2c Master interrupt source enable
        // <0> Data ready interrupt     
	// Result = 0b00000001 = decimal 1
	i2cWrite(imuI2CAddress, IMU_RA_INT_ENABLE, 1);

	// <7> enable (1) or disable (0) multi-master capability <6> wait for extrnal sensors before setting new data interrupt, (1) for enable
	// <5> write slave #3 data to FIFO? (0) for disable	 <4> enable stop-start between slave reads, (1) for enable
	// <3:0> I2C master clock speed [is set to 13 = 0b1101 = 400KHz]
	// Result = 0b00001101 = 13
	i2cWrite(imuI2CAddress, IMU_RA_I2C_MST_CTRL, 13);

	// <7> UNIMPLEMENTED						      <6> FIFO_EN, (1) enable or (0) disable
	// <5> I2C master mode (1), or tie aux and primary i2c lines (0)      <4> MUST BE ZERO!! (doesn't say why)
	// <3> UNIMPLEMENTED						      <2> reset FIFO buffer, bit resets itself after action
	// <1> resets the I C Master when set to 1 while I2C_MST_EN equals 0  <0> resets the signal paths for all sensors
	// Result = 0b00000001 = 1
	i2cWrite(imuI2CAddress, IMU_RA_USER_CTRL, 1);

	// <7> DEVICE RESET BIT						      <6> SLEEP ENABLE BIT
	// <5> toggles whether device will cycle between wake and sleep       <4> UNIMPLEMENTED
	// <3> Disables temp sensor (when set to 1)			           
	// <2:0> Clock Select: 0=internal 8MHz, 1=PLL w/ gyro X ref, 2=PLL w/ gyro Y ref, 3=PLL w/ gyro Z ref,  4=PLL w/ external 32kHz
	// 		       5= PLL w/ external 19.2MHz, 6 = RESV, 7 = stops clock
	// Result = 0b0000001 = 0
	i2cWrite(imuI2CAddress, IMU_RA_PWR_MGMT_1, 1);

	// <7:6> LP_WAKE_CTRL, 0=1.25Hz, 1=5Hz, 2=20Hz, 3=40Hz
	// <5> Accel X Standby mode enable bit      <4> Accel Y Standby mode enable bit 
	// <3> Accel Z Standby mode enable bit 	    <2> Gyro X Standby mode enable bit 
	// <1> Gyro Y Standby mode enable bit       <0> Gyro Z Standby mode enable bit s
	// Result = 0b11000000 = 196
	i2cWrite(imuI2CAddress, IMU_RA_PWR_MGMT_2, 0);

	// <7> Dealys shaddowing of external sensor data when set					      
	// <4> slave 4 accessed at decreased rate
	// <3> slave 3 accessed at decreased rate  <2> slave 2 accessed at decreased rate
	// <1> slave 1 accessed at decreased rate  <0> slave 0 accessed at decreased rate
	// Result = 0b00000000 = 0
	i2cWrite(imuI2CAddress, IMU_RA_I2C_MST_DELAY_CTRL, 0);

	// <2> resets gyroscope analogue and digital signal paths
	// <1> resets Accelaromter analogue and digital signal paths  <0> resets temperature analogue and digital signal paths
	// Result = 0b00000011 = 1
	i2cWrite(imuI2CAddress, IMU_RA_SIGNAL_PATH_RESET, 7); // RESET ALL

	// CLEAR ALL EXT_SENS_DATA registers and SLAVE ADDR, REG, CTRL, DO & DI registers (theyre not being used)
	// ...from address 0x25 to 0x35...
	for (i = 0x25; i <= 0x35; i++) i2cWrite(imuI2CAddress, i, 0);
	// ...and from address 0x49 to 0x66.
	for (i = 0x49; i <= 0x66; i++) i2cWrite(imuI2CAddress, i, 0);

	//------------------------------------MAG CONFIGURATION-----------------------------------//
	
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising Magnetometer..."); 
	#endif

	// OPERATING MODE CONTROL	
	// <3:0> 0000 = power down mode, 0001 = single measurement mode, 1000 = self test mode, 1111 = fuse ROM access mode 
	i2cWrite(magI2CAddress, MAG_RA_CNTL, 1);
	
	// SELF TEST CONTROL
	// <6> self test control, 0 = normal, 1 = generate magnetic field 
	i2cWrite(magI2CAddress, MAG_RA_ASTC, 0);

	//-----------------------------------ALT CONFIGURATION-----------------------------------//
	// currently configured to only use int1 interrupt pin, it is used for data-ready interrput 
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising Altimeter..."); 
	#endif
	
	// <7> Altimeter-barometer mode, 0 is barometer	 	      <6> Raw output mode toggle, only ADC info shown, 1 for Raw
	// <5:3> oversampling ratio (time between samples)	      
	// 000 = 6ms, 001 = 10ms, 010 = 18ms, 011 = 34ms, 100 = 66ms, 101 = 130ms, 110 = 258ms, 111 = 512ms
        // <2> Device reset bit		      
	// <1> intiates measurement (useful for one-shot in standby mode)     <0> active or standby mode, 1 is active
	// Result = 0b10000001 = decimal 129
	i2cWrite(altiI2CAddress, ALT_RA_CTRL_REG1, 129);
	
	// <5> load the target values for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH  
	// <4> selects the Target value for SRC_PW/SRC_TW and SRC_PTH/SRC_TTH 
	// <3:0> Auto acquisition time step. Default value 0. Step value is 2^(XXXX) Giving a range of 1 second to 2^15 seconds (9 hours)
	// Result = 0b00000000 = 0;
	i2cWrite(altiI2CAddress, ALT_RA_CTRL_REG2, 0);

	// <5>  Polarity of interrupt 1, 0 = active low  <4> INT 1 as internal pull-up or open drain, 0 = pullup
        // <1>  Polarity of interrupt 2, 0 = active low  <0> INT 2 as internal pull-up or open drain, 0 = pullup
	// Result = 0b00100010 = decimal 34
	i2cWrite(altiI2CAddress, ALT_RA_CTRL_REG3, 34); // active high, pull-up

	// <7> Data ready interrupt enable          <6> FIFO interrupt enable   
	// <5> pressure window interrupt enable     <4> temperature window interrupt enable   
	// <3> pressure threshold interrupt enable  <2> temperature threshold interrupt enable    
        // <1> pressure change interrupt enable     <0> temperature change interrupt enable    
	// Result = 0b10000000 = decimal 128
	i2cWrite(altiI2CAddress, ALT_RA_CTRL_REG4, 128); // PICK 2

	// <7> Data ready interrupt map to int1 or int2, (0 = int2)        <6> FIFO interrupt map to int1 or int2, (0 = int2) 
	// <5> pressure window interrupt map to int1 or int2, (0 = int2)   <4> temperature window interrupt map to int1 or int2, (0 = int2)    
	// <3> pressure threshold interrupt map to int1 or int2, (0 = int2)<2> temperature threshold interrupt map to int1 or int2, (0 = int2)    
        // <1> pressure change interrupt map to int1 or int2, (0 = int2)   <0> temperature change interrupt map to int1 or int2, (0 = int2)    
	// Result = 0b10000000 = decimal 128
	i2cWrite(altiI2CAddress, ALT_RA_CTRL_REG5, 128); // PICK 2 // data ready = int1, pressure change = int2

	// <2> Data ready event mode, set to 1 if event flag wanted for "change in state of data", 0 for "when system acquires new set of data"  
        // <1> Data event flag enable bit for pressure/altitude   <0> Data event flag enable bit for temperature
	// Result = 0b00000011 = decimal 3
	i2cWrite(altiI2CAddress, ALT_RA_PT_DATA_CFG, 3);
} 

// Serial comms initialisation
void init_Serial(void) {
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising USART..."); 
	#endif
}

// SPI comms initialisation
void init_SPI(void) {
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising SPI..."); 
	#endif
	//Setup SPI pins
	bcm2835_spi_begin();
	//Set CS pins polarity to low // CS NOT BEING IMPLEMENTED ON SLAVE DEVICE (PIC16F1828)
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, 0);
	bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS1, 0);
	//Set SPI clock speed
	//	BCM2835_SPI_CLOCK_DIVIDER_65536 = 0,       ///< 65536 = 262.144us = 3.814697260kHz (total H+L clock period) 
	//	BCM2835_SPI_CLOCK_DIVIDER_32768 = 32768,   ///< 32768 = 131.072us = 7.629394531kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_16384 = 16384,   ///< 16384 = 65.536us = 15.25878906kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_8192  = 8192,    ///< 8192 = 32.768us = 30/51757813kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_4096  = 4096,    ///< 4096 = 16.384us = 61.03515625kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_2048  = 2048,    ///< 2048 = 8.192us = 122.0703125kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_1024  = 1024,    ///< 1024 = 4.096us = 244.140625kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_512   = 512,     ///< 512 = 2.048us = 488.28125kHz
	//	BCM2835_SPI_CLOCK_DIVIDER_256   = 256,     ///< 256 = 1.024us = 976.5625MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_128   = 128,     ///< 128 = 512ns = = 1.953125MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_64    = 64,      ///< 64 = 256ns = 3.90625MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_32    = 32,      ///< 32 = 128ns = 7.8125MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_16    = 16,      ///< 16 = 64ns = 15.625MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_8     = 8,       ///< 8 = 32ns = 31.25MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_4     = 4,       ///< 4 = 16n6s = 62.5MHz
	//	BCM2835_SPI_CLOCK_DIVIDER_2     = 2,       ///< 2 = 8ns = 125MHz, fastest you can get
	//	BCM2835_SPI_CLOCK_DIVIDER_1     = 1,       ///< 1 = 262.144us = 3.814697260kHz, same as 0/65536
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16384); // (Set to 15.23kHz) 
	//Set SPI data mode
	//	BCM2835_SPI_MODE0 = 0,  // CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
	//	BCM2835_SPI_MODE1 = 1,  // CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
	//	BCM2835_SPI_MODE2 = 2,  // CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
	//	BCM2835_SPI_MODE3 = 3,  // CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE3); // could this be whats causing the problem ?????????????
	//Set which CS pin to use for next transfers // CS NOT BEING IMPLEMENTED ON SLAVE DEVICE (PIC16F1828)
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

// print out the description of a returned i2c reason code
void printI2CReasonCode (byte condition) {
	switch (condition) {
		case BCM2835_I2C_REASON_OK: printf("%s\n", "I2C success!"); break;
		case BCM2835_I2C_REASON_ERROR_NACK: printf("%s\n", "I2C fail: NACK"); break;	
		case BCM2835_I2C_REASON_ERROR_CLKT: printf("%s\n", "I2C fail: Received Clock Stretch Timeout"); break;	
		case BCM2835_I2C_REASON_ERROR_DATA: printf("%s\n", "I2C fail: Not all data written/recieved"); break;
	}
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
	printf("X: %06.4f [G],   ", accelXProcessed);
	printf("Y: %06.4f [G],   ", accelYProcessed);
	printf("Z: %06.4f [G]\n", accelZProcessed);
	printf("%s", "Gyroscope:      ");
	printf("X: %03.0f [%c/s], ", gyroXProcessed, 176);
	printf("Y: %03.0f [%c/s], ", gyroYProcessed, 176);
	printf("Z: %03.0f [%c/s]\n", gyroZProcessed, 176);
	printf("%s", "Magnetomter:    ");
	printf("Z: %04d [%cT],  ", magXProcessed, 181);
	printf("Z: %04d [%cT],  ", magYProcessed, 181);
	printf("Z: %04d [%cT]\n", magZProcessed, 181);
	printf("IMU/Mag Temp.:     %06.1f [%cC]\n", tempProcessed, 176);
	printf("Altitude:    %06.1f [m]\n\n", altiProcessedData);
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
	printf("X: %05d , ", accelXRaw);
	printf("Y: %05d , ", accelYRaw);
	printf("Z: %05d \n", accelZRaw);
	printf("%s", "Gyroscope:      ");
	printf("X: %05d, ", gyroXRaw);
	printf("Y: %05d, ", gyroYRaw);
	printf("Z: %05d\n", gyroZRaw);
	printf("%s", "Magnetomter:    ");
	printf("X: %05d, ", magXRaw);
	printf("Y: %05d, ", magYRaw);
	printf("Z: %05d\n", magZRaw);
	printf("%s", "Temperature:    ");
	printf("%06d\n", tempRaw);
	printf("%s", "Altitude:       ");
	printf("%08d\n\n", altiRawData);
}

// read from the MPL3115A2 Altitude sensor
void readAlti(void) {
	if (i2cRead_RS(altiI2CAddress, ALT_RA_INT_SOURCE) == 128) {// check which interrupt bit enabled the read
		altiRawData = i2cRead_RS(altiI2CAddress, ALT_RA_OUT_P_MSB);
		altiRawData <<= 8; // first 16 bits are whole numbers
		altiRawData += i2cRead_RS(altiI2CAddress, ALT_RA_OUT_P_CSB);
		altiRawData <<= 4; // last 4 bits are after a decimal point, resolution is 4 after decimal
		altiRawData += i2cRead_RS(altiI2CAddress, ALT_RA_OUT_P_LSB);
	}
}

// obtain intended actions as raw 8-bit peripheral values from remote controller
void readController() {
}

// read raw xyz data from the MPU9150's Accelorometer, temp & gyro (regs 3B - 48)
void readIMU(void) { 
	byte regAddress = 0x3B; // starting address
	byte i = 0;
	while (regAddress <= 0x48) {
		imuRawData[i] = i2cRead_RS(imuI2CAddress, regAddress++);    // access high byte, store in lower byte
		imuRawData[i] <<= 8; 					 // shift high byte to high side of int
		imuRawData[i++] += i2cRead_RS(imuI2CAddress, regAddress++); // access lower byte, store in lower byte
	}
	i2cRead_RS(imuI2CAddress, IMU_RA_INT_STATUS); // dummy read to clear int flag
}

// read raw xyz data from the MPU9150's Magnetometer
void readMag(void) {
	if (!(i2cRead_RS(magI2CAddress, MAG_RA_ST1))) // if no new data then return
		return;
	// READ 16-BIT XYZ VALUES
	byte regAddress = 0x03; // starting address
	byte i = 0;
	while (regAddress <= 0x08) {
		magRawData[i] = i2cRead_RS(magI2CAddress, regAddress++);    // access high byte, store in lower byte
		magRawData[i] <<= 8; 					 // shift high byte to high side of int
		magRawData[i++] += i2cRead_RS(magI2CAddress, regAddress++); // access lower byte, store in lower byte
	}
	// READ 8-BIT XYZ ADJUSTMENT VALUES
	regAddress = 0x10; // starting address
	// i = 3;
	while (regAddress <= 0x12) {
		magRawData[i++] += i2cRead_RS(magI2CAddress, regAddress++); // access and store byte
	}
}


// kalman filter to smooth out data obtained from sensors 
void runKalman(void) {
}

// PID (Proportional, integral, derivative) control to smooth out PWM
void runPID(void) {
}

// send pwm values to PIC through SPI [working correctly on pi side (tested through MOSI/MISO tie)]
void writePWM(void) {
	/*
	// make new pwm values to be overwritten (by SPI transfer shift protocol)
	byte pwmOUT[] = {pwm1OUT, pwm2OUT, pwm3OUT, pwm4OUT};
	byte tempHandshake;
	byte count;
	do {
		#ifdef DEBUG 
			printf("%s\r", "...attempting SPI initial handshake..."); 
		#endif
		tempHandshake = handshake;
		bcm2835_spi_transfer(tempHandshake);
	} while (tempHandshake != 0x51);  // handshake recieved back from pic should be 0x51 (0b01010001
	// transfer handshake then pwm values from array, pwm values replaced with received values, only pwm4 value of importance...
	bcm2835_spi_transfern(&pwmOUT[0], 4);
	// PIC will replace pmwOUT4 in array with 0x55 handshake if correctly used
	if (pwmOUT[3] == 0x55)
		return;
	else { 
		printf("%s\n","SPI for PWM failed, did not recieve manual ACK byte (0x55) in pwmOUT[7]:");
		for (count = 0; count < 4; count++) {
			printf("pwmOUT[");
			printf("%d", count);
			printf("] = %d\n", pwmOUT[count]); 
		}
	}
	*/
}












