//-------------------------------------------------------Libraries-------------------------------------------------------//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "mpu9150.h"

//--------------------------------------------------- Global Variables -------------------------------------------------//

static unsigned int imuRawData[7]; // accel x y z, temp, gyro x y z

unsigned int accelRawDataCopy[3];
unsigned int gyroRawDataCopy[3];

static float imuProcessedData[7]; // accel x y z, temp, gyro x y z

float accelProcessedDataCopy[3];
float gyroProcessedDataCopy[3];

float imuProcessedDataCopy[7];

static uint8_t ACCEL_RANGE;
static uint16_t GYRO_RANGE;
static uint8_t DLPF_FREQ; 

//----------------------------------------------- Global Program Modifiers ----------------------------------------------//

//-------------------------------------------------------Subroutines-----------------------------------------------------//
// functions in alphabetical order

// Format data from raw sensor values into SI units
void formatIMUData() {
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
}

// retrieve raw sensor data from accelarometer
unsigned int get_AccelX_Raw() {
	return accelXRaw;
}
// retrieve raw sensor data from accelarometer
unsigned int get_AccelY_Raw() {
	return accelYRaw;
}
// retrieve raw sensor data from accelarometer
unsigned int get_AccelZ_Raw() {
	return accelZRaw;
}
// retrieve location of snapshot of raw sensor data from accelarometer XYZ
unsigned int *get_AccelXYZ_Raw() {
	byte i;
	for (i = 0; i < 3; i++) {
		accelRawDataCopy[i] = imuRawData[i]; }
	return accelRawDataCopy;
}
// retrieve processed sensor data from accelarometer
float get_AccelX() {
	return accelXProcessed;
}
// retrieve processed sensor data from accelarometer
float get_AccelY() {
	return accelYProcessed;
}
// retrieve processed sensor data from accelarometer
float get_AccelZ() {
	return accelZProcessed;
}
// retrieve location of snapshot of processed sensor data from accelarometer XYZ
float *get_AccelXYZ() {
	byte i;
	for (i = 0; i < 3; i++) {
		accelProcessedDataCopy[i] = imuProcessedData[i]; }
	return accelProcessedDataCopy;
}
// retrieve raw sensor data from gyroscope
unsigned int get_GyroX_Raw() {
	return gyroXRaw;
}
// retrieve raw sensor data from gyroscope
unsigned int get_GyroY_Raw() {
	return gyroYRaw;
}
// retrieve raw sensor data from gyroscope
unsigned int get_GyroZ_Raw() {
	return gyroZRaw;
}
// retrieve location of snapshot of raw sensor data from gyroscope XYZ
unsigned int *get_GyroXYZ_Raw() {
	byte i;
	for (i = 0; i < 3; i++) {
		gyroRawDataCopy[i] = imuRawData[i+4]; }
	return gyroRawDataCopy;
}
// retrieve processed sensor data from gyroscope X axis
float get_GyroX() {
	return gyroXProcessed;
}
// retrieve processed sensor data from gyroscope Y axis
float get_GyroY() {
	return gyroYProcessed;
}
// retrieve processed sensor data from gyroscope Z axis
float get_GyroZ() {
	return gyroZProcessed;
}
// retrieve location of snapshot of processed sensor data from gyroscope XYZ
float *get_GyroXYZ() {
	byte i;
	for (i = 0; i < 3; i++) {
		gyroProcessedDataCopy[i] = imuProcessedData[i+4]; }
	return gyroProcessedDataCopy;
}
// retrieve raw sensor data from temperature sensor
unsigned int get_Temp_Raw() {
	return tempRaw;
}
// retrieve processed sensor data from temperature sensor
float get_Temp() {
	return tempProcessed;
}

// initialise i2c altitude and imu sensors
void init_IMU(byte a, uint16_t g, byte d) {
	byte i; // variable for for-loops

	printf("%s\n","		-Initialising IMU...");

	// <7> DEVICE RESET BIT						      <6> SLEEP ENABLE BIT
	// <5> toggles whether device will cycle between wake and sleep       <4> UNIMPLEMENTED
	// <3> Disables temp sensor (when set to 1)
	// <2:0> Clock Select: 0=internal 8MHz, 1=PLL w/ gyro X ref, 2=PLL w/ gyro Y ref, 3=PLL w/ gyro Z ref,  4=PLL w/ external 32kHz
	// 		       5= PLL w/ external 19.2MHz, 6 = RESV, 7 = stops clock
	// Result = 0b0000001 = 0
	i2cWrite(imuI2CAddress, IMU_RA_PWR_MGMT_1, 1);

	// sample rate = (gyro output rate)/(1 + SMPRT_DIV)
	// gyro output rate =8KHz w/o DLPF and 1kHz with
	i2cWrite(imuI2CAddress, IMU_RA_SMPRT_DIV, 1); // 500 Hz

	// set full scale range of gyroscope output (at bits <4:3>)
	// 0 = 250 degrees/s, 1 = 500 degrees/s, 2 = 1000 degrees/s, 3 = 2000 degrees/s
	set_IMU_GyroRange(g);

	// set full scale range of accelorometer output (at bits <4:3>)
	// 0 = 2g's, 1 = 4g's, 2 = 8g's, 3 =  16g's
	set_IMU_AccelRange(a);

	// set value of digital low-pass filter (DLPF) for Accel and Gyro (samples out vibrations)
	// 0=~256Hz, 1=~185Hz, 2=~95Hz, 3=~44Hz, 4=~22Hz, 5=~10Hz, 6=~5Hz, 7=RESERVED
	// FSYNC not used, DLPF value held in 3 lowest bits so can have value written directly
	set_IMU_DLPF(d);

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
	// Result = 0b00100000 = decimal 34
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
	i2cWrite(imuI2CAddress, IMU_RA_USER_CTRL, 0);

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

	printf("%s"," done.");
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
	formatIMUData();
}

// set full scale range of accelorometer, 2g's, 4g's, 8g's, 16g's
void set_IMU_AccelRange(byte range) {
	ACCEL_RANGE = range;
	printf("	Setting IMU Accelerometer range to %d... ", range);
	// set full scale range of accelorometer output (at bits <4:3>) 
	// 0 = 2g's, 1 = 4g's, 2 = 8g's, 3 =  16g's
	switch (range) {
		case 2: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 0); break;
		case 4: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 8); break;
		case 8: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 16); break;
		case 16: i2cWrite(imuI2CAddress, IMU_RA_ACCEL_CONFIG, 24); break;
		default: printf("%s\n", "ERROR: invalid accelerometer range selected"); while(1); break;
	}
	printf("%s\n", "done");
}

// set value of digital low-pass filter (DLPF) for Accel and Gyro (samples out vibrations)
void set_IMU_DLPF(byte speed) {
	byte regContents;
	printf("	Setting IMU DLPF range to %d... ", speed);
	// 0=~256Hz, 1=~185Hz, 2=~95Hz, 3=~44Hz, 4=~22Hz, 5=~10Hz, 6=~5Hz, 7=RESERVED
	switch (speed) {
		case 255: speed = 0; break;
		case 185: speed = 1; break;
		case 95: speed = 2; break;
		case 44: speed = 3; break;
		case 22: speed = 4; break;
		case 10: speed = 5; break;
		case 5: speed = 6; break;
		default: printf("%s\n", "ERROR: invalid DLPF value selected"); while(1); break;
	}
	DLPF_FREQ = speed;
	// FSYNC not used, DLPF value held in 3 lowest bits so can have value written directly
	// read current contents
	regContents = (0b11111000) & (i2cRead_RS(imuI2CAddress, IMU_RA_IMU_CONFIG));
	// change DLPF only
	i2cWrite(imuI2CAddress, IMU_RA_IMU_CONFIG, regContents+speed);
	printf("%s\n", "done");
}

// set full scale range of gyroscope output 250, 500, 1000, 2000 deg/s
void set_IMU_GyroRange (unsigned int range) {
	GYRO_RANGE = range;
	// set full scale range of gyroscope output (at bits <4:3>) 
	// 0 = 250 degrees/s, 1 = 500 degrees/s, 2 = 1000 degrees/s, 3 = 2000 degrees/s
	printf("	Setting IMU Gyroscope range to %d... ", range);
	switch (range) {
		case 250: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 0); break;
		case 500: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 8); break;
		case 1000: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 16); break;
		case 2000: i2cWrite(imuI2CAddress, IMU_RA_GYRO_CONFIG, 24); break;
		default: printf("%s\n", "ERROR: invalid gyroscope range selected"); while(1); break;
	}
	printf("%s\n", "done");
}
