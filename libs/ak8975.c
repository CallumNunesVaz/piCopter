#include <stdint.h>
#include <stdio.h>
#include "ak8975.h"
#include "easySerialComms.h"

static unsigned int magRawData[3]; // magnetometer x y z

static int magProcessedData[3]; // magnetometer x y z

static uint8_t magASAData[3];


// formats data from source device
void formatMAGData(void) {
	byte i;
	for (i = 0; i < 3; i++)
		magProcessedData[i] = magRawData[i]*0.3;
}

// initialisation routine
void init_MAG(void) {
	printf("%s\n", "		-Initialising Magnetometer..."); 
	// OPERATING MODE CONTROL	
	// <3:0> 0000 = power down mode, 0001 = single measurement mode, 1000 = self test mode, 1111 = fuse ROM access mode 
	i2cWrite(I2CAddress, MAG_RA_CNTL, 1);
	
	// SELF TEST CONTROL
	// <6> self test control, 0 = normal, 1 = generate magnetic field 
	i2cWrite(I2CAddress, MAG_RA_ASTC, 0);

	// get ASA values
	readMag_ASA();
	
	printf("%s"," done.");
}

// retrieve raw mag value on X axis 
unsigned  int get_MagX_Raw(void) {
	return magXRaw;
}

// retrieve raw mag value on Y axis 
unsigned  int get_MagY_Raw(void) {
	return magYRaw;
}

// retrieve raw mag value on Z axis 
unsigned int get_MagZ_Raw(void) {
	return magZRaw;
}

// retrieve raw mag values as array
unsigned int *get_MagXYZ_Raw(void) {
	return magRawData;
}

// retrieve processed mag value on X axis 
int get_MagX(void) {
	return magXProcessed;
}

// retrieve processed mag value on Y axis 
int get_MagY(void) {
	return magYProcessed;
}

// retrieve processed mag value on Z axis 
int get_MagZ(void) {
	return magZProcessed;
}

// retrieve processed mag values as array 
int *get_MagXYZ(void) {
	return magProcessedData;
}

// read raw data from the MPU9150's Magnetometer, store in magRawData[]
void readMag(void) {
	byte i;
	int temp;

	/* Because the magnetometer puts itself to sleep after every read and only works in single-measurement mode 
	on the 9150 aux bus (grrrrrr) you must write it to being single measurement mode to wake it from sleep and THEN read*/
	i2cWrite(I2CAddress, MAG_RA_CNTL, 1);

	if (!(i2cRead_SS(I2CAddress, MAG_RA_ST1))) // if no new data then return
		return;
	/*
	Magnetometer has internal adjustment using the raw xxyyzz data magRawData[0-5]
	and the internal xyz adjustment values magRawData[6-8]. Use the following formula 
   	as specified by the AK8975 datasheet:

                            (ASA - 128)*0.5
		Hadj = H * (---------------  +  1 )
                                  128
	
	where H is the raw value, ASA is the sensitivity adjustment, and Hadj
	is the resultant adjusted value.
	
	for (i = 0; i < 3; i++) {
		magProcessedData[i] = (magRawData[i])*(((magRawData[i+3] - 128)/256) + 1);
	}
	Measuremnet data is stored in two’s complement and Little Endian format
	four sign bits, then 12 value bits as increments of 0.3[uT]
	*/

	for (i = MAG_RA_HZH; i <= MAG_RA_HXL; i--) { // goes down by two each loop
		// get main value
		temp = i2cRead_SS(I2CAddress, i--); // read higher byte
		temp <<= 8;			     // shift to highest 8 bits
		temp += i2cRead_SS(I2CAddress, i);  // read lower byte	int int
		// calculate
		magRawData[i-7] = (temp)*(((magASAData[i-7] - 128)/256) + 1);	
	}

	// format to [uT] store in magProcessedData
	formatMAGData();
}

// read adjustment values from the MPU9150's Magnetometer, store in magASAData[]
void readMag_ASA(void) {	
	byte i;
	for (i = MAG_RA_ASAX; i <= MAG_RA_ASAZ; i++) 
		magASAData[i-16] = i2cRead_SS(I2CAddress, i);
}









