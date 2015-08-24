#include "ak8975.h"
#include "easySerialComms.h"
#include <stdint.h>

static unsigned int magRawData[3]; // magnetometer x y z

static int magProcessedData[3]; // magnetometer x y z

static uint8_t magASAData[3];


// formats data from source device
void formatData(void) {
	byte i;
	for (i = 0; i < 3; i++)
		magProcesseData[i] = magRawData[i]*0.3;
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
}

// retrieve raw mag value on X axis 
unsigned  int get_MagX_Raw(void) {
	return MagXRaw;
}

// retrieve raw mag value on Y axis 
unsigned  int get_MagY_Raw(void) {
	return MagYRaw;
}

// retrieve raw mag value on Z axis 
unsigned int get_MagZ_Raw(void) {
	return MagZRaw;
}

// retrieve raw mag values as array
unsigned int *get_MagXYZ_Raw(void) {
	return MagRawData;
}

// retrieve processed mag value on X axis 
int get_MagX(void) {
	return MagXProcessed;
}

// retrieve processed mag value on Y axis 
int get_MagY(void) {
	return MagYProcessed;
}

// retrieve processed mag value on Z axis 
int get_MagZ(void) {
	return MagZProcessed;
}

// retrieve processed mag values as array 
int *get_MagXYZ(void) {
	return MagProcessedData;
}

// read raw data from the MPU9150's Magnetometer, store in magRawData[]
void readMag(void) {
	int temp1;
	if (!(i2cRead_SS(magI2CAddress, MAG_RA_ST1))) // if no new data then return
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
		temp = i2cRead_SS(magI2CAddress, i--); // read higher byte
		temp <<= 8;			     // shift to highest 8 bits
		temp += i2cRead_SS(magI2CAddress, i);  // read lower byte	int int
		// calculate
		magRawData[i-7] = (temp1)*(((magASAData[i-7] - 128)/256) + 1);	
	}

	// format to [uT] store in magProcessedData
	formatData();
}

// read adjustment values from the MPU9150's Magnetometer, store in magASAData[]
void readMag_ASA(void) {	
	for (i = MAG_RA_ASAX; i <= MAG_RA_ASAZ; i++) 
		magASAData[i-16] = i2cRead_SS(magI2CAddress, i);
}









