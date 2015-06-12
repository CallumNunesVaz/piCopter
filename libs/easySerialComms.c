#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "bcm2835.h"   // bcm2835 library to use chip pins and comms interfaces, needs to be installed (see top of document)


// read a single value from one of the sensors registers (repeated-start format) used by mpu9150 and mpl3115a2
uint8_t i2cRead_RS(byte slaveAddress, byte regAddress) {
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
uint8_t i2cRead_SS(byte slaveAddress, byte regAddress) {
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

// comms initialisation
void init_Comms(void) {
	#ifdef DEBUG 
		printf("%s\n", "	-Initialising Communications Interfaces..."); 
	#endif
	init_I2C();       	// establish i2c comms with sensors
	init_SPI();       	// establish SPI comms with PIC
	init_Serial();    	// establish Serial comms with xbee
}

// I2C comms initialisation
static void init_I2C(void) {
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising I2C..."); 
	#endif
	bcm2835_i2c_begin(); // enables i2c module
	bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626); // speed set to 400kHz ("fast mode")
}

// Serial comms initialisation
static void init_Serial(void) {
	#ifdef DEBUG 
		printf("%s\n", "		-Initialising USART..."); 
	#endif
}

// SPI comms initialisation
static void init_SPI(void) {
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
static void printI2CReasonCode (byte condition) {
	switch (condition) {
		case BCM2835_I2C_REASON_OK: printf("%s\n", "I2C success!"); break;
		case BCM2835_I2C_REASON_ERROR_NACK: printf("%s\n", "I2C fail: NACK"); break;	
		case BCM2835_I2C_REASON_ERROR_CLKT: printf("%s\n", "I2C fail: Received Clock Stretch Timeout"); break;	
		case BCM2835_I2C_REASON_ERROR_DATA: printf("%s\n", "I2C fail: Not all data written/recieved"); break;
	}
}

