//-------------------------------------------------------Libraries-------------------------------------------------------//
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

//--------------------------------------------------- Global Variables -------------------------------------------------//

// sensor data storage
static uint32_t altiRawData; // raw altitude data is 24 bits

static float altiProcessedData;

//-------------------------------------------------------Subroutines-----------------------------------------------------//
// functions in alphabetical order

// Format data from raw sensor values into SI units
static void formatData() {
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

// get processed value for altitude
float get_Altitude() {
	return altiRawData;
}

// get raw sensor value of altitude
uint32_t get_Altitude_Raw() {
	return altiProcessedData;
}

// initialise i2c altitude sensor
void init_ALTI(void) {
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

// read from the MPL3115A2 Altitude sensor
void readAlti(void) {
	if (i2cRead_RS(altiI2CAddress, ALT_RA_INT_SOURCE) == 128) {// check which interrupt bit enabled the read
		altiRawData = i2cRead_RS(altiI2CAddress, ALT_RA_OUT_P_MSB);
		altiRawData <<= 8; // first 16 bits are whole numbers
		altiRawData += i2cRead_RS(altiI2CAddress, ALT_RA_OUT_P_CSB);
		altiRawData <<= 4; // last 4 bits are after a decimal point, resolution is 4 after decimal
		altiRawData += i2cRead_RS(altiI2CAddress, ALT_RA_OUT_P_LSB);
	}
	formatData();
}



