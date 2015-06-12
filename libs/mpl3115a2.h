#ifndef MPL3115A2_H
#define MPL3115A2_H

/////////////////PINS/////////////////

#define IntrptPin1 RPI_BPLUS_GPIO_J8_15 // Altitude sensor interrupt pin gpio5
#define IntrptPin2 RPI_BPLUS_GPIO_J8_29 // Altitude sensor interrupt pin gpio22
#define I2CAddress 0x60 // 7-bit I2C slave address is 0x60 (1100000). 8-bit read is 0xC1, 8-bit write is 0xC0.

/////////sensor register Addresses///////

#define ALT_RA_SENSOR_STATUS 	0x00
#define ALT_RA_OUT_P_MSB 	0x01
#define ALT_RA_OUT_P_CSB 	0x02
#define ALT_RA_OUT_P_LSB 	0x03
#define ALT_RA_OUT_T_MSB 	0x04
#define ALT_RA_OUT_T_LSB 	0x05
#define ALT_RA_DR_STATUS 	0x06
#define ALT_RA_OUT_P_DELTA_MSB 	0x07
#define ALT_RA_OUT_P_DELTA_CSB 	0x08
#define ALT_RA_OUT_P_DELTA_LSB 	0x09
#define ALT_RA_OUT_T_DELTA_MSB 	0x0A
#define ALT_RA_OUT_T_DELTA_LSB 	0x0B
#define ALT_RA_WHO_AM_I 	0x0C
#define ALT_RA_F_STATUS 	0x0D
#define ALT_RA_F_DATA 		0x0E
#define ALT_RA_F_SETUP 		0x0F
#define ALT_RA_TIME_DLY 	0x10
#define ALT_RA_SYSMOD 		0x11
#define ALT_RA_INT_SOURCE 	0x12
#define ALT_RA_PT_DATA_CFG 	0x13
#define ALT_RA_BAR_IN_MSB 	0x14
#define ALT_RA_BAR_IN_LSB 	0x15
#define ALT_RA_P_TGT_MSB 	0x16
#define ALT_RA_P_TGT_LSB 	0x17
#define ALT_RA_T_TGT 		0x18
#define ALT_RA_P_WND_MSB 	0x19
#define ALT_RA_P_WND_LSB 	0x1A
#define ALT_RA_T_WND 		0x1B
#define ALT_RA_P_MIN_MSB 	0x1C
#define ALT_RA_P_MIN_CSB 	0x1D
#define ALT_RA_P_MIN_LSB 	0x1E
#define ALT_RA_T_MIN_MSB 	0x1F
#define ALT_RA_T_MIN_LSB 	0x20
#define ALT_RA_P_MAX_MSB 	0x21
#define ALT_RA_P_MAX_CSB 	0x22
#define ALT_RA_P_MAX_LSB 	0x23
#define ALT_RA_T_MAX_MSB 	0x24
#define ALT_RA_T_MAX_LSB 	0x25
#define ALT_RA_CTRL_1 		0x26
#define ALT_RA_CTRL_2 		0x27
#define ALT_RA_CTRL_3 		0x28
#define ALT_RA_CTRL_4 		0x29
#define ALT_RA_CTRL_5 		0x2A
#define ALT_RA_OFF_P 		0x2B
#define ALT_RA_OFF_T 		0x2C
#define ALT_RA_OFF_H 		0x2D

//other
#define byte uint8_t


////////////Global Variables////////////

// sensor data storage
uint32_t altitudeRaw; // raw altitude data is 24 bits

float altitudeProcessed;

///////////////Prototypes///////////////
// formats data from source device
static void formatData();
// get processed value for altitude
float get_Altitude();
// get raw sensor value of altitude
uint32_t get_Altitude_Raw();
// initialisation routine
void init_ALTI(void);
// read from the MPL3115A2 Altitude sensor
void readAlti(void);

#endif









