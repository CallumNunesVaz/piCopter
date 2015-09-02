#ifndef PICOPTER_H
#define PICOPTER_H

/////////////////PINS/////////////////

#define imuIntrptPin RPI_BPLUS_GPIO_J8_11 // MPU9150 imu unit interrrupt pin gpio17
#define altiIntrptPin1 RPI_BPLUS_GPIO_J8_15 // Altitude sensor interrupt pin gpio5
#define altiIntrptPin2 RPI_BPLUS_GPIO_J8_29 // Altitude sensor interrupt pin gpio22
#define buzzerPin RPI_BPLUS_GPIO_J8_31      // gpio 6
#define gpsNavLock RPI_BPLUS_GPIO_J8_12     // gpio 18
#define xbeeRTS RPI_BPLUS_GPIO_J8_18  //gpio24
#define xbeeCTS RPI_BPLUS_GPIO_J8_36 // gpio16
#define xbeeRSSI RPI_BPLUS_GPIO_J8_38 // gpio20
#define xbeeSLEEP RPI_BPLUS_GPIO_J8_32 // gpio12
#define xbeeDTR RPI_BPLUS_GPIO_J8_40 // gpio 21
#define xbeeRESET RPI_BPLUS_GPIO_J8_22 //gpio 25

//other
#define byte uint8_t

#define handshake 0x55

// NEED BOOLEAN VALUES
enum bool {FALSE, TRUE};

////////////Global Variables////////////


///////////////Prototypes///////////////

// initialisation routine, called from main()
void init(byte a, uint16_t g, byte d);
// comms initialisation
void init_PinDir(void);
// initialise i2c altitude and imu sensors
void init_Sensors(byte a, uint16_t g, byte d);
// print out Processed sensor data in 6-decimal format, used for analysis/debugging
void printProcessedData(void);
// print out raw sensor data, used for analysis/debugging
void printRawData(void);

#endif









