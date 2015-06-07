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
#define imuI2CAddress 0x68 // ...when AD0 = 0, 7-bit adress 0b110100(1) when AD0 = 1
#define magI2CAddress 0x0C // MPU9150 has magnetometer connected to its aux i2c bus, must be put in pass-thru mode to access though
#define altiI2CAddress 0x60 // 7-bit I2C slave address is 0x60 (1100000). 8-bit read is 0xC1, 8-bit write is 0xC0.
// gps is connected to i2C line *(gps has no i2c firmware support as of 1/1/2015, will be used through pic-mcu by default)

/////////sensor register Addresses///////

// IMU sensor isters, all 8 bits
#define IMU_RA_SMPRT_DIV 		0x19
#define IMU_RA_IMU_CONFIG 		0x1A
#define IMU_RA_GYRO_CONFIG 		0x1B
#define IMU_RA_ACCEL_CONFIG 		0x1C
#define IMU_RA_FIFO_EN 			0x23
#define IMU_RA_I2C_MST_CTRL 		0x24
#define IMU_RA_INT_PIN_CFG 		0x37
#define IMU_RA_INT_ENABLE 		0x38
#define IMU_RA_INT_STATUS 		0x3A
#define IMU_RA_I2C_MST_DELAY_CTRL 	0x67
#define IMU_RA_SIGNAL_PATH_RESET 	0x68
#define IMU_RA_USER_CTRL 		0x6A
#define IMU_RA_PWR_MGMT_1 		0x6B
#define IMU_RA_PWR_MGMT_2 		0x6C

// Magnetometer sensor isters, all 8 bits
#define MAG_RA_INFO 	0x01 
#define MAG_RA_ST1 	0x02
#define MAG_RA_HXL 	0x03
#define MAG_RA_HXH 	0x04
#define MAG_RA_HYL 	0x05
#define MAG_RA_HYH 	0x06
#define MAG_RA_HZL 	0x07
#define MAG_RA_HZH 	0x08
#define MAG_RA_ST2 	0x09
#define MAG_RA_CNTL 	0x0A
#define MAG_RA_ASTC 	0x0C
#define MAG_RA_I2CDIS 	0x0F
#define MAG_RA_ASAX 	0x10
#define MAG_RA_ASAY 	0x11
#define MAG_RA_ASAZ 	0x12

// Altitude sensor isters, all 8 bits
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

#define handshake 0x55

// NEED BOOLEAN VALUES
enum bool {FALSE, TRUE};

// different levels of debugging enable printing of different info
enum debugState {NoDebug, DebugInitOnly, DebugAll};


////////////Global Variables////////////

// pwm values to go to pwm output
extern byte pwmOUT[4];
#define pwm1OUT pwmOUT[0]; 
#define pwm2OUT pwmOUT[1]; 
#define pwm3OUT pwmOUT[2]; 
#define pwm4OUT pwmOUT[3];

// sensor data storage
extern uint32_t altitudeRaw; // raw altitude data is 24 bits

extern float altitudeProcessed;

extern unsigned int magRawData[6]; // magnetometer x y z
#define magXRaw magRawData[0]
#define magYRaw magRawData[1]
#define magZRaw magRawData[2]
#define magXadjust magRawData[3]
#define magYadjust magRawData[4]
#define magZadjust magRawData[5]

extern int magProcessedData[3]; // magnetometer x y z
#define magXProcessed magProcessedData[0]
#define magYProcessed magProcessedData[1]
#define magZProcessed magProcessedData[2]

extern unsigned int imuRawData[7]; // accel x y z, temp, gyro x y z
#define accelXRaw imuRawData[0]
#define accelYRaw imuRawData[1]
#define accelZRaw imuRawData[2]
#define tempRaw   imuRawData[3]
#define gyroXRaw  imuRawData[4]
#define gyroYRaw  imuRawData[5]
#define gyroZRaw  imuRawData[6]

// Accelarometer data in G's format
extern float imuProcessedData[7]; // accel x y z, temp, gyro x y z
#define accelXProcessed imuProcessedData[0]
#define accelYProcessed imuProcessedData[1]
#define accelZProcessed imuProcessedData[2]
#define tempProcessed   imuProcessedData[3]
#define gyroXProcessed  imuProcessedData[4]
#define gyroYProcessed  imuProcessedData[5]
#define gyroZProcessed  imuProcessedData[6]

extern uint8_t ACCEL_RANGE;

extern uint16_t GYRO_RANGE;

extern uint8_t DLPF_FREQ;

extern uint32_t LOOP_CNT_DIV;

///////////////Prototypes///////////////

// formats data from source device
void formatData();
// initialisation routine, called from main()
void init(void);
// comms initialisation
void init_Comms(void);
// I2C comms initialisation
void init_I2C(void);
// initialise GPIO pin directions and state-formats using bcm2835 function libraries
void init_PinDir(void);
// initialise i2c altitude and imu sensors
void init_Sensors(void);
// Serial comms initialisation
void init_Serial(void);
// SPI comms initialisation
void init_SPI(void);
// print out the description of a returned i2c reason code
void printI2CReasonCode (byte condition);
// print out Processed sensor data in 6-decimal format, used for analysis/debugging
void printProcessedData(void);
// print out raw sensor data, used for analysis/debugging
void printRawData(void);
// read from the MPL3115A2 Altitude sensor
void readAlti(void);
// obtain intended actions as raw 8-bit peripheral values from remote controller
void readController();
// read from the MPU9150
void readIMU(void);
// read from the MPU9150's Magnetometer
void readMag(void);
// kalman filter to smooth out data obtained from sensors 
void runKalman(void);
// PID (Proportional, integral, derivative) control to smooth out PWM
void runPID(void);
// read a value from one of the i2c devices registers (repeated start format)
byte i2cRead_RS(byte slaveAddress, byte regAddress);
// read a value from one of the i2c devices registers (single start format)
byte i2cRead_SS(byte slaveAddress, byte regAddress) 
// write a value to one of the i2c devices
void i2cWrite(byte slaveAddress, byte regAddress, byte val);
// send pwm values to PIC through SPI
void writePWM(void);

#endif









