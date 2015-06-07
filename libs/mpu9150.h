#ifndef MPU9150_H
#define MPU9150_H

/////////////////PINS/////////////////

#define imuIntrptPin RPI_BPLUS_GPIO_J8_11 // MPU9150 imu unit interrrupt pin gpio17
#define I2CAddress 0x68 // ...when AD0 = 0, 7-bit adress 0b110100(1) when AD0 = 1

/////////sensor register Addresses///////

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

#define byte uint8_t

#define handshake 0x55

unsigned int imuRawData[7]; // accel x y z, temp, gyro x y z
#define accelXRaw imuRawData[0]
#define accelYRaw imuRawData[1]
#define accelZRaw imuRawData[2]
#define tempRaw   imuRawData[3]
#define gyroXRaw  imuRawData[4]
#define gyroYRaw  imuRawData[5]
#define gyroZRaw  imuRawData[6]

// Accelarometer data in G's format
float imuProcessedData[7]; // accel x y z, temp, gyro x y z
#define accelXProcessed imuProcessedData[0]
#define accelYProcessed imuProcessedData[1]
#define accelZProcessed imuProcessedData[2]
#define tempProcessed   imuProcessedData[3]
#define gyroXProcessed  imuProcessedData[4]
#define gyroYProcessed  imuProcessedData[5]
#define gyroZProcessed  imuProcessedData[6]

uint8_t ACCEL_RANGE;

uint16_t GYRO_RANGE;

uint8_t DLPF_FREQ;


///////////////Prototypes///////////////

// formats data from source device
void formatData(void);
// initialisation routine
void init_ACCEL(void);
// initialisation routine
void init_GYRO(void);
// read from the MPU9150
void readIMU(void);

// void set_gyro_Range(int range);
// void get_gyro_Range(int range);
// void set_accel_Range(int range);
// void get_accel_Range(int range);


#endif









