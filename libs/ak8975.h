#ifndef AK8975_H
#define AK8975_H

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

#define byte uint8_t

#define I2CAddress 0x0C // MPU9150 has magnetometer connected to its aux i2c bus, must be put in pass-thru mode to access though

unsigned int magRawData[3];
#define magXRaw magRawData[0]
#define magYRaw magRawData[1]
#define magZRaw magRawData[2]

int magProcessedData[3];
#define magXProcessed magProcessedData[0]
#define magYProcessed magProcessedData[1]
#define magZProcessed magProcessedData[2]

byte magASAData[3];
#define magASAX magASAData[0]
#define magASAY magASAData[1]
#define magASAZ magASAData[2]

// formats data from source device
void formatData(void);
// initialisation routine
extern void init_MAG(void);
// retrieve raw mag value on X axis 
extern unsigned int get_MagX_Raw(void);
// retrieve raw mag value on Y axis 
extern unsigned int get_MagY_Raw(void);
// retrieve raw mag value on Z axis 
extern unsigned int get_MagZ_Raw(void);
// retrieve raw mag values as array
extern unsigned int[] get_MagXYZ_Raw(void);
// retrieve processed mag value on X axis 
extern int get_MagX(void);
// retrieve processed mag value on Y axis 
extern int get_MagY(void);
// retrieve processed mag value on Z axis 
extern int get_MagZ(void);
// retrieve processed mag values as array
extern int[] get_MagXYZ(void);
// read from the MPU9150's Magnetometer, store in magRawData[]
extern void readMag(void);
// read magnetometer adjustment values for future calculations
void readMag_ASA(void)

#endif









