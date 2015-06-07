#ifndef EASYSERIALCOMMS_H
#define EASYSERIALCOMMS_H

#define byte uint8_t
#define handshake 0x55

// comms initialisation
extern void init_Comms(void);
// I2C comms initialisation
void init_I2C(void);
// Serial comms initialisation
void init_Serial(void);
// SPI comms initialisation
void init_SPI(void);
// print out the description of a returned i2c reason code
void printI2CReasonCode (byte condition);
// read a single value from one of the sensors registers (repeated-start format) used by mpu9150 and mpl3115a2
extern byte i2cRead_RS(byte slaveAddress, byte regAddress);
// read a single value from one of the sensors registers (single-start format) used by ak8975
extern byte i2cRead_SS(byte slaveAddress, byte regAddress);
// write a value to one of the i2c devices
extern void i2cWrite(byte slaveAddress, byte regAddress, byte val);

#endif









