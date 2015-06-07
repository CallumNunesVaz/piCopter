#ifndef SIMPLECOMMS_H
#define SIMPLECOMMS_H

#define byte uint8_t
#define handshake 0x55

// comms initialisation
void init(void);
// I2C comms initialisation
void init_I2C(void);
// Serial comms initialisation
void init_Serial(void);
// SPI comms initialisation
void init_SPI(void);
// print out the description of a returned i2c reason code
void printI2CReasonCode (byte condition);
// read a value from one of the i2c devices registers
byte i2cRead(byte slaveAddress, byte regAddress);
// write a value to one of the i2c devices
void i2cWrite(byte slaveAddress, byte regAddress, byte val);

#endif









