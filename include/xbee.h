#ifndef XBEE_H
#define XBEE_H

#define xbeeRTS RPI_BPLUS_GPIO_J8_18  //gpio24
#define xbeeCTS RPI_BPLUS_GPIO_J8_36 // gpio16
#define xbeeRSSI RPI_BPLUS_GPIO_J8_38 // gpio20
#define xbeeSLEEP RPI_BPLUS_GPIO_J8_32 // gpio12
#define xbeeDTR RPI_BPLUS_GPIO_J8_40 // gpio 21
#define xbeeRESET RPI_BPLUS_GPIO_J8_22 //gpio 25

#define byte uint8_t
#define handshake 0x55

// obtain intended actions as raw 8-bit peripheral values from remote controller
void readController();

#endif









