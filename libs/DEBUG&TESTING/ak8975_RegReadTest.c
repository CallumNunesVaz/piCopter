#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "../easySerialComms.h"

int i;

uint_8 ak8975Regs[] = {
0x00,
0x01,
0x02,
0x03,
0x04,
0x05,
0x06,
0x07,
0x08,
0x09,
0x0A,
//0x0B, DO NOT ACCESS
0x0C,
//0x0D, DO NOT ACCESS
//0x0E, DO NOT ACCESS
0x0F,
0x10,
0x11,
0x12
};

char *ak8975RegNames[] = {
"MAG_RA_WIA   ",
"MAG_RA_INFO  ",
"MAG_RA_ST1   ",
"MAG_RA_HXL   ",
"MAG_RA_HXH   ",
"MAG_RA_HYL   ",
"MAG_RA_HYH   ",
"MAG_RA_HZL   ",
"MAG_RA_HZH   ",
"MAG_RA_ST2   ",
"MAG_RA_CNTL  ",
//"MAG_RA_RSV   ", DO NOT ACCESS
"MAG_RA_ASTC  ",
//"MAG_RA_TS1   ", DO NOT ACCESS
//"MAG_RA_TS2   ", DO NOT ACCESS
"MAG_RA_I2CDIS", 	
"MAG_RA_ASAX  ",
"MAG_RA_ASAY  ",
"MAG_RA_ASAZ  "
};

int arrayLength = sizeof(ak8975Regs) / sizeof(uint_8);

int main(void) {
	printf("            REG               CONTENTS   \n");
	for (i = 0; i < arrayLength; startAddress++) {
		printf("Register: ");
		char *pos = ak8975RegNames[i];
        	while (*pos != '\0') {
            		printf("%c\n", *(pos++));
        	}
		printf("  @%03d   Contents: %03d\n", ak8975Regs[i], i2cRead_SS(0x0C, ak8975Regs[i]));
	}
	exit(0);
} 
