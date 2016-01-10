# piCopter

Author: Callum Nunes-Vaz

Current Problems: 
 * Magnetomerwill not update values, appears to succeffully communicate and show up on i2c line through mpu9150 but this is suspected to be untrue
 * Some BCM2835 pin functions cause CPU hangs, must find and test alternative method. see init_PinDir() for commented out code.

Current Checklist:
 * Finish code framework that reads with sensors
 * Finish code that converts sensor values and prints them
 * start and finish algorithms to achieve yaw pitch and roll
 * All of above for unpurchased GPS module

Non-Standard (for C) Software Dependencies: 
 * C library for Broadcom BCM 2835 as used in Raspberry Pi [V1.42] {http://www.airspayce.com/mikem/bcm2835/}, not included to avoid IP infringement

Description: 
This program drives a sensor board attached to the 40-pin GPIO in order to fly a quadcopter. Input is
taken serially via an Xbee unit which is combined with data read from i2c sensors. The desired speed 
of the propellors is calculated and the data for the PWM controlling PIC is sent through SPI to a 
PIC18F1828 which drives 4 speed controllers with 1kHz PWM. 
This program is intentionally overcomplicated in terms of devices and their communication interfaces, 
and is intended to be used as a learning tool.


Board Communication scheme diagram:
```
                             -------------------
                             | Raspberry Pi A+ |
                             -------------------
                               |      |      |              --------------------                ----------------------
     -------------------i2c----'      |      '-------SPI----|    PIC16F1828    |------USART------| VENUS638FLPx (GPS) |
     |         |                    USART                   --------------------                ----------------------
     |   -------------                |                      |     |      |     |
     |   | MPL3115A2 |                |                    PWM    PWM    PWM    PWM			
     |	 -------------	     	  --------	            |      |      |      |
 -----------			  | XBEE |	        |ESC1|  |ESC2|  |ESC3|  |ESC4|
 | MPU9150 |                      --------
 -----------
```

IMPORTANT NOTES:
 * Compile using makefile
 * Run using bash script e.g. "sudo ./run.sh"


