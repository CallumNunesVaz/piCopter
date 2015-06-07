# piCopter

Author: Callum Nunes-Vaz

Current Problems: 
 * Currently investigating SPI comms (pic won't seem to talk back, most probably problem with PIC).
 * Magnetomer gives NACKS, must have incorrectly configured aux mode on mpu9150
 * Entire program contained in single .c file for dev purposes, will be reformatted for 1.0 version
 * Some BCM2835 functions cause CPU hangs, must find and test alternative method. see init_PinDir().

Non-Standard (for C) Software Dependencies: 
 * C library for Broadcom BCM 2835 as used in Raspberry Pi [V1.42] {http://www.airspayce.com/mikem/bcm2835/}

Description: 
This program drives a sensor board attached to the 40-pin GPIO in order to fly a quadcopter. Input is
taken serially via an Xbee unit which is combined with data read from i2c sensors. The desired speed 
of the propellors is calculated and the data for the PWM controlling PIC is sent through SPI to a 
PIC18F1828 which drives 4 speed controllers with 1kHz PWM. 
This program is intentionally overcomplicated in terms of devices and their communication interfaces, 
and is intended to be used as a learning tool.


Board Communication scheme diagram:

                             -------------------
                             | Raspberry Pi A+ |
                             -------------------
                               |      |      |              --------------------                ----------------------
     -------------------i2c----'      |      '-------SPI----|    PIC16F1828    |------USART------| VENUS638FLPx (GPS) |
     |         |                    USART                   --------------------                ----------------------
     |   -------------                |                      |     |      |     |
     |	 | MPL3115A2 |                |  		   PWM    PWM    PWM    PWM			
     |	 -------------	     	  --------	            |      |      |      |
 -----------			  | XBEE |	        |ESC1|  |ESC2|  |ESC3|  |ESC4|
 | MPU9150 |                      --------
 -----------
 
IMPORTANT NOTES:
 * To compile using gcc run  "gcc -o piCopter.prog piCopter.c -l bcm2835" in target dir (make sure bcm2835 libs installed)
 * MUST BE RUN AS SUPER USER TO ACCESS GPIO e.g. "sudo ./piCopterprogram.run"
 * Debugging is enabled at compile time by setting debug macro as defined e.g. "gcc -o piCopter.prog piCopter.c -DDEBUG -l bcm2835"

