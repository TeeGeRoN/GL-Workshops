/* This is a library for PCA9685 16-channel controller.
 *  Uses I2C to communicate, 2 pins are required to interface.
 */


#include "main.h"
#include <stdio.h>
#include <stdlib.h>

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PCA9685LIB_H
#define PCA9685LIB_H

#ifdef __cplusplus
extern "C" {
#endif


// Register addresses from data sheet
#define PCA9685_MODE1_REG           0x00

#define PCA9685_ALLCALL_REG         0x05
#define PCA9685_LED0_REG_ON_L       0x06      // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_LED0_REG_ON_H       0x07      //LED0 output and brightness control byte 1
#define PCA9685_LED0_REG_OFF_L      0x08      //LED0 output and brightness control byte 2
#define PCA9685_LED0_REG_OFF_H      0x09      //LED0 output and brightness control byte 3
#define PCA9685_PRESCALE_REG        0xFE      //prescaler for output frequency
#define PCA9685_ALLLED_REG_ON_L     0xFA      //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define PCA9685_ALLLED_REG_ON_H     0xFB      //load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define PCA9685_ALLLED_OFF_L        0xFC      //load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define PCA9685_ALLLED_OFF_H        0xFD      //load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define LED_MULTIPLYER              4    	  // For the other 15 channels
// Mode1 register values
#define PCA9685_MODE1_RESTART       0x80
#define PCA9685_MODE1_EXTCLK        0x40
#define PCA9685_MODE1_AUTOINC       0x20
#define PCA9685_MODE1_SLEEP         0x10

#define PCA9685_MODE1_ALLCALL       0x01

#define PCA9685_SW_RESET            0x06       // Sent to address 0x00 to reset all devices on Wire line
#define PCA9685_PWM_FULL            0x1000     // Special value for full on/full off LEDx modes

#define PCA9685_CHANNEL_COUNT       16
#define PCA9685_MIN_CHANNEL         0
#define PCA9685_MAX_CHANNEL         (PCA9685_CHANNEL_COUNT - 1)
#define PCA9685_ALLLED_CHANNEL      -1                  // Special value for ALLLED registers
#define FREQUENCY_OSCILLATOR 		25000000  			// Int. osc. frequency




uint16_t _i2cAddress;       // Module's i2c address
I2C_HandleTypeDef _hi2c;  	// I2C_Handler
uint32_t oscillator_freq;	// Osc. frequency

/* Functions for interacting with PCA9685 PWM chip */
void initPCA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
void writeRegister(uint8_t regAddress, uint8_t value);
void resetDevice();
void wakeupPCA();
void sleepPCA();
uint16_t getAllPWM();
void setPWMFrequency(float pwmFrequency);
void setOscillatorFrequency(uint32_t Frequency);
uint32_t getOscillatorFrequency(void);
uint16_t getPWMFrequency();
void setLedOn(uint8_t led);
void setLedOff(uint8_t led);
void setAllLedOn();
void setAllLedOff();
void setLedPWM(uint8_t led, int value);
void setPWM(uint8_t led, int on_value, int off_value);
void setAllLedPWM(int value);
void setAllPWM(int on_value, int off_value);
uint16_t getPWM(uint8_t led);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
