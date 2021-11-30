#include "PCA9685lib.h"


void writeRegister(uint8_t regAddress, uint8_t value)
{
	/* Writes in the register.
	 * param regAddress - where (at what address) to write
	 * param value - value to write
	 * */
	HAL_Delay(10);
	uint8_t TxBuffer[8];
	TxBuffer[0] = regAddress;
	TxBuffer[1] = value;
	HAL_I2C_Master_Transmit(&_hi2c, _i2cAddress, (uint8_t *) &TxBuffer, 2, 1000);
}

uint8_t readRegister(uint8_t regAddress)
{
	/* Reads the register.
	 * param regAddress - from which register to read (his address)
	 * */
	uint8_t RxBuffer;
	uint8_t Addr = regAddress;
	HAL_I2C_Master_Transmit(&_hi2c, _i2cAddress, &Addr, sizeof(Addr), 1000);
	HAL_I2C_Master_Receive(&_hi2c, _i2cAddress, &RxBuffer, sizeof(RxBuffer), 1000);
	return RxBuffer;
}


void initPCA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
	/* Initializes the PCA9685, sets the default frequency (can be disabled).
	 * param hi2c - I2C_Handler
	 * param DevAddress - PCA9685 address
	 * */
	HAL_Delay(10);
    _i2cAddress = DevAddress;
    _hi2c = *hi2c;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE1_ALLCALL);
    resetDevice();
    // set the default internal frequency
    setOscillatorFrequency(FREQUENCY_OSCILLATOR);
    // set a default frequency
    setPWMFrequency(1000);

}

void setOscillatorFrequency(uint32_t Freq)
{
	/* Setter for the internally tracked oscillator used for
	 * frequency calculations
	 * */
	  oscillator_freq = Freq;
}
uint32_t getOscillatorFrequency(void)
{
	/* Getter for the internally tracked oscillator used for
	 * frequency calculations
	 * */
	return oscillator_freq;
}
void resetDevice() 
{
	/* Sends a reset command to the PCA9685 */
    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE1_RESTART);
    HAL_Delay(10);
}
void sleepPCA()
{
	/* Puts board into sleep mode */
  uint8_t awake = readRegister(PCA9685_MODE1_REG);
  uint8_t sleep = awake | PCA9685_MODE1_SLEEP; // set sleep bit high
  writeRegister(PCA9685_MODE1_REG, sleep);
  HAL_Delay(5); // wait until cycle ends for sleep to be active
}

void wakeupPCA()
{
	/* Wakes board from sleep */
	uint8_t sleep = readRegister(PCA9685_MODE1_REG);
	uint8_t wakeup = sleep & ~PCA9685_MODE1_SLEEP; // set sleep bit low
	writeRegister(PCA9685_MODE1_REG, wakeup);
}

void setPWMFrequency(float Frequency)
{
	/* Setter for the frequency, uses the prescaler calculation.
	 * This is why this function sets the approximate value.
	 * param Frequency(0-4095) to set desirable frequency
	 * */
	 if (Frequency < 1)
		 Frequency = 1;
	 if (Frequency > 3500)
		 Frequency = 3500; // Datasheet limit

	 float prescaleval = ((oscillator_freq / (Frequency * 4096.0)) + 0.5) - 1;;
	 if (prescaleval < 3)
		 prescaleval = 3;
	 if (prescaleval > 255)
		 prescaleval = 255;

	  uint8_t prescale = (uint8_t)prescaleval;
	  uint8_t oldmode = readRegister(PCA9685_MODE1_REG);
	  uint8_t newmode = (oldmode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP; // sleep
	  writeRegister(PCA9685_MODE1_REG, newmode);                             // go to sleep
	  writeRegister(PCA9685_PRESCALE_REG, prescale); // set the prescaler
	  writeRegister(PCA9685_MODE1_REG, oldmode);
	  HAL_Delay(5);
	  // This sets the MODE1 register to turn on auto increment.
	  writeRegister(PCA9685_MODE1_REG, oldmode | PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);

}
uint16_t getPWMFrequency()
{
	/* Getter for the frequency, that is set to the PCA9685
	 * Returns the approximate value.
	 * */
	uint8_t presc = readRegister(PCA9685_PRESCALE_REG); // set the prescaler
	uint16_t Frequency = (oscillator_freq/((float)(presc + 1))-0.5) / (float)4096;
	return Frequency;
}

void setLedPWM(uint8_t led, int value)
{
	/* PWM a single channel
	 * param led channel (1-16) to set PWM value for
	 * param on_value 0-4095 value to turn on the pulse
	 * Using setPWM() function.
	 * */
	setPWM(led, 0, value);
}

void setPWM(uint8_t led, int on_value, int off_value) 
{
	/* PWM a single channel
	 * param led channel (1-16) to set PWM value for
	 * param on_value 0-4095 value to turn on the pulse
	 * param off_value 0-4095 value to turn off the pulse
	 * */
	writeRegister(PCA9685_LED0_REG_ON_L + LED_MULTIPLYER * (led - 1), on_value & 0xFF);
	writeRegister(PCA9685_LED0_REG_ON_H + LED_MULTIPLYER * (led - 1), on_value >> 8);
	writeRegister(PCA9685_LED0_REG_OFF_L + LED_MULTIPLYER * (led - 1), off_value & 0xFF);
	writeRegister(PCA9685_LED0_REG_OFF_H + LED_MULTIPLYER * (led - 1), off_value >> 8);
}

void setAllLedPWM(int value)
{
	/* Sets PWM for all channels
	 * param on_value 0-4095 value to turn on the pulse
	 * */
	setAllPWM(0, value);
}

void setAllPWM(int on_value, int off_value) 
{
	/* Sets PWM for all channels
	 * param on_value 0-4095 value to turn on the pulse
	 * param off_value 0-4095 value to turn off the pulse
	 * */
	writeRegister(PCA9685_ALLLED_REG_ON_L, on_value & 0xFF);
	writeRegister(PCA9685_ALLLED_REG_ON_H, on_value >> 8);
	writeRegister(PCA9685_ALLLED_OFF_L, off_value & 0xFF);
	writeRegister(PCA9685_ALLLED_OFF_H, off_value >> 8);
}
uint16_t getPWM(uint8_t led)
{
	/* Getter for the PWM of a single channel value
	 * */
	int ledval = 0;
	ledval = readRegister(PCA9685_LED0_REG_OFF_H + LED_MULTIPLYER * (led-1));
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += readRegister(PCA9685_LED0_REG_OFF_L + LED_MULTIPLYER * (led-1));
	return ledval;
}
uint16_t getAllPWM()
{
	/* Getter for the PWM of all channels value
	 * */
	int ledval = 0;
	ledval = readRegister(PCA9685_ALLLED_OFF_H);
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += readRegister(PCA9685_ALLLED_OFF_L);
	return ledval;
}
void setLedOn(uint8_t led)
{
	/* Setter a single channel to the max value(on)
	 * */
	setPWM(led, 0, 4095);
}

void setLedOff(uint8_t led)
{
	/* Setter a single channel to the min value(off)
	 * */
	setPWM(led, 0, 0);
}

void setAllLedOn()
{
	/* Setter all channels to the max value(on)
	 * */
	setAllPWM(0, 4095);
}

void setAllLedOff()
{
	/* Setter all channels to the min value(off)
	 * */
	setAllPWM(0, 0);
}
