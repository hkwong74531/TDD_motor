#include "inverter_test.h"

static uint8_t I2C1_Buffer1_Tx[8];
static uint8_t motor_datum_switch_X = 1;
static uint8_t motor_datum_switch_Z = 1;
static uint16_t encoder_count_X = 0;
static uint16_t encoder_count_Z = 0;

void fake_inverter_setError(uint16_t* errAdd, uint16_t patt)
{
	*(errAdd) |= patt;
}

void fake_inverter_clearError(uint16_t* errAdd, uint16_t patt)
{
	*(errAdd) &= ~patt;
}

void fake_motor_setXDatum(uint8_t pos)
{
	motor_datum_switch_X = pos;
}

void fake_motor_setZDatum(uint8_t pos)
{
	motor_datum_switch_Z = pos;
}

uint8_t fake_motor_getXDatum(void)
{
	return motor_datum_switch_X;
}

uint8_t fake_motor_getZDatum(void)
{
	return motor_datum_switch_Z;
}

uint8_t motor_getXDatum(void)
{
	return (fake_motor_getXDatum());
}

uint8_t motor_getZDatum(void)
{
	return (fake_motor_getZDatum());
}

void fake_encoder_setXCnt(uint8_t cnt)
{
	encoder_count_X = cnt;
}

void fake_encoder_setZCnt(uint8_t cnt)
{
	encoder_count_Z = cnt;
}

uint16_t getEncoderCount_X(void)
{
	return encoder_count_X;
}

uint16_t getEncoderCount_Z(void)
{
	return encoder_count_Z;
}
	
void GPIO_SetBits(uint16_t* GPIOx, uint16_t GPIO_Pin)
{
	*(GPIOx) |= GPIO_Pin;
}
	
void GPIO_ResetBits(uint16_t* GPIOx, uint16_t GPIO_Pin)
{
	*(GPIOx) &= ~GPIO_Pin;
}

uint8_t setSpeed(uint16_t speed, uint8_t channel)
{
	if((speed <= 0xFFF) && (channel == 1 || channel == 2))
	{
		I2C1_Buffer1_Tx[0] = 0x50;
	
		I2C1_Buffer1_Tx[channel + 2] = speed;
		I2C1_Buffer1_Tx[channel + 1] = 0x0F & (speed>>8);

		if(1)	// mocking I2C success
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
}