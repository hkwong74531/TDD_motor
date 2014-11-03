#ifndef D_InverterTest_H
#define D_InverterTest_H

#include <stdlib.h>
#include <stdint.h>
#include <memory.h>

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

void fake_inverter_setError(uint16_t* errAdd, uint16_t patt);
void fake_inverter_clearError(uint16_t* errAdd, uint16_t patt);
void fake_motor_setXDatum(uint8_t pos);
uint8_t fake_motor_getXDatum(void);
uint8_t motor_getXDatum(void);
void fake_motor_setZDatum(uint8_t pos);
uint8_t fake_motor_getZDatum(void);
uint8_t motor_getZDatum(void);
void fake_encoder_setXCnt(uint8_t cnt);
void fake_encoder_setZCnt(uint8_t cnt);
uint16_t getEncoderCount_X(void);
uint16_t getEncoderCount_Z(void);

void GPIO_SetBits(uint16_t* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(uint16_t* GPIOx, uint16_t GPIO_Pin);
uint8_t setSpeed(uint16_t speed, uint8_t channel);

#endif  /* D_InverterTest_H */