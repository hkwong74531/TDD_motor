#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdlib.h>
#include <stdint.h>
#include <memory.h>

#include "inverter.h"

typedef enum
{
	MOTOR_STOP = 0,
	MOTOR_POSDIR,
	MOTOR_NEGDIR,
} motorDirection_t;

typedef enum
{
	MOTOR_RESET = 0,
	MOTOR_HOMING,
	MOTOR_DATUM,
	MOTOR_READY,
	MOTOR_ERROR,
	MOTOR_CYCLE,
	MOTOR_DUMMY,
} motorState_t;

typedef enum
{
	MOTOR_INIT_EVENT = 0,
	MOTOR_DATUM_EVENT,
	MOTOR_READY_EVENT,
	MOTOR_TO_TARGET_EVENT,
	MOTOR_ERROR_EVENT,
	MOTOR_RESUME_EVENT,
	MOTOR_CYCLE_EVENT,
	MOTOR_DUMMY_EVENT,
} motorEvent_t;

typedef void* motorData;

motorData motor_create(inverterInterface_t*, motorData, uint8_t (*)(void), uint16_t (*)(void));

motorState_t motor_getState(motorData);
void motor_init(motorData);
void motor_stop(motorData);

void motor_state_int(motorData);
void motor_state_process(motorData);

#endif