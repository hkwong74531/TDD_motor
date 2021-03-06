#ifndef MOTOR_H_
#define MOTOR_H_

#define	_CPPUTEST_

#include <stdlib.h>
#include <stdint.h>
#ifdef _CPPUTEST_
#include <memory.h>
#else
#include <string.h>
#endif

#include "inverter.h"

#define WRONG_DIRECTION_TIMER_CNT 10000
#define POSITION_TOLERANCE_DEFAULT  15

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

typedef void* motor_t;

motor_t motor_create(inverterInterface_t*, inverter_t, uint8_t (*)(void), uint16_t (*)(void));

void motor_setTolerance(motor_t, uint8_t);
motorState_t motor_getState(motor_t);
void motor_init(motor_t);
void motor_stop(motor_t);
void motor_brake(motor_t);

void motor_toReady(motor_t);
void motor_toCycle(motor_t);

void motor_setMax(motor_t, int16_t);
void motor_setMin(motor_t, int16_t);
void motor_setSegment(motor_t, uint8_t);
void motor_setDown(motor_t, int16_t, uint8_t);
void motor_setSpeedDown(motor_t, uint16_t, uint8_t);
void motor_setDwellDown(motor_t, uint16_t, uint8_t);
void motor_setUp(motor_t, int16_t, uint8_t);
void motor_setSpeedUp(motor_t, uint16_t, uint8_t);
void motor_setDwellUp(motor_t, uint16_t, uint8_t);

void motor_setSpeed(motor_t, uint16_t);
uint16_t motor_getSpeed(motor_t);
int16_t motor_getPosition(motor_t);
void motor_setTarget(motor_t, int16_t);
uint8_t motor_checkPosition(motor_t);
motorDirection_t motor_getDirection(motor_t);
void motor_setGate(motor_t, uint8_t);

void motor_state_int(motor_t);
void motor_state_process(motor_t);

#endif