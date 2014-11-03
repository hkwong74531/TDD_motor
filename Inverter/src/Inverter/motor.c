#include "motor.h"

typedef struct
{
//	uint16_t speed;
	int16_t  position;
	int16_t  pulse_N;
//	int16_t  target;
//	int16_t  max;
//	int16_t  down[2];
//	int16_t  up[2];
//	uint16_t dwellDown[3];
//	uint16_t dwellUp[3];
//	int16_t  min;
//	uint16_t speedDown[3];
//	uint16_t speedUp[3];
//	uint8_t  segment;
	uint8_t	 current_sg;
	motorDirection_t direction;
	motorState_t state;
	motorState_t preState;
	motorEvent_t event;
	void* inverter;
	inverterInterface_t* inverterInterface;
	uint8_t (*getDatumSwitch)(void);
	uint16_t (*getEncoderCount)(void);
} motorData_t;

static void motor_reset_state(motorData_t*);
static void motor_homing_state(motorData_t*);
static void motor_datum_state(motorData_t*);

static void motor_homing_enter(motorData_t*);
static void motor_datum_enter(motorData_t*);
static void motor_ready_enter(motorData_t*);

void (*motor_state_function[MOTOR_DUMMY])(motorData_t*) = 
{
	motor_reset_state,
	motor_homing_state,
	motor_datum_state,
	NULL,
	NULL,
	NULL,
};
#if 0
static void motor_reset_enter(void* motor)
{
	motorData_t* motorData;
	motorState_t preState;
	
	motorData = (motorData_t*)(motor);

	preState = motorData->state;	
	memset(motorData, 0, sizeof(motorData_t));
	motorData->state = MOTOR_RESET;
	motorData->preState = preState;
	(motorData->inverterInterface)->turnOff(motorData->inverter);
}
#endif

static void motor_reset_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_INIT_EVENT:
		motor_homing_enter(motorData);
		break;
	case MOTOR_DATUM_EVENT:
	case MOTOR_READY_EVENT:
	case MOTOR_TO_TARGET_EVENT:
	case MOTOR_ERROR_EVENT:
	case MOTOR_RESUME_EVENT:
	case MOTOR_CYCLE_EVENT:
	case MOTOR_DUMMY_EVENT:		
	default:
		break;
	}

	motorData->event = MOTOR_DUMMY_EVENT;
}

static void motor_homing_enter(motorData_t* motorData)
{
	motorData->preState = motorData->state;
	motorData->state = MOTOR_HOMING;
	motorData->direction = MOTOR_NEGDIR;
	(motorData->inverterInterface)->turnOn(motorData->inverter);
	(motorData->inverterInterface)->dirNegative(motorData->inverter);
}

static void motor_homing_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_DATUM_EVENT:
		motor_datum_enter(motorData);
		break;
	case MOTOR_INIT_EVENT:
	case MOTOR_READY_EVENT:
	case MOTOR_TO_TARGET_EVENT:
	case MOTOR_ERROR_EVENT:
	case MOTOR_RESUME_EVENT:
	case MOTOR_CYCLE_EVENT:
	case MOTOR_DUMMY_EVENT:		
	default:
		break;
	}

	motorData->event = MOTOR_DUMMY_EVENT;
}

static void motor_datum_enter(motorData_t* motorData)
{
	motorData->preState = motorData->state;
	motorData->state = MOTOR_DATUM;
	motorData->direction = MOTOR_POSDIR;
}

static void motor_datum_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_READY_EVENT:	
		motor_ready_enter(motorData);
		break;
	case MOTOR_DATUM_EVENT:
	case MOTOR_INIT_EVENT:
	case MOTOR_TO_TARGET_EVENT:
	case MOTOR_ERROR_EVENT:
	case MOTOR_RESUME_EVENT:
	case MOTOR_CYCLE_EVENT:
	case MOTOR_DUMMY_EVENT:		
	default:
		break;
	}

	motorData->event = MOTOR_DUMMY_EVENT;
}

static void motor_ready_enter(motorData_t* motorData)
{
	motorData->preState = motorData->state;
	motorData->state = MOTOR_READY;
	
	motorData->current_sg = 0;
	if(motorData->direction == MOTOR_POSDIR)
	{
		(motorData->inverterInterface)->turnOn(motorData->inverter);
		(motorData->inverterInterface)->dirPositive(motorData->inverter);
	}
	else if(motorData->direction == MOTOR_NEGDIR)
	{
		(motorData->inverterInterface)->turnOn(motorData->inverter);
		(motorData->inverterInterface)->dirNegative(motorData->inverter);;
	}
	else
	{
		(motorData->inverterInterface)->turnOff(motorData->inverter);
	}	
}





void* motor_create(inverterInterface_t* inverterInterface, void* inverter, uint8_t (*getDatumSwitch)(void), uint16_t (*getEncoderCount)(void))
{
	motorData_t* motorData;
	
	motorData = malloc(sizeof(motorData_t));
	memset(motorData, 0, sizeof(motorData_t));
	motorData->inverter = inverter;
	motorData->inverterInterface = inverterInterface;
	motorData->getDatumSwitch = getDatumSwitch;
	motorData->getEncoderCount = getEncoderCount;
	motorData->event = MOTOR_DUMMY_EVENT;
	
	return (void*)motorData;
}

motorState_t motor_getState(void* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	
	return motorData->state;
}

void motor_init(void* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);

	motorData->event = MOTOR_INIT_EVENT;
}

void motor_brake(void* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	(motorData->inverterInterface)->turnOff(motorData->inverter);
}

void motor_stop(void* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	(motorData->inverterInterface)->turnOff(motorData->inverter);
	motorData->direction = MOTOR_STOP;
}

void motor_start(void* motor, motorDirection_t dir)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	motorData->direction = dir;
}

void motor_state_int(void* motor)
{
	motorData_t* motorData;
	uint16_t delta_N, pulse_N0;
	
	motorData = (motorData_t*)(motor);
	
    pulse_N0 = motorData->pulse_N;
    motorData->pulse_N  = motorData->getEncoderCount();
    delta_N =  motorData->pulse_N - pulse_N0;
    if(delta_N < -500)
    {
    	delta_N = 1000 + delta_N;
    }
    else if (delta_N>500)
    {
    	delta_N = 1000 -2 - delta_N;
    }
    motorData->position = motorData->position + delta_N;	
	
	switch(motorData->state)
	{
		static uint8_t position_switch;
	
		case MOTOR_HOMING:
			position_switch = motorData->getDatumSwitch();
			if (position_switch == 0)
			{
//				motorZ_int_cnt = 0;
				(motorData->inverterInterface)->dirPositive(motorData->inverter);
				motorData->event = MOTOR_DATUM_EVENT;
			}			
			break;
		case MOTOR_DATUM:
			position_switch = motorData->getDatumSwitch();
			if (position_switch == 1)
			{
//				motorZ_int_cnt = 0;
				motorData->position = 0;
				(motorData->inverterInterface)->turnOff(motorData->inverter);
//				motorZ_setSpeed(0);
				motorData->direction = MOTOR_STOP;
				motorData->event = MOTOR_READY_EVENT;
			}			
			break;
		case MOTOR_READY:
		
			break;
		case MOTOR_RESET:
		case MOTOR_ERROR:
		case MOTOR_CYCLE:
		case MOTOR_DUMMY:
			break;
		default:
			break;
	}
}

void motor_state_process(void* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	(*motor_state_function[(motorData->state)])(motorData);
}