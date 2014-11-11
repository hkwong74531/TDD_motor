#include "motor.h"

typedef struct
{
	motor_t  base;
	uint16_t speed;
	uint8_t	 speed_flag;
	int16_t  position;
	int16_t  position_0;
	uint16_t int_cnt;
	uint8_t  wrong_dir;
	int16_t  pulse_N;
	int16_t  target;
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
	uint8_t  gate;
	uint8_t  tolerance;
	motorDirection_t direction;
	motorState_t state;
	motorState_t preState;
	motorEvent_t event;
	inverter_t* inverter;
	inverterInterface_t* inverterInterface;
	uint8_t (*getDatumSwitch)(void);
	uint16_t (*getEncoderCount)(void);
} motorData_t;

static void motor_start(motorData_t*, motorDirection_t);

static void motor_reset_state(motorData_t*);
static void motor_homing_state(motorData_t*);
static void motor_datum_state(motorData_t*);
static void motor_ready_state(motorData_t*);
static void motor_error_state(motorData_t*);

static void motor_homing_enter(motorData_t*);
static void motor_datum_enter(motorData_t*);
static void motor_ready_enter(motorData_t*);
static void motor_error_enter(motorData_t*);

void (*motor_state_function[MOTOR_DUMMY])(motorData_t*) = 
{
	motor_reset_state,
	motor_homing_state,
	motor_datum_state,
	motor_ready_state,
	motor_error_state,
	NULL,
};

/*
 *
 *		Private Interface
 *
 */

static void motor_reset_enter(motorData_t* motorData)
{
	motorState_t preState;

	preState = motorData->state;	
	memset(motorData, 0, sizeof(motorData_t));
	motorData->state = MOTOR_RESET;
	motorData->preState = preState;
	(motorData->inverterInterface)->turnOff(motorData->inverter);
}

static void motor_reset_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_INIT_EVENT:
		motor_homing_enter(motorData);
		break;
	case MOTOR_ERROR_EVENT:
		motor_error_enter(motorData);
		break;		
	case MOTOR_DATUM_EVENT:
	case MOTOR_READY_EVENT:
	case MOTOR_TO_TARGET_EVENT:
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
	case MOTOR_ERROR_EVENT:
		motor_error_enter(motorData);
		break;		
	case MOTOR_INIT_EVENT:
	case MOTOR_READY_EVENT:
	case MOTOR_TO_TARGET_EVENT:
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
	(motorData->inverterInterface)->turnOn(motorData->inverter);
	(motorData->inverterInterface)->dirPositive(motorData->inverter);	
}

static void motor_datum_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_DATUM_EVENT:	
		motor_ready_enter(motorData);
		break;
	case MOTOR_ERROR_EVENT:
		motor_error_enter(motorData);
		break;
	case MOTOR_READY_EVENT:
	case MOTOR_INIT_EVENT:
	case MOTOR_TO_TARGET_EVENT:
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

static void motor_ready_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_INIT_EVENT:
		motorData->event = MOTOR_DUMMY_EVENT;
		motor_homing_enter(motorData);
		break;
	case MOTOR_TO_TARGET_EVENT:
		motorData->event = MOTOR_DUMMY_EVENT;
		if(motorData->position >= motorData->target + motorData->tolerance)
		{
			motor_start(motorData, MOTOR_NEGDIR);
		}
		else if(motorData->position <= motorData->target - motorData->tolerance)
		{
			motor_start(motorData, MOTOR_POSDIR);
		}	
		break;
	case MOTOR_ERROR_EVENT:
		motorData->event = MOTOR_DUMMY_EVENT;
		motor_error_enter(motorData);
		break;
	case MOTOR_CYCLE_EVENT:
		motorData->event = MOTOR_DUMMY_EVENT;
//		motorZ_cycle_enter();
		break;
	case MOTOR_DATUM_EVENT:
	case MOTOR_READY_EVENT:
	case MOTOR_RESUME_EVENT:
		motorData->event = MOTOR_DUMMY_EVENT;	
		break;
	case MOTOR_DUMMY_EVENT:				
	default:
		motorData->event = MOTOR_DUMMY_EVENT;		
		if(motorData->direction == MOTOR_STOP)
		{
			if(motorData->position > motorData->target + motorData->tolerance)
			{
				motorData->position_0 = motorData->position;
				motor_setSpeed((motor_t*)motorData, 30);
				motor_start(motorData, MOTOR_NEGDIR);
			}
			else if(motorData->position < motorData->target - motorData->tolerance)
			{
				motorData->position_0 = motorData->position;
				motor_setSpeed((motor_t*)motorData, 30);				
				motor_start(motorData, MOTOR_POSDIR);
			}		
		}		
		break;
	}
}

static void motor_error_enter(motorData_t* motorData)
{
	motorData->preState = motorData->state;
	motorData->state = MOTOR_ERROR;
	(motorData->inverterInterface)->turnOff(motorData->inverter);	
}

static void motor_error_state(motorData_t* motorData)
{
	switch(motorData->event)
	{
	case MOTOR_RESUME_EVENT:
		switch(motorData->preState)
		{
		case MOTOR_RESET:
			motor_reset_enter(motorData);
			break;
		case MOTOR_HOMING:
			motor_homing_enter(motorData);
			break;
		case MOTOR_DATUM:
			motor_datum_enter(motorData);
			break;
		case MOTOR_READY:
			motor_ready_enter(motorData);
			break;
		case MOTOR_CYCLE:
//			motor_cycle_enter(motorData);
			break;
		case MOTOR_ERROR:
		case MOTOR_DUMMY:
		default:
			break;
		}
		break;
	case MOTOR_ERROR_EVENT:
		break;
	case MOTOR_INIT_EVENT:
		switch(motorData->preState)
		{
		case MOTOR_RESET:
		case MOTOR_READY:
		case MOTOR_CYCLE:
			motorData->preState = MOTOR_HOMING;
			break;
		case MOTOR_HOMING:
		case MOTOR_DATUM:
		case MOTOR_ERROR:
		case MOTOR_DUMMY:
		default:
			break;
		}
		break;
	case MOTOR_TO_TARGET_EVENT:
	case MOTOR_DATUM_EVENT:
	case MOTOR_READY_EVENT:
	case MOTOR_CYCLE_EVENT:
	case MOTOR_DUMMY_EVENT:
	default:
		break;
	}

	motorData->event = MOTOR_DUMMY_EVENT;
}

static void motor_start(motorData_t* motorData, motorDirection_t dir)
{
	if(dir != MOTOR_STOP)
	{
		motorData->direction = dir;
		motorData->int_cnt = 0;
		if(dir == MOTOR_POSDIR)
		{
			(motorData->inverterInterface)->dirPositive(motorData->inverter);
		}
		else
		{
			(motorData->inverterInterface)->dirNegative(motorData->inverter);
		}
		(motorData->inverterInterface)->turnOn(motorData->inverter);
	}	
}

/*
 *
 *		Public Interface
 *
 */
motor_t* motor_create(inverterInterface_t* inverterInterface, inverter_t* inverter, uint8_t (*getDatumSwitch)(void), uint16_t (*getEncoderCount)(void))
{
	motorData_t* motorData;
	
	motorData = malloc(sizeof(motorData_t));
	memset(motorData, 0, sizeof(motorData_t));
	motorData->inverter = inverter;
	motorData->inverterInterface = inverterInterface;
	motorData->getDatumSwitch = getDatumSwitch;
	motorData->getEncoderCount = getEncoderCount;
	motorData->event = MOTOR_DUMMY_EVENT;
	
	return (motor_t*)motorData;
}

void motor_setTolerance(motor_t* motor, uint8_t tolerance)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	motorData->tolerance = tolerance;
}

motorState_t motor_getState(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	
	return motorData->state;
}

void motor_init(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);

	motorData->event = MOTOR_INIT_EVENT;
}

void motor_stop(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	(motorData->inverterInterface)->turnOff(motorData->inverter);
	motorData->direction = MOTOR_STOP;
}

void motor_brake(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	(motorData->inverterInterface)->turnOff(motorData->inverter);
}

void motor_setSpeed(motor_t* motor, uint16_t speed)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);

	if(speed != motorData->speed)
	{
		motorData->speed_flag = 1;
		motorData->speed = speed;
	}
}

uint16_t motor_getSpeed(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);

	return (motorData->speed); 
}

void motor_setTarget(motor_t* motor, int16_t target)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	motorData->target = target;
	motorData->event = MOTOR_TO_TARGET_EVENT;		
}

motorDirection_t motor_getDirection(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	return (motorData->direction);
}

int16_t motor_getPosition(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);

	return motorData->position;
}

uint8_t motor_checkPosition(motor_t* motor)
{	
	motorData_t* motorData;
	int16_t position, target;

	motorData = (motorData_t*)(motor);
	position = motorData->position;
	target = motorData->target;

	if((position > (target - motorData->tolerance)) && (position < (target + motorData->tolerance)))
	{
		return 1;
	}
	
	return 0;
}

void motor_setGate(motor_t* motor, uint8_t gate)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	if(gate == 1 || gate == 0)
	{
		motorData->gate = gate;
	}
}

void motor_state_int(motor_t* motor)
{
	motorData_t* motorData;
	uint16_t pulse_N0;
	int16_t  delta_N;
	
	motorData = (motorData_t*)(motor);
#if 0
	pulse_XN0 = pulse_XN;
    ENC_Get_Electrical_Angle();
	if(pulse_XN - pulse_XN0 > 640)	// step reverse and cross 0. actually -ve
	{
		delta_XN = pulse_XN - pulse_XN0 - 1024;
	}
	else if(pulse_XN0 - pulse_XN > 640) // step forward and cross 1023. actually +ve
	{
		delta_XN = pulse_XN - pulse_XN0 + 1024;
	}
	else
	{
		delta_XN = pulse_XN - pulse_XN0;
	}
#endif
    pulse_N0 = motorData->pulse_N;
    motorData->pulse_N  = motorData->getEncoderCount();
    delta_N =  motorData->pulse_N - pulse_N0;
    if(delta_N < -500)
    {
    	delta_N = 1000 + delta_N;
    }
    else if (delta_N>500)
    {
//    	delta_N = 1000 -2 - delta_N;
		delta_N = delta_N - 998;
    }
    motorData->position = motorData->position + delta_N;	

	if(motorData->direction != MOTOR_STOP && motorData->state != MOTOR_ERROR)
	{
		if(motorData->int_cnt >= WRONG_DIRECTION_TIMER_CNT)
		{
			motorData->int_cnt = 0;
			if(motorData->direction == MOTOR_POSDIR)
			{
				if(motorData->position <= motorData->position_0)
				{
					motorData->wrong_dir = 1;
				}
			}
			else if(motorData->direction == MOTOR_NEGDIR)
			{
				if(motorData->position >= motorData->position_0)
				{
					motorData->wrong_dir = 1;
				}
			}
		}
		if(motorData->int_cnt == 0)
		{
			motorData->position_0 = motorData->position;
		}
		motorData->int_cnt++;
	}
	else
	{
		motorData->int_cnt = 0;
	}
		
	
	if((motorData->gate == 1) || ((motorData->inverterInterface)->getErrorStatus(motorData->inverter)) == 1 || (motorData->wrong_dir == 1))
	{
		(motorData->inverterInterface)->turnOff(motorData->inverter);
		motorData->int_cnt = 0;
		motorData->event = MOTOR_ERROR_EVENT;
	}
	else
	{
		switch(motorData->state)
		{
			static uint8_t position_switch;
	
			case MOTOR_HOMING:
				position_switch = motorData->getDatumSwitch();
				if (position_switch == 0)
				{
					motorData->int_cnt = 0;
					(motorData->inverterInterface)->dirPositive(motorData->inverter);
					motorData->event = MOTOR_DATUM_EVENT;
				}			
				break;
			case MOTOR_DATUM:
				position_switch = motorData->getDatumSwitch();
				if (position_switch == 1)
				{
					motorData->int_cnt = 0;
					motorData->position = 0;
					(motorData->inverterInterface)->turnOff(motorData->inverter);
					motorData->direction = MOTOR_STOP;
					motorData->event = MOTOR_DATUM_EVENT;
				}			
				break;
			case MOTOR_READY:
				if(motorData->direction == MOTOR_POSDIR)
				{
					if(motorData->position >= motorData->target - motorData->tolerance)
					{
						motorData->int_cnt = 0;
						(motorData->inverterInterface)->turnOff(motorData->inverter);
						motorData->direction = MOTOR_STOP;
					}
				}
				else if(motorData->direction == MOTOR_NEGDIR)
				{
					if(motorData->position <= motorData->target + motorData->tolerance)
					{
						motorData->int_cnt = 0;
						(motorData->inverterInterface)->turnOff(motorData->inverter);
						motorData->direction = MOTOR_STOP;
					}
				}
				break;
			case MOTOR_ERROR:
				motorData->event = MOTOR_RESUME_EVENT;
				break;
			case MOTOR_RESET:
			case MOTOR_CYCLE:
			case MOTOR_DUMMY:
				break;
			default:
				break;
		}
	}
}

void motor_state_process(motor_t* motor)
{
	motorData_t* motorData;
	
	motorData = (motorData_t*)(motor);
	(*motor_state_function[(motorData->state)])(motorData);
	
	if(motorData->speed_flag == 1)
	{
		if(1 == ((motorData->inverterInterface)->setSpeed(motorData->inverter, motorData->speed)))
		{
			motorData->speed_flag = 0;		
		}
	}
}