//- ------------------------------------------------------------------
//-    Copyright (c) James W. Grenning -- All Rights Reserved         
//-    For use by owners of Test-Driven Development for Embedded C,   
//-    and attendees of Renaissance Software Consulting, Co. training 
//-    classes.                                                       
//-                                                                   
//-    Available at http://pragprog.com/titles/jgade/                 
//-        ISBN 1-934356-62-X, ISBN13 978-1-934356-62-3               
//-                                                                   
//-    Authorized users may use this source code in your own          
//-    projects, however the source code may not be used to           
//-    create training material, courses, books, articles, and        
//-    the like. We make no guarantees that this source code is       
//-    fit for any purpose.                                           
//-                                                                   
//-    www.renaissancesoftware.net james@renaissancesoftware.net      
//- ------------------------------------------------------------------

extern "C"
{
#include "motor.h"
#include "inverter_spy.h"
#include "inverter_test.h"
}

#include "CppUTest/TestHarness.h"

// helper variables
int16_t steps;
int16_t encoder_loop;

// X
uint16_t GPIOD = 0xFFFF;	//SW
uint16_t GPIOE = 0xFFFB;	//DIR
// Z
uint16_t GPIOC = 0xDFFF;	//SW
uint16_t GPIOA = 0x0000;	//DIR

inverterInterface_t inverterInterface;

inverter_t *inverter_X, *inverter_Z;
motor_t *motor_X, *motor_Z;

TEST_GROUP(MotorDriver)
{
	void check_onOff_X(uint8_t onOff)
	{
		if(onOff == 0)
		{
			LONGS_EQUAL(0x0000, GPIOD & 0x8000);
		}
		else if(onOff == 1)
		{
			LONGS_EQUAL(0x8000, GPIOD & 0x8000);
		}
		else
		{
			FAIL("Wrong input to check X on/off");		
		}
	}

	void check_onOff_Z(uint8_t onOff)
	{
		if(onOff == 0)
		{
			LONGS_EQUAL(0x0000, GPIOC & 0x0200);
		}
		else if(onOff == 1)
		{
			LONGS_EQUAL(0x0200, GPIOC & 0x0200);
		}
		else
		{
			FAIL("Wrong input to check Z on/off");		
		}		
	}
	
	void check_direction_X(int8_t dir)
	{
		if(dir > 0)
		{
			LONGS_EQUAL(0xFFFB & ~(0x4000), GPIOE);
		}
		else if(dir < 0)
		{
			LONGS_EQUAL(0xFFFB | (0x4000), GPIOE);
		}
		else
		{
			FAIL("Wrong input to check X direction");		
		}		
	}

	void check_direction_Z(int8_t dir)
	{
		if(dir > 0)
		{
			LONGS_EQUAL(0x0000 | (0x0004), GPIOA);
		}
		else if(dir < 0)
		{
			LONGS_EQUAL(0x0000 & ~(0x0004), GPIOA);	
		}
		else
		{
			FAIL("Wrong input to check Z direction");		
		}		
	}
	
	void ready_enter()
	{
		motor_init(motor_X);
		motor_init(motor_Z);
		motor_state_int(motor_X);
		motor_state_int(motor_Z);	
		motor_state_process(motor_X);
		motor_state_process(motor_Z);

		fake_encoder_setXCnt(770);
		fake_motor_setXDatum(0);	
		fake_motor_setZDatum(0);	
		motor_state_int(motor_X);
		motor_state_int(motor_Z);	
		motor_state_process(motor_X);
		motor_state_process(motor_Z);

		fake_encoder_setXCnt(780);		
		fake_motor_setXDatum(1);
		fake_motor_setZDatum(1);
		motor_state_int(motor_X);
		motor_state_int(motor_Z);	
		motor_state_process(motor_X);
		motor_state_process(motor_Z);
	}
	
    void setup()
    {
		steps = 0;
		encoder_loop = 0;
	
		GPIOE = 0xFFFB;	//DIR
		GPIOC = 0xDFFF;	//SW
		GPIOD = 0xFFFF;
		GPIOA = 0x0000;	
		memset(I2C1_Buffer1_Tx, 0, 8);
		inverter_X = inverterSpy_init(&inverterInterface, &GPIOD, 0x8000, &GPIOE, 0x4000, 0, &GPIOE, 0x0004, 1);
		inverter_Z = inverterSpy_init(&inverterInterface, &GPIOC, 0x0200, &GPIOA, 0x0004, 1, &GPIOC, 0x2000, 2);
		motor_X = motor_create(&inverterInterface, inverter_X, motor_getXDatum, getEncoderCount_X);
		motor_Z = motor_create(&inverterInterface, inverter_Z, motor_getZDatum, getEncoderCount_Z);
		motor_setTolerance(motor_X, 15);
		motor_setTolerance(motor_Z, 30);
    }

    void teardown()
    {
       free(inverter_X);
	   free(inverter_Z);
	   free(motor_X);
	   free(motor_Z);
    }
};

TEST(MotorDriver, motorCreate)
{
	LONGS_EQUAL(0x0000, GPIOC & 0x0200);
	LONGS_EQUAL(MOTOR_RESET, motor_getState(motor_Z));
	LONGS_EQUAL(0x0000, GPIOD & 0x8000);
	LONGS_EQUAL(MOTOR_RESET, motor_getState(motor_X));	
//  FAIL("Start here");
}

IGNORE_TEST(MotorDriver, motorInit)
{
	motor_init(motor_Z);
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	check_onOff_X(0);
	check_onOff_Z(1);
	check_direction_Z(-1);
	LONGS_EQUAL(MOTOR_HOMING, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_RESET, motor_getState(motor_X));	
	motor_init(motor_X);
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	check_onOff_X(1);
	check_onOff_Z(1);
	check_direction_X(-1);
	LONGS_EQUAL(MOTOR_HOMING, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_HOMING, motor_getState(motor_X));	
}

IGNORE_TEST(MotorDriver, motorDatum)
{
	motor_init(motor_X);
	motor_init(motor_Z);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);

	fake_motor_setZDatum(0);	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	LONGS_EQUAL(MOTOR_DATUM, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_HOMING, motor_getState(motor_X));	
	check_onOff_X(1);
	check_onOff_Z(1);
	check_direction_X(-1);
	check_direction_Z(1);
	
	fake_motor_setXDatum(0);	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	LONGS_EQUAL(MOTOR_DATUM, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_DATUM, motor_getState(motor_X));	
	check_onOff_X(1);
	check_onOff_Z(1);
	check_direction_X(1);
	check_direction_Z(1);	
}

IGNORE_TEST(MotorDriver, motorReady)
{
	motor_init(motor_X);
	motor_init(motor_Z);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);

	fake_motor_setZDatum(0);	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	
	fake_motor_setXDatum(0);	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);

	fake_motor_setZDatum(1);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);
	motor_state_process(motor_Z);

	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_DATUM, motor_getState(motor_X));	
	check_onOff_X(1);
	check_onOff_Z(0);
	check_direction_X(1);

	fake_motor_setXDatum(1);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_X));	
	check_onOff_X(0);
	check_onOff_Z(0);	
}

TEST(MotorDriver, motorToPosTarget)
{
	ready_enter();

	motor_setTarget(motor_X, 1024);
	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	BYTES_EQUAL(MOTOR_POSDIR, motor_getDirection(motor_X));	
	BYTES_EQUAL(MOTOR_STOP, motor_getDirection(motor_Z));		
	check_onOff_X(1);
	check_onOff_Z(0);	
	check_direction_X(1);
}

TEST(MotorDriver, motorArrivePosTarget)
{
	ready_enter();

	motor_setTarget(motor_X, 290);

	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);
		

	encoder_loop = 780;

	while(motor_checkPosition(motor_X)==0)
	{
		steps += 3;
		encoder_loop += 3;		
		if(encoder_loop > 1000)
		{
			encoder_loop -= 1000;
		}
		fake_encoder_setXCnt(encoder_loop);

		motor_state_int(motor_X);
		motor_state_int(motor_Z);	
		motor_state_process(motor_X);	
		motor_state_process(motor_Z);
	}

	LONGS_EQUAL(291, motor_getPosition(motor_X)+15);
	LONGS_EQUAL(0, motor_getPosition(motor_Z));
#if 1	
	BYTES_EQUAL(MOTOR_STOP, motor_getDirection(motor_X));	
	BYTES_EQUAL(MOTOR_STOP, motor_getDirection(motor_Z));		
	check_onOff_X(0);
	check_onOff_Z(0);	
	check_direction_X(1);
#endif	
}

TEST(MotorDriver, motorArriveNegTarget)
{
	ready_enter();

	motor_setTarget(motor_Z, -373);

	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);
		
	BYTES_EQUAL(MOTOR_NEGDIR, motor_getDirection(motor_Z));
		
	encoder_loop = 780;

	while(motor_checkPosition(motor_Z)==0)
	{
		steps -= 2;
		encoder_loop -= 2;		
		if(encoder_loop < 0)
		{
			encoder_loop += 1000;
		}
		fake_encoder_setZCnt(encoder_loop);

		motor_state_int(motor_X);
		motor_state_int(motor_Z);	
		motor_state_process(motor_X);	
		motor_state_process(motor_Z);
	}

	LONGS_EQUAL(0, motor_getPosition(motor_X));
	LONGS_EQUAL(-373+29, motor_getPosition(motor_Z));
#if 1	
	BYTES_EQUAL(MOTOR_STOP, motor_getDirection(motor_X));	
	BYTES_EQUAL(MOTOR_STOP, motor_getDirection(motor_Z));		
	check_onOff_X(0);
	check_onOff_Z(0);	
	check_direction_Z(-1);
#endif	
}

TEST(MotorDriver, motorToNegTarget)
{
	ready_enter();

	motor_setTarget(motor_Z, -1024);
	motor_setSpeed(motor_Z, 0x824);
	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	BYTES_EQUAL(MOTOR_NEGDIR, motor_getDirection(motor_Z));	
	BYTES_EQUAL(MOTOR_STOP, motor_getDirection(motor_X));		
	check_onOff_X(0);
	check_onOff_Z(1);	
	check_direction_Z(-1);
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[1]);
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[2]);	
	LONGS_EQUAL(0x08, I2C1_Buffer1_Tx[3]);
	LONGS_EQUAL(0x24, I2C1_Buffer1_Tx[4]);	
}

TEST(MotorDriver, motorSetError_Resume)
{
	ready_enter();

	motor_setTarget(motor_X, 1024);
	motor_setSpeed(motor_X, 0x824);
	motor_setTarget(motor_Z, -1024);
	motor_setSpeed(motor_Z, 0x248);
	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);
	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	check_onOff_X(1);
	check_onOff_Z(1);
//	fake_inverter_setError(&GPIOE, 0x0004);	
	motor_setGate(motor_Z, 1);
	
	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	check_onOff_X(1);
	check_onOff_Z(0);
	LONGS_EQUAL(MOTOR_ERROR, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_X));		

	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);
	
//	fake_inverter_clearError(&GPIOE, 0x0004);
	motor_setGate(motor_Z, 0);

	motor_state_int(motor_X);
	motor_state_int(motor_Z);	
	motor_state_process(motor_X);	
	motor_state_process(motor_Z);

	check_onOff_X(1);
	check_onOff_Z(1);
	check_direction_X(1);
	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_X));			
}

TEST(MotorDriver, motorWrongDirection)
{
	uint16_t i;

	ready_enter();

	motor_setTarget(motor_X, 290);

	for(i = 0; i < 2+WRONG_DIRECTION_TIMER_CNT; i++)
	{
		motor_state_int(motor_X);
		motor_state_int(motor_Z);	
		motor_state_process(motor_X);	
		motor_state_process(motor_Z);
	}

	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_Z));
	LONGS_EQUAL(MOTOR_ERROR, motor_getState(motor_X));			
	check_onOff_X(0);
	check_onOff_Z(0);	
}

#if 0
TEST(MotorDriver, motorInit)
{
	motor_init(motor_X);
	motor_init(motor_Z);
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);

	LONGS_EQUAL(0x0200, GPIOC & 0x0200);	// motor on?
	LONGS_EQUAL(0x0000, GPIOA & 0x0004);	// -ve dir?
	LONGS_EQUAL(MOTOR_HOMING, motor_getState(motor_Z));	// motor state?
	
	LONGS_EQUAL(0x8000, GPIOD & 0x8000);	// motor on?
	LONGS_EQUAL(0x4000, GPIOE & 0x4000);	// -ve dir?
	LONGS_EQUAL(MOTOR_HOMING, motor_getState(motor_X));	// motor state?
	
	fake_motor_setZDatum(0);
	fake_motor_setXDatum(0);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);
	motor_state_process(motor_X);
	motor_state_process(motor_Z);
	
	LONGS_EQUAL(0x0200, GPIOC & 0x0200);	// motor on?
	LONGS_EQUAL(0x0004, GPIOA & 0x0004);	// +ve dir?
	LONGS_EQUAL(MOTOR_DATUM, motor_getState(motor_Z));
	
	LONGS_EQUAL(0x8000, GPIOD & 0x8000);	// motor on?
	LONGS_EQUAL(0x0000, GPIOE & 0x4000);	// +ve dir?
	LONGS_EQUAL(MOTOR_DATUM, motor_getState(motor_X));	// motor state?

	fake_motor_setZDatum(1);
	fake_motor_setXDatum(1);
	motor_state_int(motor_X);
	motor_state_int(motor_Z);
	motor_state_process(motor_X);
	motor_state_process(motor_Z);	

	LONGS_EQUAL(0x0000, GPIOC & 0x0200);	// motor off?
	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_Z));

	LONGS_EQUAL(0x0000, GPIOD & 0x8000);	// motor off?
	LONGS_EQUAL(MOTOR_READY, motor_getState(motor_X));	// motor state?
	
}
#endif
