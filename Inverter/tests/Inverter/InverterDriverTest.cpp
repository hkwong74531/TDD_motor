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

// X
uint16_t GPIOD = 0xFFFF;	//SW
uint16_t GPIOE = 0xFFFF;	//DIR
// Z
uint16_t GPIOC = 0xFFFF;	//SW
uint16_t GPIOA = 0xFFFF;	//DIR

inverterInterface_t inverterInterface;

inverterSpy inverter_X, inverter_Z;
motorData motor_X, motor_Z;

TEST_GROUP(MotorDriver)
{
    void setup()
    { 
		inverter_X = inverterSpy_init(&inverterInterface, &GPIOD, 0x8000, &GPIOE, 0x4000, 0, 1);
		inverter_Z = inverterSpy_init(&inverterInterface, &GPIOC, 0x0200, &GPIOA, 0x0004, 1, 2);
		motor_X = motor_create(&inverterInterface, inverter_X, motor_getXDatum, getEncoderCount_X);
		motor_Z = motor_create(&inverterInterface, inverter_Z, motor_getZDatum, getEncoderCount_Z);
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
//	motor_stop(motor_Z);
	LONGS_EQUAL(0x0000, GPIOC & 0x0200);
	LONGS_EQUAL(MOTOR_RESET, motor_getState(motor_Z));
	LONGS_EQUAL(0x0000, GPIOD & 0x8000);
	LONGS_EQUAL(MOTOR_RESET, motor_getState(motor_X));	
//  FAIL("Start here");
}

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

#if 0
TEST(InverterDriver, inverterTurnOn)
{
	inverter_switch(&inverter_X, INVERTER_ON);
	inverter_switch(&inverter_Z, INVERTER_ON);
	LONGS_EQUAL(0x8000, GPIOD & 0x8000);
	LONGS_EQUAL(0x0200, GPIOC & 0x0200);	
}

TEST(InverterDriver, inverterTurnOff)
{
	inverter_switch(&inverter_X, INVERTER_ON);
	inverter_switch(&inverter_Z, INVERTER_ON);
	inverter_switch(&inverter_X, INVERTER_OFF);
	inverter_switch(&inverter_Z, INVERTER_OFF);	
	LONGS_EQUAL(0x0000, GPIOD & 0x8000);
	LONGS_EQUAL(0x0000, GPIOC & 0x0200);	
}

TEST(InverterDriver, inverterTurnDir)
{
	inverter_direction(&inverter_X, INVERTER_POS_DIR);
	LONGS_EQUAL(0x0000, GPIOE & 0x4000);
	inverter_direction(&inverter_X, INVERTER_NEG_DIR);
	LONGS_EQUAL(0x4000, GPIOE & 0x4000);
	inverter_direction(&inverter_Z, INVERTER_POS_DIR);
	LONGS_EQUAL(0x0000, GPIOA & 0x0004);
	inverter_direction(&inverter_Z, INVERTER_NEG_DIR);
	LONGS_EQUAL(0x0004, GPIOA & 0x0004);	
}

TEST(InverterDriver, inverterSetSpeed)
{
	uint8_t ret;
	
	ret = inverter_speed(&inverter_X, 250);
	LONGS_EQUAL(1, ret);
	ret = inverter_speed(&inverter_Z, 400);
	LONGS_EQUAL(1, ret);	
}
#endif