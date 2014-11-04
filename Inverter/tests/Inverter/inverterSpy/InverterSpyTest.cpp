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
#include "inverter_spy.h"
#include "inverter_test.h"
}

#include "CppUTest/TestHarness.h"

// X
uint16_t GPIOD = 0xFFFF;	//SW
uint16_t GPIOE = 0xFFFB;	//DIR
// Z
uint16_t GPIOC = 0xDFFF;	//SW
uint16_t GPIOA = 0x0000;	//DIR

inverterInterface_t inverterInterface;

inverter_t *inverter_X, *inverter_Y;

TEST_GROUP(inverterSpyDriver)
{
    void setup()
    {
		GPIOE = 0xFFFB;	//DIR
		GPIOC = 0xDFFF;	//SW
		GPIOD = 0xFFFF;
		GPIOA = 0x0000;
		memset(I2C1_Buffer1_Tx, 8, 0);
		inverter_X = inverterSpy_init(&inverterInterface, &GPIOD, 0x8000, &GPIOE, 0x4000, 0, &GPIOE, 0x0004, 1);
		inverter_Y = inverterSpy_init(&inverterInterface, &GPIOC, 0x0200, &GPIOA, 0x0004, 1, &GPIOC, 0x2000, 2);
    }

    void teardown()
    {
       free(inverter_X);
	   free(inverter_Y);	   
    }
};

TEST(inverterSpyDriver, inverterSpyCreate)
{
	LONGS_EQUAL(0x0000, GPIOC & 0x0200);
	LONGS_EQUAL(0x0000, GPIOD & 0x8000);	
}

TEST(inverterSpyDriver, inverterSpyTurnOn)
{
	inverterSpy_turnOn(inverter_X);
	LONGS_EQUAL(0x8000, GPIOD & 0x8000);
	LONGS_EQUAL(0x0000, GPIOC & 0x0200);
	inverterSpy_turnOn(inverter_Y);
	LONGS_EQUAL(0x8000, GPIOD & 0x8000);
	LONGS_EQUAL(0x0200, GPIOC & 0x0200);	
}

TEST(inverterSpyDriver, inverterSpyTurnOff)
{
	inverterSpy_turnOn(inverter_X);
	inverterSpy_turnOn(inverter_Y);
	inverterSpy_turnOff(inverter_X);
	LONGS_EQUAL(0x0000, GPIOD & 0x8000);
	LONGS_EQUAL(0x0200, GPIOC & 0x0200);
	inverterSpy_turnOff(inverter_Y);
	LONGS_EQUAL(0x0000, GPIOD & 0x8000);
	LONGS_EQUAL(0x0000, GPIOC & 0x0200);		
}

TEST(inverterSpyDriver, inverterSpyDirPos)
{
	inverterSpy_dirPositive(inverter_X);
	inverterSpy_dirPositive(inverter_Y);
	LONGS_EQUAL(0xFFFB & ~(0x4000), GPIOE);
	LONGS_EQUAL(0x0000 | (0x0004), GPIOA);
}

TEST(inverterSpyDriver, inverterSpyDirNeg)
{
	inverterSpy_dirPositive(inverter_X);
	inverterSpy_dirPositive(inverter_Y);
	inverterSpy_dirNegative(inverter_X);	
	LONGS_EQUAL(0xFFFB | (0x4000), GPIOE);
	LONGS_EQUAL(0x0000 | (0x0004), GPIOA);
	inverterSpy_dirNegative(inverter_Y);	
	LONGS_EQUAL(0xFFFB | (0x4000), GPIOE);
	LONGS_EQUAL(0x0000 & ~(0x0004), GPIOA);	
}

TEST(inverterSpyDriver, inverterSpyError)
{
	CHECK(inverterSpy_getErrorStatus(inverter_X) == 0);
	CHECK(inverterSpy_getErrorStatus(inverter_Y) == 0);	
	fake_inverter_setError(&GPIOE, 0x0004);
	CHECK(inverterSpy_getErrorStatus(inverter_X) == 1);
	CHECK(inverterSpy_getErrorStatus(inverter_Y) == 0);	
	fake_inverter_setError(&GPIOC, 0x2000);
	CHECK(inverterSpy_getErrorStatus(inverter_X) == 1);
	CHECK(inverterSpy_getErrorStatus(inverter_Y) == 1);	
	fake_inverter_clearError(&GPIOC, 0x2000);	
	CHECK(inverterSpy_getErrorStatus(inverter_X) == 1);
	CHECK(inverterSpy_getErrorStatus(inverter_Y) == 0);	
	fake_inverter_clearError(&GPIOE, 0x0004);
	CHECK(inverterSpy_getErrorStatus(inverter_X) == 0);
	CHECK(inverterSpy_getErrorStatus(inverter_Y) == 0);		
}

TEST(inverterSpyDriver, inverterSetSpeed)
{
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[1]);
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[2]);	
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[3]);
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[4]);	
	inverterSpy_setSpeed(inverter_X, 0x842);
	LONGS_EQUAL(0x08, I2C1_Buffer1_Tx[1]);
	LONGS_EQUAL(0x42, I2C1_Buffer1_Tx[2]);	
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[3]);
	LONGS_EQUAL(0x00, I2C1_Buffer1_Tx[4]);	
	inverterSpy_setSpeed(inverter_Y, 0x248);
	LONGS_EQUAL(0x08, I2C1_Buffer1_Tx[1]);
	LONGS_EQUAL(0x42, I2C1_Buffer1_Tx[2]);	
	LONGS_EQUAL(0x02, I2C1_Buffer1_Tx[3]);
	LONGS_EQUAL(0x48, I2C1_Buffer1_Tx[4]);	
}
