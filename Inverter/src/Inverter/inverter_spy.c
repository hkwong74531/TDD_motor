/***
 * Excerpted from "Test-Driven Development for Embedded C",
 * published by The Pragmatic Bookshelf.
 * Copyrights apply to this code. It may not be used to create training material, 
 * courses, books, articles, and the like. Contact us if you are in doubt.
 * We make no guarantees that this code is fit for any purpose. 
 * Visit http://www.pragmaticprogrammer.com/titles/jgade for more book information.
***/
/*- ------------------------------------------------------------------ -*/
/*-    Copyright (c) James W. Grenning -- All Rights Reserved          -*/
/*-    For use by owners of Test-Driven Development for Embedded C,    -*/
/*-    and attendees of Renaissance Software Consulting, Co. training  -*/
/*-    classes.                                                        -*/
/*-                                                                    -*/
/*-    Available at http://pragprog.com/titles/jgade/                  -*/
/*-        ISBN 1-934356-62-X, ISBN13 978-1-934356-62-3                -*/
/*-                                                                    -*/
/*-    Authorized users may use this source code in your own           -*/
/*-    projects, however the source code may not be used to            -*/
/*-    create training material, courses, books, articles, and         -*/
/*-    the like. We make no guarantees that this source code is        -*/
/*-    fit for any purpose.                                            -*/
/*-                                                                    -*/
/*-    www.renaissancesoftware.net james@renaissancesoftware.net       -*/
/*- ------------------------------------------------------------------ -*/

#include "inverter_spy.h"

typedef struct inverterSpy_t{
	uint16_t* onAdd;
	uint16_t  onPattern;
	uint16_t* dirAdd;
	uint16_t  dirPattern;
	uint16_t* errorAdd;
	uint16_t  errorPattern;
	uint8_t	  dirPolarity;
	uint8_t   speedCh;
	inverterInterface_t* inverterInterface;
} inverterSpy_t;

uint8_t I2C1_Buffer1_Tx[8];

static void turnOn(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	*(inverter->onAdd) |= inverter->onPattern;
}

static void turnOff(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	*(inverter->onAdd) &= ~(inverter->onPattern);
}

static void dirPositive(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	if(inverter->dirPolarity == 1)
	{
		*(inverter->dirAdd) |= inverter->dirPattern;
	}
	else
	{
		*(inverter->dirAdd) &= ~(inverter->dirPattern);	
	}
}

static void dirNegative(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	if(inverter->dirPolarity == 0)
	{
		*(inverter->dirAdd) |= inverter->dirPattern;
	}
	else
	{
		*(inverter->dirAdd) &= ~(inverter->dirPattern);	
	}
}

static uint8_t getErrorStatus(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	if((*(inverter->errorAdd) & (inverter->errorPattern))==0)
	{
		return 0;
	}
	return 1;
}

static uint8_t setSpeed(void* spy, uint16_t speed)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;

	I2C1_Buffer1_Tx[(inverter->speedCh)*2] = speed;
	I2C1_Buffer1_Tx[(inverter->speedCh)*2 - 1] = 0x0F & (speed>>8) ;

	return 1;
}

void* inverterSpy_init(inverterInterface_t* inverterInterface, uint16_t* onAdd, uint16_t onPattern, uint16_t* dirAdd, uint16_t dirPattern, uint8_t dirPolarity, uint16_t* errorAdd, uint16_t errorPattern, uint8_t speedCh)
{
	inverterSpy_t* inverterSpy;
	
	inverterSpy = malloc(sizeof(inverterSpy_t));
	
	inverterSpy->onAdd		= onAdd;
	inverterSpy->onPattern	= onPattern;

	inverterSpy->dirAdd		= dirAdd;
	inverterSpy->dirPattern	= dirPattern;
	inverterSpy->dirPolarity= dirPolarity;

	inverterSpy->errorAdd	= errorAdd;
	inverterSpy->errorPattern = errorPattern;
	
	inverterSpy->speedCh	= speedCh;

	inverterSpy->inverterInterface = inverterInterface;
	
	(inverterSpy->inverterInterface)->turnOn	= turnOn;
	(inverterSpy->inverterInterface)->turnOff	= turnOff;
	(inverterSpy->inverterInterface)->dirPositive	= dirPositive;
	(inverterSpy->inverterInterface)->dirNegative	= dirNegative;	
	(inverterSpy->inverterInterface)->getErrorStatus = getErrorStatus;		
	(inverterSpy->inverterInterface)->setSpeed	= setSpeed;
	
	*(onAdd) &= ~(onPattern);

	return (void*)(inverterSpy);
}

void inverterSpy_destroy(void)
{
}

void inverterSpy_turnOn(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	(inverter->inverterInterface)->turnOn(inverter);
}

void inverterSpy_turnOff(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	(inverter->inverterInterface)->turnOff(inverter);
}

void inverterSpy_dirPositive(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	(inverter->inverterInterface)->dirPositive(inverter);
}

void inverterSpy_dirNegative(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	(inverter->inverterInterface)->dirNegative(inverter);
}

uint8_t inverterSpy_getErrorStatus(void* spy)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	return ((inverter->inverterInterface)->getErrorStatus(inverter));
}

uint8_t inverterSpy_setSpeed(void* spy, uint16_t speed)
{
	inverterSpy_t* inverter;
	
	inverter = (inverterSpy_t*)spy;
	return ((inverter->inverterInterface)->setSpeed(inverter, speed));
}




