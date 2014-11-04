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

#ifndef D_InverterSpy_H
#define D_InverterSpy_H

#define	_CPPUTEST_

#include <stdlib.h>
#include <stdint.h>
#include <memory.h>

#include "inverter.h"

enum {INVERTER_ON = 1, INVERTER_OFF = 0};
enum {INVERTER_POS_DIR = 1, INVERTER_NEG_DIR = 0};
enum {INVERTER_SUCCESS = 1, INVERTER_FAILURE = 0};

extern uint8_t I2C1_Buffer1_Tx[8];

inverter_t* inverterSpy_init(inverterInterface_t*, uint16_t*, uint16_t, uint16_t*, uint16_t, uint8_t, uint16_t*, uint16_t, uint8_t);
void inverterSpy_destroy(void);
void inverterSpy_turnOn(inverter_t*);
void inverterSpy_turnOff(inverter_t*);
void inverterSpy_dirPositive(inverter_t*);
void inverterSpy_dirNegative(inverter_t*);
uint8_t inverterSpy_getErrorStatus(inverter_t*);
uint8_t inverterSpy_setSpeed(inverter_t*, uint16_t);

#endif  /* D_InverterSpy_H */
