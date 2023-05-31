/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "xtmrctr.h"

/* ----------------------- Variables ----------------------------------------*/
extern XIntc Core1_nIRQ____InterruptController;
extern XTmrCtr Timer1CounterInst;   /* The instance of the Timer Counter */

/* ----------------------- static functions ---------------------------------*/
//static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
	float timerPeriod = (float)(usTim1Timerout50us * 50) * ((float)XPAR_TMRCTR_1_CLOCK_FREQ_HZ / 1000000.0);
	XTmrCtr_SetResetValue(&Timer1CounterInst, 0, (u32)timerPeriod);
	XTmrCtr_Enable(&Timer1CounterInst, XTC_TIMER_0);

    return TRUE;
}

inline void
vMBPortTimersEnable(  )
{
	XTmrCtr_Start(&Timer1CounterInst, XTC_TIMER_0);
}

inline void
vMBPortTimersDisable(  )
{
	XTmrCtr_Stop(&Timer1CounterInst, XTC_TIMER_0);
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
/*
 * call in irq_callbacj.c
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
    
}
*/

