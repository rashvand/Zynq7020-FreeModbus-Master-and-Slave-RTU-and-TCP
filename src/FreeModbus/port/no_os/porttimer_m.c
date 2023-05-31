/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
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
 * File: $Id: porttimer_m.c,v 1.60 2013/08/13 15:07:05 Armink add Master Functions$
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mb_m.h"
#include "mbport.h"
#include "xtmrctr.h"



#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
/* ----------------------- Variables ----------------------------------------*/
extern XIntc Core1_nIRQ____InterruptController;
extern XTmrCtr Timer1CounterInst;   /* The instance of the Timer Counter */

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBMasterPortTimersInit(USHORT usTimeOut50us)
{
	float timerPeriod = (float)(usTimeOut50us * 50) * ((float)XPAR_TMRCTR_1_CLOCK_FREQ_HZ / 1000000.0);
	XTmrCtr_SetResetValue(&Timer1CounterInst, 0, (u32)timerPeriod);
	XTmrCtr_Enable(&Timer1CounterInst, XTC_TIMER_0);

    return TRUE;
}

void vMBMasterPortTimersT35Enable()
{
    vMBMasterSetCurTimerMode(MB_TMODE_T35);
    XTmrCtr_Start(&Timer1CounterInst, XTC_TIMER_0);
}

void vMBMasterPortTimersConvertDelayEnable()
{
	XTmrCtr_SetResetValue(&Timer1CounterInst, XTC_TIMER_0, MB_MASTER_DELAY_MS_CONVERT * (XPAR_TMRCTR_1_CLOCK_FREQ_HZ/1000));
	vMBMasterSetCurTimerMode(MB_TMODE_CONVERT_DELAY);
	XTmrCtr_Start(&Timer1CounterInst, XTC_TIMER_0);
}

void vMBMasterPortTimersRespondTimeoutEnable()
{
	XTmrCtr_SetResetValue(&Timer1CounterInst, XTC_TIMER_0, MB_MASTER_TIMEOUT_MS_RESPOND * (XPAR_TMRCTR_1_CLOCK_FREQ_HZ/1000));
	vMBMasterSetCurTimerMode(MB_TMODE_RESPOND_TIMEOUT);
	XTmrCtr_Start(&Timer1CounterInst, XTC_TIMER_0);
}

void vMBMasterPortTimersDisable()
{
	XTmrCtr_Stop(&Timer1CounterInst, XTC_TIMER_0);
}


#endif
