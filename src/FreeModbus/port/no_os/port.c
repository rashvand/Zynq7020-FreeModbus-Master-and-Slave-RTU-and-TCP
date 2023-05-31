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
  * File: $Id: port.c,v 1.60 2015/02/01 9:18:05 Armink $
  */

/* ----------------------- System includes --------------------------------*/

/* ----------------------- Modbus includes ----------------------------------*/
#include "port.h"
/* ----------------------- Variables ----------------------------------------*/
extern XTmrCtr SysTickTimerInst;   /* The instance of the Timer Counter */
extern XTmrCtr Timer1CounterInst;   /* The instance of the Timer Counter */

/* ----------------------- Start implementation -----------------------------*/
void EnterCriticalSection(void)
{
//	Xil_ExceptionDisable();
	XTmrCtr_Stop(&SysTickTimerInst, XTC_TIMER_0);
//	XTmrCtr_Stop(&Timer1CounterInst, XTC_TIMER_0);
}

void ExitCriticalSection(void)
{
//	Xil_ExceptionEnable();
	XTmrCtr_Start(&SysTickTimerInst, XTC_TIMER_0);
//	XTmrCtr_Start(&Timer1CounterInst, XTC_TIMER_0);
}

