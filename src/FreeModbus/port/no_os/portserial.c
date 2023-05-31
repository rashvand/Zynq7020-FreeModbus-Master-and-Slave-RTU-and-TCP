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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "xuartns550.h"
#include "xgpiops.h"

extern XGpioPs xGpioPs;
extern XUartNs550 UartNs550Instance;

#define EMIO_RS485_DE		(uint32_t)62U

/* ----------------------- static functions ---------------------------------*/
//static void prvvUARTTxReadyISR( void );
//static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
  if (xRxEnable) {
	  while(XUartNs550_IsSending(&UartNs550Instance));
	  XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_MCR_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_MCR_OFFSET) | (XUN_MCR_OUT_1) );
	  XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_IER_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_IER_OFFSET) | (XUN_IER_RX_LINE | XUN_IER_RX_DATA) );
  }
  else {
	  XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_IER_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_IER_OFFSET) & ~(XUN_IER_RX_LINE | XUN_IER_RX_DATA) );
  }
  
  if (xTxEnable) { 
	  XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_MCR_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_MCR_OFFSET) & ~(XUN_MCR_OUT_1) );
	  XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_IER_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_IER_OFFSET) | (XUN_IER_TX_EMPTY) );
  }
  else {
	  XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_IER_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_IER_OFFSET) & ~(XUN_IER_TX_EMPTY) );
  }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity, UCHAR StopBits )
{
	XUartNs550Format _XUartNs550Format;

	if(ulBaudRate >= 300 && ulBaudRate <= 115200){
		_XUartNs550Format.BaudRate 	= ulBaudRate; // 300 ~ 115200
	}
	else{
		return FALSE;
	}

	switch(ucDataBits){
	case 5:
		_XUartNs550Format.DataBits = XUN_FORMAT_5_BITS;
		break;
	case 6:
		_XUartNs550Format.DataBits = XUN_FORMAT_6_BITS;
		break;
	case 7:
		_XUartNs550Format.DataBits = XUN_FORMAT_7_BITS;
		break;
	case 8:
		_XUartNs550Format.DataBits = XUN_FORMAT_8_BITS;
		break;
	default:
		return FALSE;
	}

	switch(eParity){
	case MB_PAR_NONE:
		_XUartNs550Format.Parity = XUN_FORMAT_NO_PARITY;
		break;
	case MB_PAR_ODD:
		_XUartNs550Format.Parity = XUN_FORMAT_ODD_PARITY;
		break;
	case MB_PAR_EVEN:
		_XUartNs550Format.Parity = XUN_FORMAT_EVEN_PARITY;
		break;
	default:
		return FALSE;
	}

	switch(StopBits){
	case 1:
		_XUartNs550Format.StopBits 	= XUN_FORMAT_1_STOP_BIT;
		break;
	case 2:
		_XUartNs550Format.StopBits 	= XUN_FORMAT_2_STOP_BIT;
		break;
	default:
		return FALSE;
	}

	if(XST_SUCCESS == XUartNs550_SetDataFormat(&UartNs550Instance, &_XUartNs550Format)){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */

	XUartNs550_WriteReg(XPAR_UARTNS550_0_BASEADDR, XUN_THR_OFFSET, (u32)ucByte);
	return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	*pucByte = (CHAR) XUartNs550_ReadReg(XPAR_UARTNS550_0_BASEADDR, XUN_RBR_OFFSET);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */

/*
 *move to irq_callback.c
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}
*/

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
/*
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}
*/
