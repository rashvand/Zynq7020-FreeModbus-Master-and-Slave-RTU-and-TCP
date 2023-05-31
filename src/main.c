#include <stdio.h>
#include <stdbool.h>
#include "netif/xadapter.h"
#include "platform.h"
#include "platform_config.h"
#include "xil_printf.h"
#include "lwip/tcp.h"
#include "xil_cache.h"
#include "xparameters.h"
#include "xparameters_ps.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "lwip/tcp.h"
#include "platform.h"
#include "platform_config.h"
#include "netif/xadapter.h"
#include "netif/xemacpsif.h"
#include "xscutimer.h"
#include "xil_cache.h"
#include "xil_cache_l.h"
#include "xil_mmu.h"
#include "xtime_l.h"
#include "sleep.h"

#include "xtmrctr.h"
#include "xuartns550.h"
#include "xil_exception.h"

#include "xintc.h"
#include "xintc_l.h"
#include "xintc_i.h"

#include "xgpiops.h"

#include "mb_m.h"
#include "mb.h"
#include "user_mb_app.h"

#define COUNTS_PER_USECOND  (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / (2U*1000000U))

#define LOG_TYPE_DATA 		"\033[0m"
#define LOG_TYPE_INFO		"\033[32m"
#define LOG_TYPE_WARNING	"\033[33m"
#define LOG_TYPE_ERROR		"\033[31m"

#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define TIMER_DEVICE_ID		XPAR_SCUTIMER_DEVICE_ID
#define INTC_DIST_BASE_ADDR	XPAR_SCUGIC_0_DIST_BASEADDR
#define TIMER_IRPT_INTR		XPAR_SCUTIMER_INTR
#define RESET_RX_CNTR_LIMIT	400
#define ETH____TRANCIEVE_LENGTH 			2048

#define EMIO_RS485_DE		(uint32_t)62U

XGpioPs_Config *GPIOConfigPtr;
XGpioPs xGpioPs;

extern uint32_t  xemacpsif_addr;
extern void XEmacPs_IntrHandler(void *XEmacPsPtr);
extern void timer_callback(void);

XIntc Core1_nIRQ____InterruptController;
XTmrCtr SysTickTimerInst;   /* The instance of the Timer Counter */
XTmrCtr Timer1CounterInst;   /* The instance of the Timer Counter */
XUartNs550 UartNs550Instance;	/* Instance of the UART Device */



eMBMasterReqErrCode Modbus_err;



typedef struct{

	//--------- S: SoftUart SharedVariables ---------
	uint32_t 			SoftUart_Flag;
	uint32_t 			SoftUart_Data;
	//--------- E: SoftUart SharedVariables ---------

	//--------- S: Network SharedVariables ---------
	uint32_t 			NetIF_isReady;

	uint8_t  			NetIF____IP_Address[4];
	uint8_t  			NetIF____Subnet_Address[4];
	uint8_t  			NetIF____Gateway_Address[4];
	uint8_t  			NetIF____PriDNS_Server_Address[4];
	uint8_t  			NetIF____SecDNS_Server_Address[4];
	uint8_t  			NetIF____MAC_Address[6];
	uint16_t 			NetIF____Port_Number;
	uint64_t 			SysIF____Serial;
	uint8_t	 			NetIF____Host_Name[15];
	uint8_t  			NetIF____DHCP_STATUS;

	uint8_t				Connected;
	uint32_t 			NumberOfByteRecieve;
	uint8_t 			RecieveLan[ETH____TRANCIEVE_LENGTH];
	uint32_t 			NumberOfByteSend;
	uint8_t 			SendLan[ETH____TRANCIEVE_LENGTH];
	uint8_t 			StartForDefault;
	uint32_t 			EmacDataLenSend;
	uint32_t 			EmacDataSend[1000];
	uint32_t 			EmacDataLenReciev;
	uint8_t 			EmacDataReciev[1000];
	int32_t 			Network_Speed;

	//--------- E: Network SharedVariables ---------

	//--------- S: RTC SharedVariables ---------
	uint8_t				IsUnixTimeValid;
	uint32_t			UnixTime;
	uint64_t			UnixTimeInMiliSec;
	//--------- E: RTC SharedVariables ---------

	//--------- S: IEC61850 SharedVariables ---------
	uint32_t  			SharedMemoryArray[64];
	uint64_t			SerialNumber;
	uint8_t 			RelayOperationMode;
	//--------- E: IEC61850 SharedVariables ---------

} SharedMemoryOnOCM_t;


SharedMemoryOnOCM_t *SharedMemoryOnOCM=(SharedMemoryOnOCM_t*)0xFFFF0000;
XScuGic_Config *IntcConfig;
XScuGic IntcInstancePtr;
struct tcp_pcb *pcb;
err_t err;
extern volatile int TcpFastTmrFlag;
extern volatile int TcpSlowTmrFlag;
static struct netif server_netif;
struct netif *echo_netif;
void tcp_fasttmr(void);
void tcp_slowtmr(void);
void lwip_init();
void tcp_fasttmr(void);
void tcp_slowtmr(void);
extern struct netif *echo_netif;
volatile int TcpFastTmrFlag = 0;
volatile int TcpSlowTmrFlag = 0;
static int ResetRxCntr = 0;

void timer_callback()
{
	static int DetectEthLinkStatus = 0;
	static int odd = 1;
	ResetRxCntr++;
	if (ResetRxCntr >= 400) {
		xemacpsif_resetrx_on_no_rxdata(echo_netif);
		ResetRxCntr = 0;
	}
	DetectEthLinkStatus++;
	TcpFastTmrFlag = 1;
	odd = !odd;
	if (odd) {
		TcpSlowTmrFlag = 1;
	}
	if (DetectEthLinkStatus == ETH_LINK_DETECT_INTERVAL) {
		eth_link_detect(echo_netif);
		DetectEthLinkStatus = 0;
	}
}


int send_data () {
	tcp_write(pcb,SharedMemoryOnOCM->SendLan, SharedMemoryOnOCM->NumberOfByteSend,1);
	SharedMemoryOnOCM->NumberOfByteSend=0;
	return 1;
}

err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err){
	if (!p) {
		tcp_close(tpcb);
		tcp_recv(tpcb, NULL);
		return ERR_OK;
	}

	tcp_recved(tpcb, p->len);
	pcb=tpcb;

	for(int i=0; i<p->len; i++){
		SharedMemoryOnOCM->RecieveLan[i]=((uint8_t *)p->payload)[i];
	}
	int num=p->len;
	pbuf_free(p);
	SharedMemoryOnOCM->NumberOfByteRecieve=num;
	return ERR_OK;
}
err_t accept_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	static int connection = 1;
	tcp_recv(newpcb, recv_callback);
	tcp_arg(newpcb, (void*)(UINTPTR)connection);
	connection++;

	return ERR_OK;
}
void send_dataEmac(){
	struct pbuf *p;

	p = pbuf_alloc(PBUF_RAW, XEMACPS_MAX_FRAME_SIZE, PBUF_POOL);
	struct xemac_s *xemac = (struct xemac_s *)(echo_netif->state);
	xemacpsif_s *xemacpsif = (xemacpsif_s *)(xemac->state);

	for(int i=0; i<SharedMemoryOnOCM->EmacDataLenSend; i++){
		*(((uint32_t *)p->payload)+i) = SharedMemoryOnOCM->EmacDataSend[i];
	}
	p->len = SharedMemoryOnOCM->EmacDataLenSend;
	emacps_sgsend(xemacpsif,p);
	pbuf_free(p);
	
	SharedMemoryOnOCM->EmacDataLenSend=0;
}
void DoWorkForGoose(struct pbuf *p){
	//Copy From xemacpsif.c line 170
	u8 GooseFlag=0;
	u8  *dp=(u8 *)p->payload;
	if(p->len>=32 && p->len<1000){
		if(dp[6] == 0xAA && dp[7] == 0xEE){
				if(dp[0] == 0xFF && dp[1] == 0xFF && dp[2] == 0xFF && dp[3] == 0xFF && dp[4] == 0xFF && dp[5] == 0xFF && dp[6] == 0xAA && dp[7] == 0xEE && dp[8] == 0x44 && dp[9] == 0x44 && dp[10] == 0x44 && dp[11] == 0x44){
					GooseFlag = 1;
				}
		}
		else if (dp[16] == 0x88 && dp[17] == 0xb8){
			GooseFlag = 1;
		}
	}
	if(GooseFlag==1){

		for(int i=0; i<p->len; i++){
			SharedMemoryOnOCM->EmacDataReciev[i]=*(((uint8_t *)p->payload)+i);
		}
		SharedMemoryOnOCM->EmacDataLenReciev=p->len;
	}
}

void UART16550_0_Irq(void *CallBackRef, u32 Event, unsigned int EventData)
{
	XUartNs550 *UartNs550Ptr = (XUartNs550 *)CallBackRef;

	switch (Event) {
	case XUN_EVENT_RECV_TIMEOUT:
		break;
	case XUN_EVENT_MODEM:
		break;
	case XUN_EVENT_SENT_DATA:
		XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_IER_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_IER_OFFSET) | (XUN_IER_TX_EMPTY) );
		#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
			pxMBMasterFrameCBTransmitterEmpty();
		#endif
		#if MB_SLAVE_RTU_ENABLED>0 || MB_SLAVE_ASCII_ENABLED>0
			pxMBFrameCBTransmitterEmpty();
		#endif
		break;
	 case XUN_EVENT_RECV_DATA:
		#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
			pxMBMasterFrameCBByteReceived();
		#endif

		#if MB_SLAVE_RTU_ENABLED>0 || MB_SLAVE_ASCII_ENABLED>0
			pxMBFrameCBByteReceived();
		#endif
		break;
	 case XUN_EVENT_RECV_ERROR:
		XUartNs550_GetLastErrors(UartNs550Ptr);
		break;
	}
}
bool BlinkyFlag = false;
void SysTickTimer_Handler(void *CallBackRef, u8 TmrCtrNumber){

	XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

	if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber)) {
		#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
		  eMBMasterPoll();
		#endif
		#if MB_SLAVE_RTU_ENABLED>0 || MB_SLAVE_ASCII_ENABLED>0
		  eMBPoll();
		#endif

//		BlinkyFlag = !BlinkyFlag;
//		XGpioPs_WritePin(&xGpioPs, EMIO_RS485_DE, BlinkyFlag);
	}
}

void Timer1_Irq(void *CallBackRef, u8 TmrCtrNumber){

	XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

	/*
	 * Check if the timer counter has expired, checking is not necessary
	 * since that's the reason this function is executed, this just shows
	 * how the callback reference can be used as a pointer to the instance
	 * of the timer counter that expired, increment a shared variable so
	 * the main thread of execution can see the timer expired
	 */

	if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber)) {

		#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0
		  (void) pxMBMasterPortCBTimerExpired();
		#endif
		#if MB_SLAVE_RTU_ENABLED>0 || MB_SLAVE_ASCII_ENABLED>0
		   (void )pxMBPortCBTimerExpired();
		#endif

//		XTmrCtr_SetOptions(InstancePtr, TmrCtrNumber, 0);
	}
}

uint64_t XTime_GetInMiliSec(void) {
	XTime tCur;
	XTime_GetTime(&tCur);
	return ((uint64_t)((tCur / COUNTS_PER_USECOND) / 1000));
}
uint64_t XTime_GetInMicroSec(void) {
	XTime tCur;
	XTime_GetTime(&tCur);
	return ((uint64_t)((tCur / COUNTS_PER_USECOND)));
}

int main(){
//	Xil_L2CacheDisable();
	Xil_SetTlbAttributes(0xFFFF0000,0x14de2); // S=b1 TEX=b100 AP=b11, Domain=b1111, C=b0, B=b0

	//---------S: Interrupts Config --------
	int Intc_Status;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	Intc_Status = XIntc_Initialize(&Core1_nIRQ____InterruptController, XPAR_INTC_0_DEVICE_ID);
	if (Intc_Status != XST_SUCCESS) {
		xil_printf("%sXIntc_Initialize Error\r\n", LOG_TYPE_ERROR);
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly.
	 */
	Intc_Status = XIntc_SelfTest(&Core1_nIRQ____InterruptController);
	if (Intc_Status != XST_SUCCESS) {
		xil_printf("%sXIntc_SelfTest Error\r\n", LOG_TYPE_ERROR);
	}
	//---------E: Interrupts Config --------

	//---------S: UART Config ---------
	/*
	 * Initialize the UART driver so that it's ready to use.
	 */
	Intc_Status = XUartNs550_Initialize(&UartNs550Instance, XPAR_UARTNS550_0_DEVICE_ID);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	Intc_Status = XUartNs550_SelfTest(&UartNs550Instance);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Intc_Status = XIntc_Connect(&Core1_nIRQ____InterruptController, XPAR_INTC_0_UARTNS550_0_VEC_ID, (XInterruptHandler)XUartNs550_InterruptHandler, (void *)&UartNs550Instance);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handlers for the UART that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the UART driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	XUartNs550_SetHandler(&UartNs550Instance, UART16550_0_Irq, &UartNs550Instance);

	/*
	 * Enable the interrupt of the UART so interrupts will occur, setup
	 * a local loopback so data that is sent will be received, and keep the
	 * FIFOs enabled.
	 */
	u16 Options = XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE | XUN_OPTION_RESET_TX_FIFO | XUN_OPTION_RESET_RX_FIFO | XUN_OPTION_ASSERT_OUT1;
	XUartNs550_SetOptions(&UartNs550Instance, Options);

	// Set DE low
	XUartNs550_WriteReg( XPAR_UARTNS550_0_BASEADDR, XUN_MCR_OFFSET, XUartNs550_ReadReg((XPAR_UARTNS550_0_BASEADDR), XUN_MCR_OFFSET) | (XUN_MCR_OUT_1) );

	XUartNs550_SetFifoThreshold(&UartNs550Instance, XUN_FIFO_TRIGGER_01);

	/*
	 * Set the baud rate and number of stop bits
	 */
//	XUartNs550_SetBaud(XPAR_UARTNS550_0_BASEADDR, XPAR_UARTNS550_0_CLOCK_FREQ_HZ, 115200);
//
//	XUartNs550_SetLineControlReg(XPAR_UARTNS550_0_BASEADDR, XUN_LCR_8_DATA_BITS);
	//---------E: UART Config ---------

	//---------S: Timer 0 Config (Systick) ---------
	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Intc_Status = XTmrCtr_Initialize(&SysTickTimerInst, XPAR_TMRCTR_0_DEVICE_ID);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly, use the 1st timer in the device (0)
	 */
	Intc_Status = XTmrCtr_SelfTest(&SysTickTimerInst, XTC_TIMER_0);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Intc_Status = XIntc_Connect(&Core1_nIRQ____InterruptController, XPAR_INTC_0_TMRCTR_0_VEC_ID, (XInterruptHandler)XTmrCtr_InterruptHandler, (void *)&SysTickTimerInst);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the timer counter that will be called from the
	 * interrupt context when the timer expires, specify a pointer to the
	 * timer counter driver instance as the callback reference so the
	 * handler is able to access the instance data
	 */
	XTmrCtr_SetHandler(&SysTickTimerInst, SysTickTimer_Handler, &SysTickTimerInst);

	/*
	 * Enable the interrupt of the timer counter so interrupts will occur
	 * and use auto reload mode such that the timer counter will reload
	 * itself automatically and continue repeatedly, without this option
	 * it would expire once only
	 */
	XTmrCtr_SetOptions(&SysTickTimerInst, XTC_TIMER_0, XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION | XTC_AUTO_RELOAD_OPTION);

	/*
	 * Set a reset value for the timer counter such that it will expire
	 * eariler than letting it roll over from 0, the reset value is loaded
	 * into the timer counter when it is started
	 */
	XTmrCtr_SetResetValue(&SysTickTimerInst, XTC_TIMER_0, (XPAR_TMRCTR_0_CLOCK_FREQ_HZ/1000)); // Systick (1ms)

	/*
	 * Start the timer counter such that it's incrementing by default,
	 * then wait for it to timeout a number of times
	 */
	XTmrCtr_Start(&SysTickTimerInst, XTC_TIMER_0);
	//---------E: Timer 0 Config (Systick) ---------

	//---------S: Timer 1 Config (Modbus) ---------
	/*
	 * Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Intc_Status = XTmrCtr_Initialize(&Timer1CounterInst, XPAR_TMRCTR_1_DEVICE_ID);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly, use the 1st timer in the device (0)
	 */
	Intc_Status = XTmrCtr_SelfTest(&Timer1CounterInst, XTC_TIMER_0);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Intc_Status = XIntc_Connect(&Core1_nIRQ____InterruptController, XPAR_INTC_0_TMRCTR_1_VEC_ID, (XInterruptHandler)XTmrCtr_InterruptHandler, (void *)&Timer1CounterInst);
	if (Intc_Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the timer counter that will be called from the
	 * interrupt context when the timer expires, specify a pointer to the
	 * timer counter driver instance as the callback reference so the
	 * handler is able to access the instance data
	 */
	XTmrCtr_SetHandler(&Timer1CounterInst, Timer1_Irq, &Timer1CounterInst);

	XTmrCtr_SetOptions(&Timer1CounterInst, XTC_TIMER_0, XTC_INT_MODE_OPTION | XTC_DOWN_COUNT_OPTION); //  | XTC_AUTO_RELOAD_OPTION
	//---------E: Timer1 Config (Modbus) ---------

	//---------S: Intrerrupt Start ---------
	/*
	 * Start the interrupt controller such that interrupts are enabled for
	 * all devices that cause interrupts, specify simulation mode so that
	 * an interrupt can be caused by software rather than a real hardware
	 * interrupt.
	 */
	Intc_Status = XIntc_Start(&Core1_nIRQ____InterruptController, XIN_REAL_MODE);
	if (Intc_Status != XST_SUCCESS) {
		xil_printf("%sXIntc_Start Error\r\n", LOG_TYPE_ERROR);
	}

	/*
	 * Enable the interrupt for the device and then cause (simulate) an
	 * interrupt so the handlers will be called.
	 */
	XIntc_Enable(&Core1_nIRQ____InterruptController, XPAR_INTC_0_UARTNS550_0_VEC_ID);
	XIntc_Enable(&Core1_nIRQ____InterruptController, XPAR_INTC_0_TMRCTR_0_VEC_ID);
	XIntc_Enable(&Core1_nIRQ____InterruptController, XPAR_INTC_0_TMRCTR_1_VEC_ID);

	/*
	 * Acknowledge all pending interrupts by reading the interrupt status
	 * register and writing the value to the acknowledge register
	 */
	u32 Temp = XIntc_In32(Core1_nIRQ____InterruptController.BaseAddress + XIN_ISR_OFFSET);

	XIntc_Out32(Core1_nIRQ____InterruptController.BaseAddress + XIN_IAR_OFFSET, Temp);

	/*
	 * Verify that there are no interrupts by reading the interrupt status
	 */
	u32 CurrentISR = XIntc_In32(Core1_nIRQ____InterruptController.BaseAddress + XIN_ISR_OFFSET);

	/*
	 * ISR should be zero after all interrupts are acknowledged
	 */
	if (CurrentISR != 0) {
		xil_printf("%sXST_INTC_FAIL_SELFTEST Error\r\n", LOG_TYPE_ERROR);
	}

	/*
	 * Initialize the exception table.
	 */
	Xil_ExceptionInit();

	/*
	 * Register the interrupt controller handler with the exception table.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XIntc_InterruptHandler, &Core1_nIRQ____InterruptController); // XIL_EXCEPTION_ID_INT   XIL_EXCEPTION_ID_FIQ_INT

	/*
	 * Enable exceptions.
	 */
	Xil_ExceptionEnable();
	//---------E: Intrerrupt Start ---------

#if(0)
	eMBMasterInit(MB_RTU, 0, 115200, MB_PAR_NONE);
	eMBMasterEnable();
//	Modbus_err=eMBMasterReqReadInputRegister(0x01,0x0000,2,200);
//	Modbus_err=eMBMasterReqReadHoldingRegister(0x01,0x0000,4,200);
//	Modbus_err=eMBMasterReqReadDiscreteInputs(0x01,0x0000,16,200);
//	Modbus_err=eMBMasterReqReadCoils(0x01,0x0000,16,200);


	while(1){
		Modbus_err = eMBMasterReqWriteHoldingRegister(1, 1, 10, 1000);

		usleep(500000);
	}
#else
	eMBErrorCode eStatus = eMBInit( MB_RTU, 1, 0, 115200, MB_PAR_NONE );
	eStatus = eMBEnable();

	while(1){
		usleep(500000);
	}
#endif



	while(SharedMemoryOnOCM->NetIF_isReady != (uint32_t)0xAB12CD34){
		usleep(1000);
	}
	usleep(500000);
//	xil_printf("Core 1 is Start\r\n");

	ip_addr_t ipaddr, netmask, gw;
	unsigned char mac_ethernet_address[] = {SharedMemoryOnOCM->NetIF____MAC_Address[0],SharedMemoryOnOCM->NetIF____MAC_Address[1],SharedMemoryOnOCM->NetIF____MAC_Address[2],SharedMemoryOnOCM->NetIF____MAC_Address[3],SharedMemoryOnOCM->NetIF____MAC_Address[4],SharedMemoryOnOCM->NetIF____MAC_Address[5]};
	echo_netif = &server_netif;

	lwip_init();

	xemac_add(echo_netif, &ipaddr, &netmask, &gw, mac_ethernet_address, XPAR_XEMACPS_1_BASEADDR);
	netif_set_default(echo_netif);
	netif_set_up(echo_netif);

	IP4_ADDR(&(echo_netif->ip_addr),SharedMemoryOnOCM->NetIF____IP_Address[0], SharedMemoryOnOCM->NetIF____IP_Address[1], SharedMemoryOnOCM->NetIF____IP_Address[2], SharedMemoryOnOCM->NetIF____IP_Address[3]);
	IP4_ADDR(&(echo_netif->gw), 	SharedMemoryOnOCM->NetIF____Gateway_Address[0], SharedMemoryOnOCM->NetIF____Gateway_Address[1], SharedMemoryOnOCM->NetIF____Gateway_Address[2], SharedMemoryOnOCM->NetIF____Gateway_Address[3]);
	IP4_ADDR(&(echo_netif->netmask),SharedMemoryOnOCM->NetIF____Subnet_Address[0], SharedMemoryOnOCM->NetIF____Subnet_Address[1], SharedMemoryOnOCM->NetIF____Subnet_Address[2], SharedMemoryOnOCM->NetIF____Subnet_Address[3]);
	ipaddr.addr = echo_netif->ip_addr.addr;
	gw.addr = echo_netif->gw.addr;
	netmask.addr = echo_netif->netmask.addr;

	unsigned port = SharedMemoryOnOCM->NetIF____Port_Number;

	pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
	err = tcp_bind(pcb, IP_ANY_TYPE, port);
	tcp_arg(pcb, NULL);
	pcb = tcp_listen(pcb);
	tcp_accept(pcb, accept_callback);

	while(1){
		timer_callback();
		if (TcpFastTmrFlag) {
			tcp_fasttmr();
			TcpFastTmrFlag = 0;
		}
		if (TcpSlowTmrFlag) {
			tcp_slowtmr();
			TcpSlowTmrFlag = 0;
		}
		XEmacPs_IntrHandler((void *)xemacpsif_addr);
		xemacif_input(echo_netif);
		if (SharedMemoryOnOCM->NumberOfByteSend != 0)
			send_data();
		if (SharedMemoryOnOCM->EmacDataLenSend != 0)
			send_dataEmac();
		SharedMemoryOnOCM->Connected = pcb->state;

		usleep(100);
	}

	cleanup_platform();
	return 0;
}
