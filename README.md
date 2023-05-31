# Zynq7020 FreeModbus Master and Slave RTU and TCP
- FreeModbus example with Zynq7020 Master/Slave RTU/TCP
- For implementation, I used AXI Intc and 2 AXI Timer connected to Core1_nIRQ interrupt resource and AXI Uart16550
- One AXI Timer for Systick and another for 50us timer.
- in AXI Uart16550 it's Out1n ip core output I used for RS485 DE signal pin.
- For Block Design schematic see this file "Vivado_Block_Design.pdf"

