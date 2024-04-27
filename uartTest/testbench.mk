TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(shell pwd)/uart.v
TOPLEVEL = UartTB
MODULE = Uart_test
SIM=icarus 
WAVES=1
include $(shell cocotb-config --makefiles)/Makefile.sim