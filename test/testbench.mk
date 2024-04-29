TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(shell pwd)/Bob.v
TOPLEVEL = BobTop
MODULE = Bob_test_with_UART
SIM=icarus 
WAVES=1
include $(shell cocotb-config --makefiles)/Makefile.sim