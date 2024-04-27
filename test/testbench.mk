TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(shell pwd)/Bob.v
TOPLEVEL = Bob
MODULE = Bob_test
SIM=icarus 
WAVES=1
include $(shell cocotb-config --makefiles)/Makefile.sim