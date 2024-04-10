TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(shell pwd)/Bob.v
TOPLEVEL = Bob
MODULE = Bob_test
SIM = verilator
EXTRA_ARGS += --trace -Wno-WIDTHTRUNC -Wno-UNOPTFLAT -Wno-fatal
include $(shell cocotb-config --makefiles)/Makefile.sim