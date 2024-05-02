#!/bin/bash

yosys -p "read_verilog BobFPGA.v; synth_ecp5 -json synth_out.json -top BobTop"
nextpnr-ecp5 --12k --json synth_out.json --lpf constraints.lpf --textcfg pnr_out.config
ecppack --compress pnr_out.config bitstream.bit
fujprog bitstream.bit
