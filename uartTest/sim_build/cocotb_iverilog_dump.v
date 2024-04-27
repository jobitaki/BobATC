module cocotb_iverilog_dump();
initial begin
    $dumpfile("sim_build/UartTB.fst");
    $dumpvars(0, UartTB);
end
endmodule
