`default_nettype none

module BaudRateGenerator #(
    parameter int CLK_HZ    = 25_000_000,
              int BAUD_RATE = 115200
) (
    input  logic clock,
    input  logic reset,
    input  logic start_rx,
    input  logic start_tx,
    output logic tick
);

  parameter int DIVISOR = CLK_HZ / BAUD_RATE;

  logic [$clog2(DIVISOR) + 1:0] clockCount;

  assign tick = clockCount == DIVISOR;

  always_ff @(posedge clock)
    if (reset | tick) clockCount <= '0;
    else if (start_rx) clockCount <= DIVISOR / 2;
    else if (start_tx) clockCount <= '0;
    else clockCount <= clockCount + 1'b1;

endmodule : BaudRateGenerator
