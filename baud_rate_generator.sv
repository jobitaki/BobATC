`default_nettype none

module baud_rate_generator
  #(parameter CLK_HZ      = 25_000_000,
              BAUD_RATE   = 9600,
              SAMPLE_RATE = 16)
  (input  logic clock, reset,
   input  logic start_rx,
   input  logic start_tx,
   output logic tick);

  parameter DIVISOR = CLK_HZ / (BAUD_RATE * SAMPLE_RATE);

  logic [$clog2(DIVISOR) + 1:0] clockCount;

  assign tick = clockCount == DIVISOR;

  always_ff @(posedge clock, posedge reset)
    if (reset | tick)
      clockCount <= '0;
    else if (start_rx)
      clockCount <= DIVISOR / 2;
    else if (start_tx)
      clockCount <= '0;
    else
      clockCount <= clockCount + 1'b1;

endmodule : baud_rate_generator

module baud_rate_generator_tb();
  logic clock, reset;
  logic start_rx;
  logic start_tx;
  logic tick;

  baud_rate_generator dut(.*);

  initial begin
    clock = 0;
    forever #1 clock = ~clock;
  end

  initial begin
    $monitor("clockCount %b", dut.clockCount,,
             "tick %b", tick);
  end

  initial begin
    start_rx = 0;
    reset <= 1'b1;
    @(posedge clock);
    reset <= 1'b0;
    for (int i = 0; i < 1000; i++)
      @(posedge clock);
    start_rx <= 1'b1;
    @(posedge clock);
    start_rx <= 1'b0;
    for (int i = 0; i < 1000; i++)
      @(posedge clock);
    $finish;
  end
endmodule : baud_rate_generator_tb