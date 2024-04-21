`default_nettype none

module baud_rate_generator
  #(parameter CLK_HZ      = 25_000_000,
              BAUD_RATE   = 9600,
              SAMPLE_RATE = 16)
  (input  logic clock, reset_n,
   output logic tick);

  parameter DIVISOR = CLK_HZ / (BAUD_RATE * SAMPLE_RATE);

  logic [$clog2(DIVISOR) + 1:0] clockCount;

  assign tick = clockCount == $rtoi(DIVISOR);

  always_ff @(posedge clock, negedge reset_n)
    if (~reset_n | tick)
      clockCount <= '0;
    else
      clockCount <= clockCount + 1'b1;

endmodule : baud_rate_generator

module baud_rate_generator_tb();
  logic clock, reset_n;
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
    reset_n <= 1'b0;
    @(posedge clock);
    reset_n <= 1'b1;
    for (int i = 0; i < 5000; i++)
      @(posedge clock);

    $finish;
  end
endmodule : baud_rate_generator_tb