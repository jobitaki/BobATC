// A basic implementation of 9N1 UART protocol
// 9 data bits, no parity bit, 1 stop bit

module uart_tb();
  logic       clock, reset_n;
  logic       send;           // High to send data
  logic [8:0] data;           // Data to send
  logic       tx;             // Serial data output line
  logic       ready;

  uart_tx dut(.*);

  initial begin
    clock = 0;
    forever #1 clock = ~clock;
  end

  initial begin
    data = 9'b0110_1010_1;
    
    reset_n <= 1'b0;
    @(posedge clock);
    reset_n <= 1'b1;
    @(posedge clock);
    @(posedge clock);
    @(posedge clock);
    @(posedge clock);
    send <= 1'b1;
    @(posedge clock);
    for (int i = 0; i < 5000; i++)
      @(posedge clock);
    send <= 1'b0;
    for (int i = 0; i < 5000; i++)
      @(posedge clock);
    $finish;
  end

endmodule : uart_tb
