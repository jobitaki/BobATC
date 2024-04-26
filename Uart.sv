// A basic implementation of 9N1 UART protocol
// 9 data bits, no parity bit, 1 stop bit

module UartTB();
  logic       clock, reset;
  logic       send;           // High to send data
  logic [8:0] data_tx;           // Data to send
  logic       tx;             // Serial data output line
  logic       ready;

  UartTX dut_tx(.data(data_tx), .*);

  logic hello;

  logic       rx;             // Serial data input line
  logic [8:0] data_rx;        // Data received
  logic       done;           // High if data is fully received
  logic       framing_error;

  assign rx = tx;

  UartRX dut_rx(.data(data_rx), .*);

  initial begin
    clock = 0;
    forever #1 clock = ~clock;
  end

initial begin
    data_tx = 9'b0110_1010_1;
    
    reset <= 1'b1;
    @(posedge clock);
    reset <= 1'b0;
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

endmodule : UartTB
