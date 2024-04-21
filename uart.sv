// A basic implementation of 8N1 UART protocol
// 8 data bits, no parity bit, 1 stop bit

module uart_tb
  (input logic clock, reset_n);

  baud_rate_generator #(
    .CLK_HZ(25_000_000), 
    .BAUD_RATE(9600),
    .SAMPLE_RATE(16)
  ) brg(
    .clock(clock),
    .reset_n(reset_n),
    .tick()
  );

endmodule : uart_top

module uart_rx
  ();
endmodule : uart_rx

module uart_tx
  ();
endmodule : uart_tx