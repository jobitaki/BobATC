// A basic implementation of 9N1 UART protocol
// 9 data bits, no parity bit, 1 stop bit

module UartTB (
  input logic clock,
  input logic reset,
  input logic send,
  input logic [8:0] data_tx,
  output logic tx,
  output logic ready,
  input logic rx,
  output logic [8:0] data_rx,
  output logic done,
  output logic framing_error
);

  UartTX dut_tx(.data(data_tx), .*);
  UartRX dut_rx(.data(data_rx), .*);

endmodule : UartTB
