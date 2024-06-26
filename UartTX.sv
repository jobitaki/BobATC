`default_nettype none

module UartTX #(
    parameter int CLK_HZ    = 25_000_000,
              int BAUD_RATE = 115200
) (
    input  logic       clock,
    input  logic       reset,
    input  logic       send,    // High to send data
    input  logic [7:0] data,    // Data to send
    output logic       tx,      // Serial data output line
    output logic       ready,   // High if TX is not busy
    output logic       sending  // High when sending a packet
);

  logic start, tick;

  BaudRateGenerator #(
      .CLK_HZ(CLK_HZ),
      .BAUD_RATE(BAUD_RATE)
  ) conductor (
      .clock(clock),
      .reset(reset),
      .start_rx(1'b0),
      .start_tx(start),
      .tick(tick)
  );

  logic       en_data_counter;
  logic [3:0] data_counter;
  logic       done_data;
  logic       clear_data_counter;

  assign done_data = data_counter == 4'd8;

  always_ff @(posedge clock)
    if (reset || clear_data_counter) data_counter <= '0;
    else if (en_data_counter && tick) data_counter <= data_counter + 1;

  logic [7:0] saved_data;
  logic       data_bit;
  logic       send_data;

  always_ff @(posedge clock)
    if (reset) saved_data <= '0;
    else if (start) saved_data <= data;
    else if (send_data && tick) saved_data <= saved_data >> 1;  // LSB first

  always_ff @(posedge clock)
    if (reset || start) data_bit <= 1'b0;
    else if (send_data && tick) data_bit <= saved_data[0];

  logic send_start_bit;
  logic send_stop_bit;

  always_ff @(posedge clock)
    if (reset) tx <= 1'b1;
    else if (send_start_bit) tx <= 1'b0;
    else if (send_data) tx <= data_bit;
    else if (send_stop_bit) tx <= 1'b1;
    else tx <= 1'b1;

  UartTXFsm fsm (
      .clock(clock),
      .reset(reset),
      .send(send),
      .tick(tick),
      .done_data(done_data),
      .start(start),
      .send_start_bit(send_start_bit),
      .send_data(send_data),
      .send_stop_bit(send_stop_bit),
      .en_data_counter(en_data_counter),
      .clear_data_counter(clear_data_counter),
      .ready(ready),
      .sending(sending)
  );

endmodule : UartTX

module UartTXFsm (
    input  logic clock,
    input  logic reset,
    input  logic send,
    input  logic tick,
    input  logic done_data,
    output logic start,
    output logic send_start_bit,
    output logic send_data,
    output logic send_stop_bit,
    output logic en_data_counter,
    output logic clear_data_counter,
    output logic ready,
    output logic sending
);

  typedef enum logic [1:0] {
    IDLE,
    START,
    SEND,
    STOP
  } state_t;

  state_t state, next_state;

  always_comb begin
    start              = 1'b0;
    send_start_bit     = 1'b0;
    send_data          = 1'b0;
    send_stop_bit      = 1'b0;
    en_data_counter    = 1'b0;
    clear_data_counter = 1'b0;
    ready              = 1'b0;
    sending            = 1'b0;
    next_state         = IDLE;

    case (state)
      IDLE: begin
        if (send) begin
          next_state     = START;
          start          = 1'b1;
          send_start_bit = 1'b1;
        end else begin
          next_state = IDLE;
          ready      = 1'b1;
        end
      end

      START: begin
        if (tick) begin
          next_state      = SEND;
          send_data       = 1'b1;
          en_data_counter = 1'b1;
        end else begin
          next_state     = START;
          send_start_bit = 1'b1;
        end
        sending = 1'b1;
      end

      SEND: begin
        if (tick && done_data) begin
          next_state         = STOP;
          send_stop_bit      = 1'b1;
          clear_data_counter = 1'b1;
        end else begin
          next_state      = SEND;
          send_data       = 1'b1;
          en_data_counter = 1'b1;
        end
        sending = 1'b1;
      end

      STOP: begin
        if (tick) begin
          next_state = IDLE;
          ready      = 1'b1;
        end else begin
          next_state    = STOP;
          send_stop_bit = 1'b1;
        end
        sending = 1'b1;
      end

      default: next_state = IDLE;
    endcase
  end

  always_ff @(posedge clock)
    if (reset) state <= IDLE;
    else state <= next_state;

endmodule : UartTXFsm
