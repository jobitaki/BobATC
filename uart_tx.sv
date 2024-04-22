`default_nettype none

module uart_tx(
  input  logic       clock, reset_n, 
  input  logic       send,           // High to send data
  input  logic [8:0] data,           // Data to send
  output logic       tx,             // Serial data output line
  output logic       ready           // High if TX is not busy
);         

  logic start, tick;

  baud_rate_generator #(
    .CLK_HZ(25_000_000),
    .BAUD_RATE(9600),
    .SAMPLE_RATE(16)
  ) conductor(
    .clock(clock),
    .reset_n(reset_n),
    .start_rx(1'b0),
    .start_tx(start),
    .tick(tick)
  );

  logic       en_data_counter;
  logic [3:0] data_counter;
  logic       done_data;
  logic       clear_data_counter;

  assign done_data = data_counter == 4'd9;

  always_ff @(posedge clock, negedge reset_n) 
    if (!reset_n || clear_data_counter) 
      data_counter <= '0;
    else if (en_data_counter && tick)
      data_counter <= data_counter + 1;

  logic [8:0] saved_data;
  logic       data_bit;
  logic       send_data;

  always_ff @(posedge clock, negedge reset_n) 
    if (!reset_n)
      saved_data <= '0;
    else if (start)
      saved_data <= data;
    else if (send_data && tick)
      saved_data <= saved_data >> 1; // LSB first

  always_ff @(posedge clock, negedge reset_n)
    if (!reset_n) 
      data_bit <= 1'b0;
    else if (send_data && tick) 
      data_bit <= saved_data[0];

  logic send_start_bit;
  logic send_stop_bit;

  always_ff @(posedge clock, negedge reset_n)
    if (!reset_n) 
      tx <= 1'b1;
    else if (send_start_bit) 
      tx <= 1'b0;
    else if (send_data)
      tx <= data_bit;
    else if (send_stop_bit)
      tx <= 1'b1;
    else 
      tx <= 1'b1;

  uart_tx_fsm fsm(
    .clock(clock),
    .reset_n(reset_n),
    .send(send),
    .tick(tick),
    .done_data(done_data),
    .start(start),
    .send_start_bit(send_start_bit),
    .send_data(send_data),
    .send_stop_bit(send_stop_bit),
    .en_data_counter(en_data_counter),
    .clear_data_counter(clear_data_counter),
    .ready(ready)
  );
  
endmodule : uart_tx

module uart_tx_fsm(
  input  logic clock, reset_n,
  input  logic send,
  input  logic tick, 
  input  logic done_data,
  output logic start,
  output logic send_start_bit,
  output logic send_data,
  output logic send_stop_bit,
  output logic en_data_counter,
  output logic clear_data_counter,
  output logic ready
);

  enum logic [1:0] {IDLE, START, SEND, STOP} state, next_state;

  always_comb begin
    start              = 1'b0;
    send_start_bit     = 1'b0;
    send_data          = 1'b0;
    send_stop_bit      = 1'b0;
    en_data_counter    = 1'b0;
    clear_data_counter = 1'b0;
    ready              = 1'b0;

    case (state)
      IDLE: begin
        if (send) begin
          next_state     = START;
          start          = 1'b1;
          send_start_bit = 1'b1;
        end else begin
          next_state     = IDLE;
          ready          = 1'b1;
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
      end

      STOP: begin
        if (tick) begin
          next_state = IDLE;
          ready      = 1'b1;
        end else begin
          next_state    = STOP;
          send_stop_bit = 1'b1;
        end
      end
    endcase
  end

  always_ff @(posedge clock, negedge reset_n)
    if (!reset_n)
      state <= IDLE;
    else
      state <= next_state;

endmodule : uart_tx_fsm