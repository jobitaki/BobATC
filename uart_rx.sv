`default_nettype none

module uart_rx(
  input  logic       clock, reset, 
  input  logic       rx,              // Serial data input line
  output logic [8:0] data,            // Data received
  output logic       done,            // High if data is fully received
  output logic       framing_error
);      

  logic start, tick;

  baud_rate_generator #(
    .CLK_HZ(25_000_000),
    .BAUD_RATE(9600),
    .SAMPLE_RATE(16)
  ) conductor(
    .clock(clock),
    .reset(reset),
    .start_rx(start),
    .start_tx(1'b0),
    .tick(tick)
  );

  logic       collect_data;
  logic       en_data_counter;
  logic       clear_data_counter;
  logic [3:0] data_counter;
  logic       done_data;

  always_ff @(posedge clock, posedge reset)
    if (reset) 
      data <= '0;
    else if (collect_data && tick) begin
      data    <= data >> 1;
      data[8] <= rx;
    end

  assign done_data = data_counter == 4'd9;

  always_ff @(posedge clock, posedge reset)
    if (reset || clear_data_counter)
      data_counter <= '0;
    else if (en_data_counter && tick) 
      data_counter <= data_counter + 1'b1;

  uart_rx_fsm fsm(
    .clock(clock),
    .reset(reset),
    .tick(tick),
    .rx(rx),
    .done_data(done_data),
    .start(start),
    .collect_data(collect_data),
    .en_data_counter(en_data_counter),
    .clear_data_counter(clear_data_counter),
    .framing_error(framing_error),
    .done(done)
  );

endmodule : uart_rx

module uart_rx_fsm(
  input  logic clock, reset,
  input  logic tick,
  input  logic rx,
  input  logic done_data,
  output logic start,
  output logic collect_data,
  output logic en_data_counter,
  output logic clear_data_counter,
  output logic framing_error,
  output logic done
);

  enum logic [1:0] {
    IDLE, 
    START, 
    DATA, 
    FRAMING_ERROR
  } state, next_state;

  always_comb begin
    start              = 1'b0;
    collect_data       = 1'b0;
    en_data_counter    = 1'b0;
    clear_data_counter = 1'b0;
    framing_error      = 1'b0;
    done               = 1'b0;

    case (state)
      IDLE: begin
        if (!rx) begin
          next_state = START;
          start = 1'b1;
        end else
          next_state = IDLE;
      end

      START: begin
        if (tick && rx) begin
          next_state = IDLE;  // Spurious start bit
        end else if (tick && !rx) begin
          next_state = DATA;  // Actual start but
        end else
          next_state = START; // Wait for tick
      end

      DATA: begin
        if (tick && !done_data) begin
          next_state      = DATA;
          collect_data    = 1'b1;
          en_data_counter = 1'b1;
        end else if (tick && done_data) begin
          if (!rx) begin // Should be stop bit, but we got logic low
            next_state    = FRAMING_ERROR;
            framing_error = 1'b1;
          end else begin
            next_state         = IDLE;
            clear_data_counter = 1'b1;
            done               = 1'b1;
          end
        end
      end

      FRAMING_ERROR: begin
        if (tick && rx) begin
          next_state         = IDLE;
          clear_data_counter = 1'b1;
        end else if (tick && !rx) begin
          next_state    = FRAMING_ERROR;
          framing_error = 1'b1;
        end
      end
    endcase
  end

  always_ff @(posedge clock, posedge reset)
    if (reset) 
      state <= IDLE;
    else
      state <= next_state;

endmodule : uart_rx_fsm