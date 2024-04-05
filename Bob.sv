`default_nettype none

import BobATC::*;

module Bob
  (input  logic       clock, reset_n,
   input  logic [8:0] uart_rx_data,   // Data from UART
   input  logic       uart_rx_valid,  // High if data is ready to be read
   input  logic [1:0] runway_landing, // High if runway is occupied by landing
   output logic [1:0] runway_takeoff, // High if runway is occupied by takeoff
   output logic [8:0] uart_tx_data,   // Data to write to UART
   output logic       uart_tx_en);    // High if ready to write to UART

  ///////////////////////////////
  // UART Request Storage FIFO //
  ///////////////////////////////

  msg_t uart_request;
  logic rd_request;
  logic uart_empty;

  FIFO #(.WIDTH(9), .DEPTH(15)) uart_requests(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_rx_data),
    .we(uart_rx_valid),
    .re(rd_request),
    .data_out(uart_request),
    .full(),
    .empty(uart_empty)
  );

  ///////////////////////////////
  // UART Reply Storage FIFO //
  ///////////////////////////////

  msg_t uart_request;
  logic rd_request;

  FIFO #(.WIDTH(9), .DEPTH(15)) uart_replies(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_rx_data),
    .we(uart_rx_valid),
    .re(rd_request),
    .data_out(uart_request),
    .full(),
    .empty(uart_empty)
  );

  //////////////////////////////
  // Request Validity Checker //
  //////////////////////////////

  logic msg_valid;

  always_comb begin
    if (uart_request.type >= 3'b100) 
      msg_valid = 1'b0; 
    else if (uart_request.action > 2'b01) 
      msg_valid = 1'b0;
    else
      msg_valid = 1'b1;
  end

  ///////////////////////////////////////
  // Take-Off Requesting Aircraft FIFO //
  ///////////////////////////////////////

  logic takeoff_fifo_full;

  FIFO #(.WIDTH(4), .DEPTH(15)) uart_requests(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_request.id),
    .we(),
    .re(),
    .data_out(),
    .full(takeoff_fifo_full),
    .empty()
  );

  //////////////////
  // Reply Sender //
  //////////////////

  
  

endmodule : Bob

module ReadRequestFSM
  (input  logic       clock, reset_n,
   input  logic       uart_empty,
   input  msg_type_t  msg_type,
   input  logic [1:0] msg_action,
   output logic       rd_request,
   output logic       );

  enum logic [3:0] {QUIET, 
                    INTERPRET, 
                    CLEAR, 
                    HOLD, 
                    DENY,
                    SWITCH_FREQ, 
                    SAY_AGAIN, 
                    DECLARE} state, next_state;

  always_comb begin
    case (state) 
      QUIET: begin
        if (uart_empty) 
          next_state = QUIET;
        else begin
          next_state = INTERPRET;
          rd_request = 1'b1;
        end
      end
      INTERPRET: begin
        if (msg_type == T_REQUEST) begin
          if (msg_action == 2'b00) begin
            // Trying to take off, and if fifo full, just deny.
            if (takeoff_fifo_full) begin
              next_state = DENY;
            end else begin
              next_state = CLEAR;
            end 
          end else if (msg_action == 2'b01) begin
            // Trying to land -- switch frequency
          end
        end else if (msg_type == T_DECLARE) begin
          if (msg_action == 2'b00) begin
            // Declared takeoff on runway 0, remove lock on runway 0
          end else if (msg_action == 2'b01) begin
            // Declared takeoff on runway 1, remove lock on runway 1
          end else begin
            // Switch frequency
          end
        end else if (msg_type == T_EMERGENCY) begin
          // Lock both runways permanently
        end else begin
          // Switch frequency
        end
      end
      CLEAR: begin
        
      end
      HOLD: begin
      end
      SWITCH_FREQ: begin
      end 
      SAY_AGAIN: begin
      end
      DECLARE: begin
      end
      default: begin
        rd_request = 1'b0;
      end
    endcase
  end

  always_ff @(posedge clock, negedge, reset_n)
    if (~reset_n) 
      state <= LISTEN;
    else 
      state <= next_state;

endmodule : ReadRequestFSM

module ClearQueueFSM
  ();
endmodule : ClearQueueFSM

//
//  Module 'FIFO'
//
//  A FIFO (First In First Out) buffer with reconfigurable depth with the given
//  interface and constraints
//    - The buffer is initally empty
//    - Reads are combinational, so data_out is valid unless empty is asserted
//    - Removal from the queue is processed on the clock edge.
//    - Writes are processed on the clock edge
//    - If a write is pending while the buffer is full, do nothing
//    - If a read is pending while the buffer is empty, do nothing
//
module FIFO
 #(parameter WIDTH=9,
             DEPTH=4)
  (input  logic             clock, reset_n,
   input  logic [WIDTH-1:0] data_in,
   input  logic             we, re,
   output logic [WIDTH-1:0] data_out,
   output logic             full, empty);

  logic [DEPTH-1:0][WIDTH-1:0] queue;          
  logic [$clog2(DEPTH):0] count;
  logic [$clog2(DEPTH)-1:0] put_ptr, get_ptr; 

  assign empty = (count == 0);
  assign full = (count == DEPTH);

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      count <= 0;
      get_ptr <= 0;
      put_ptr <= 0;
    end
    else begin
      // If reading (and not empty) and writing, first read and then write
      if ((re && !empty) && we) begin
        data_out <= queue[get_ptr];
        get_ptr <= get_ptr + 1;
        queue[put_ptr] <= data_in;
        put_ptr <= put_ptr + 1;
      end else if (actually_re && (!empty)) begin
        data_out <= queue[get_ptr];
        get_ptr <= get_ptr + 1;
        count <= count - 1;
      end else if (we && (!full)) begin
        queue[put_ptr] <= data_in;
        put_ptr <= put_ptr + 1;
        count <= count + 1;
      end
    end
  end

endmodule : FIFO