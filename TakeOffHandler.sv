`default_nettype none

import BobATC::*;

module TakeOffHandler
  (input  logic       clock, reset_n,
   input  logic [8:0] uart_rx_data,  // Data from UART
   input  logic       uart_rx_valid, // High if data is ready to be read
   input  logic [1:0] runway_active, // High if runway is occupied
   output logic [8:0] uart_tx_data,  // Data to write to UART
   output logic       uart_tx_en);   // High if ready to write to UART

  ///////////////////////////////
  // UART Request Storage FIFO //
  ///////////////////////////////

  msg_t uart_request;

  FIFO #(.WIDTH(9), .DEPTH(15)) uart_requests(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_rx_data),
    .we(uart_rx_valid),
    .re(),
    .data_out(uart_request),
    .full(),
    .empty()
  );

  //////////////////////////////
  // Request Validity Checker //
  //////////////////////////////

  logic msg_valid;

  always_comb begin
    if (uart_request.type >= 3'b011) 
      msg_valid = 1'b0; 
    else if (uart_request.action > 2'b01) 
      msg_valid = 1'b0;
    else
      msg_valid = 1'b1;
  end

  ///////////////////////////////////////
  // Take-Off Requesting Aircraft FIFO //
  ///////////////////////////////////////

  FIFO #(.WIDTH(4), .DEPTH(15)) uart_requests(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_request.id),
    .we(),
    .re(),
    .data_out(),
    .full(),
    .empty()
  );

  //////////////////
  // Reply Sender //
  //////////////////

endmodule : TakeOffHandler

module TakeOffHandlerFSM
  ();

endmodule

module ReplySender
  ();
endmodule : ReplySender

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