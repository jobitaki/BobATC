////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   --  --  --  --  --  --  --  --  BobATC  --  --  --  --  --  --  --  --   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

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
  logic uart_rd_request;
  logic uart_empty;

  FIFO #(.WIDTH(9), .DEPTH(15)) uart_requests(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_rx_data),
    .we(uart_rx_valid),
    .re(uart_rd_request),
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

  ////////////////////////////
  // Aircraft Take-Off FIFO //
  ////////////////////////////

  logic queue_takeoff_plane, clear_takeoff_plane;
  logic [3:0] cleared_takeoff_id;
  logic takeoff_fifo_full;

  FIFO #(.WIDTH(4), .DEPTH(15)) takeoff_fifo(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_request.id),
    .we(queue_takeoff_plane),
    .re(clear_takeoff_plane),
    .data_out(cleared_takeoff_id),
    .full(takeoff_fifo_full),
    .empty()
  );

  ///////////////////////////
  // Aircraft Landing FIFO //
  ///////////////////////////

  logic queue_landing_plane, clear_landing_plane;
  logic [3:0] cleared_landing_id;
  logic landing_fifo_full;

  FIFO #(.WIDTH(4), .DEPTH(15)) landing_fifo(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_request.id),
    .we(queue_landing_plane),
    .re(clear_landing_plane),
    .data_out(cleared_landing_id),
    .full(landing_fifo_full),
    .empty()
  );

  //////////////////
  // Reply Sender //
  //////////////////

  msg_t reply_to_send;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      reply_to_send <= 0;
    end else if (send_clear) begin
      reply_to_send.plane_id <= uart_request.plane_id;
      reply_to_send.msg_type <= T_CLEAR;
      reply_to_send.action   <= // Runway
    end else if (send_hold) begin
      reply_to_send.plane_id <= uart_request.plane_id;
      reply_to_send.msg_type <= T_HOLD;
    end else if (send_say_ag) begin
      reply_to_send.plane_id <= uart_request.plane_id;
      reply_to_send.msg_type <= T_SAY_AGAIN;
    end else if (send_divert) begin
      reply_to_send.plane_id <= uart_request.plane_id;
      reply_to_send.msg_type <= T_DIVERT;
    end
  end

  /////////////////////////////
  // UART Reply Storage FIFO //
  /////////////////////////////

  logic queue_reply, send_reply;

  FIFO #(.WIDTH(9), .DEPTH(15)) uart_replies(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(reply_to_send),
    .we(queue_reply),
    .re(send_reply),
    .data_out(uart_tx_data),
    .full(),
    .empty()
  );

endmodule : Bob

module ReadRequestFSM
  (input  logic clock, reset_n,
   input  logic uart_empty,
   input  msg_t uart_request,
   input  logic takeoff_fifo_full,
   input  logic landing_fifo_full,
   output logic uart_rd_request,
   output logic queue_takeoff_plane,
   output logic queue_landing_plane,
   output logic send_clear,
   output logic send_hold,
   output logic send_say_ag,
   output logic send_divert);

  msg_type_t  msg_type;
  logic [1:0] msg_action;
  
  assign msg_type   = uart_request.msg_type;
  assign msg_action = uart_request.msg_action;

  enum logic [3:0] {QUIET, 
                    INTERPRET, 
                    HOLD, 
                    DIVERT,
                    SAY_AGAIN, 
                    DECLARE,
                    CLR_TAKEOFF,
                    CLR_LANDING} state, next_state;

  always_comb begin
    case (state) 
      QUIET: begin
        if (uart_empty) 
          next_state      = QUIET;
        else begin
          next_state      = INTERPRET;
          uart_rd_request = 1'b1;
        end
      end
      INTERPRET: begin
        if (msg_type == T_REQUEST) begin
          if (msg_action == 2'b0x) begin
            // Trying to take off, and if fifo full, just deny.
            if (takeoff_fifo_full) begin
              next_state          = DIVERT;
              send_divert         = 1'b1;
            end else begin
              next_state          = HOLD;
              queue_takeoff_plane = 1'b1;
              send_hold           = 1'b1;
            end 
          end else if (msg_action == 2'b1x) begin
            // Trying to land, and if fifo full, just deny.
            if (landing_fifo_full) begin
              next_state          = DIVERT;
              send_divert         = 1'b1;
            end else begin
              next_state          = HOLD;
              queue_landing_plane = 1'b1;
              send_hold           = 1'b1;
            end 
          end
        end else if (msg_type == T_DECLARE) begin
          // Free up runway
        end else if (msg_type == T_EMERGENCY) begin
          // Lock both runways until response received
        end else if (msg_type == T_POSITION) begin
          // Switch frequency
        end else begin
          // Message invalid (100 and above) say again
          next_state  = SAY_AGAIN;
          send_say_ag = 1'b1;
        end
      end
      HOLD: begin
      end
      DIVERT: begin
      end
      SAY_AGAIN: begin
      end
      DECLARE: begin
      end
      CLR_TAKEOFF: begin
      end
      CLR_LANDING: begin
      end
      default: begin
        uart_rd_request     = 1'b0;
        queue_takeoff_plane = 1'b0;
        queue_landing_plane = 1'b0;
        send_clear          = 1'b0;
        send_hold           = 1'b0;
        send_say_ag         = 1'b0;
        send_divert         = 1'b0;
      end
    endcase
  end

  always_ff @(posedge clock, negedge reset_n)
    if (~reset_n) 
      state <= LISTEN;
    else 
      state <= next_state;

endmodule : ReadRequestFSM

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
  assign full  = (count == DEPTH);

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      count   <= 0;
      get_ptr <= 0;
      put_ptr <= 0;
    end
    else begin
      // If reading (and not empty) and writing, first read and then write
      if ((re && !empty) && we) begin
        data_out       <= queue[get_ptr];
        get_ptr        <= get_ptr + 1;
        queue[put_ptr] <= data_in;
        put_ptr        <= put_ptr + 1;
      end else if (actually_re && (!empty)) begin
        data_out       <= queue[get_ptr];
        get_ptr        <= get_ptr + 1;
        count          <= count - 1;
      end else if (we && (!full)) begin
        queue[put_ptr] <= data_in;
        put_ptr        <= put_ptr + 1;
        count          <= count + 1;
      end
    end
  end

endmodule : FIFO