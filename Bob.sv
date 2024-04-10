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
   output logic [8:0] uart_tx_data,   // Data to write to UART
   input  logic       uart_tx_ready,  // High if ready to write to UART
   output logic       uart_tx_send,   // High if data is ready for transmit
   output logic       bob_busy);      // High if Bob has too many requests
  
  // For UART Request Storage FIFO
  msg_t uart_request;
  logic uart_rd_request;
  logic uart_empty;

  // For RunwayManager
  logic runway_id;
  logic lock, unlock;
  logic [1:0] runway_active;

  // For Aircraft Takeoff FIFO
  logic queue_takeoff_plane, clear_takeoff_plane;
  logic [3:0] cleared_takeoff_id;
  logic takeoff_fifo_full, takeoff_fifo_empty;

  // For Aircraft Landing FIFO
  logic queue_landing_plane, clear_landing_plane;
  logic [3:0] cleared_landing_id;
  logic landing_fifo_full, landing_fifo_empty;

  // For Reply Generation
  logic send_hold, send_say_ag, send_divert;
  logic [1:0] send_clear;
  msg_t reply_to_send;

  // For UART Reply Storage FIFO
  logic send_reply, queue_reply;
  logic reply_fifo_full, reply_fifo_empty;

  ///////////////////////////////
  // UART Request Storage FIFO //
  ///////////////////////////////

  FIFO #(.WIDTH(9), .DEPTH(4)) uart_requests(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_rx_data),
    .we(uart_rx_valid),
    .re(uart_rd_request),
    .data_out({
      uart_request.plane_id,
      uart_request.msg_type,
      uart_request.msg_action
    }),
    .full(bob_busy),
    .empty(uart_empty)
  );

  ////////////////////////////
  // Aircraft Take-Off FIFO //
  ////////////////////////////

  FIFO #(.WIDTH(4), .DEPTH(4)) takeoff_fifo(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_request.plane_id),
    .we(queue_takeoff_plane),
    .re(clear_takeoff_plane),
    .data_out(cleared_takeoff_id),
    .full(takeoff_fifo_full),
    .empty(takeoff_fifo_empty)
  );

  ///////////////////////////
  // Aircraft Landing FIFO //
  ///////////////////////////
  
  FIFO #(.WIDTH(4), .DEPTH(4)) landing_fifo(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(uart_request.plane_id),
    .we(queue_landing_plane),
    .re(clear_landing_plane),
    .data_out(cleared_landing_id),
    .full(landing_fifo_full),
    .empty(landing_fifo_empty)
  );

  /////////
  // FSM //
  /////////

  ReadRequestFSM fsm(.*);

  /////////////////////
  // Reply Generator //
  /////////////////////

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      reply_to_send <= 0;
    end else if (send_clear[0] ^ send_clear[1]) begin
      if (send_clear[0]) begin
        reply_to_send.plane_id   <= cleared_takeoff_id;
        reply_to_send.msg_type   <= T_CLEAR;
        reply_to_send.msg_action <= {1'b0, runway_id};
      end else if (send_clear[1]) begin
        reply_to_send.plane_id   <= cleared_landing_id;
        reply_to_send.msg_type   <= T_CLEAR;
        reply_to_send.msg_action <= {1'b0, runway_id};
      end
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

  FIFO #(.WIDTH(9), .DEPTH(4)) uart_replies(
    .clock(clock),
    .reset_n(reset_n),
    .data_in(reply_to_send),
    .we(queue_reply),
    .re(send_reply),
    .data_out(uart_tx_data),
    .full(reply_fifo_full),
    .empty(reply_fifo_empty)
  );

  SendReplyFSM reply_fsm(
    .clock(clock),
    .reset_n(reset_n),
    .uart_tx_ready(uart_tx_ready),
    .reply_fifo_empty(reply_fifo_empty),
    .send_reply(send_reply),
    .uart_tx_send(uart_tx_send)
  );

  RunwayManager manager(
    .clock(clock),
    .reset_n(reset_n),
    .plane_id(uart_request.plane_id),
    .runway_id(runway_id),
    .lock(lock),
    .unlock(unlock),
    .runway_active(runway_active)
  );

endmodule : Bob

module ReadRequestFSM
  (input  logic       clock, reset_n,
   input  logic       uart_empty,
   input  msg_t       uart_request,
   input  logic       takeoff_fifo_full,
   input  logic       landing_fifo_full,
   input  logic       takeoff_fifo_empty,
   input  logic       landing_fifo_empty,
   input  logic       reply_fifo_full,
   input  logic [1:0] runway_active,
   output logic       uart_rd_request,
   output logic       queue_takeoff_plane,
   output logic       queue_landing_plane,
   output logic [1:0] send_clear,
   output logic       send_hold,
   output logic       send_say_ag,
   output logic       send_divert,
   output logic       queue_reply,
   output logic       lock, unlock,
   output logic       runway_id);

  msg_type_t  msg_type;
  logic [1:0] msg_action;
  logic       takeoff_first;
  
  assign msg_type   = uart_request.msg_type;
  assign msg_action = uart_request.msg_action;

  enum logic [2:0] {QUIET, 
                    INTERPRET, 
                    REPLY, 
                    CLR_TAKEOFF,
                    CLR_LANDING} state, next_state;

  always_comb begin
    uart_rd_request     = 1'b0;
    queue_takeoff_plane = 1'b0;
    queue_landing_plane = 1'b0;
    send_clear          = 2'b00;
    send_hold           = 1'b0;
    send_say_ag         = 1'b0;
    send_divert         = 1'b0;
    queue_reply         = 1'b0;
    lock                = 1'b0;
    unlock              = 1'b0;
    runway_id           = 1'b0;
    unique case (state) 
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
          next_state = REPLY;
          if (msg_action == 2'b0x) begin
            // Trying to take off, and if fifo full, just deny.
            if (takeoff_fifo_full) begin
              send_divert         = 1'b1;
            end else begin
              queue_takeoff_plane = 1'b1;
              send_hold           = 1'b1;
            end 
          end else if (msg_action == 2'b1x) begin
            // Trying to land, and if fifo full, just deny.
            if (landing_fifo_full) begin
              send_divert         = 1'b1;
            end else begin
              queue_landing_plane = 1'b1;
              send_hold           = 1'b1;
            end 
          end
        end else if (msg_type == T_DECLARE) begin
          // Free up runway
          next_state = (takeoff_first) ? CLR_TAKEOFF : CLR_LANDING;
        end else if (msg_type == T_EMERGENCY) begin
          // Lock both runways until response received
        end else if (msg_type == T_POSITION) begin
          // Say Again
          next_state  = REPLY;
          send_say_ag = 1'b1;
        end else begin
          // Message invalid (100 and above) say again
          next_state  = REPLY;
          send_say_ag = 1'b1;
        end
      end
      REPLY: begin
        if (reply_fifo_full) begin
          next_state = REPLY;
        end else begin
          next_state = (takeoff_first) ? CLR_TAKEOFF : CLR_LANDING;
          queue_reply = 1'b1;
        end
      end
      CLR_TAKEOFF: begin
        next_state = QUIET;
        if (~takeoff_fifo_empty) begin
          if (~runway_active[0]) begin
            // Place lock on runway 0
            // TODO before coming here do read from fifo
            runway_id    = 1'b0;
            lock         = 1'b1;
            send_clear   = 2'b01;
          end else if (~runway_active[1]) begin
            runway_id    = 1'b1;
            lock         = 1'b1;
            send_clear   = 2'b01;
          end
        end
      end
      CLR_LANDING: begin
        next_state = QUIET;
        if (~landing_fifo_empty) begin
          if (~runway_active[0]) begin
            // Place lock on runway 0
            runway_id    = 1'b0;
            lock         = 1'b1;
            send_clear   = 2'b10;
          end else if (~runway_active[1]) begin
            runway_id    = 1'b1;
            lock         = 1'b1;
            send_clear   = 2'b10;
          end
        end
      end
    endcase
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      state           <= QUIET;
      takeoff_first   <= 1'b0;
    end else begin
      state           <= next_state;
      if (takeoff_fifo_empty) begin
        takeoff_first <= 1'b0;
      end else if (landing_fifo_empty) begin
        takeoff_first <= 1'b1; 
      end else begin
        takeoff_first <= ~takeoff_first;
      end
    end
  end

endmodule : ReadRequestFSM

module SendReplyFSM
  (input  logic clock, reset_n,
   input  logic uart_tx_ready,
   input  logic reply_fifo_empty,
   output logic send_reply,
   output logic uart_tx_send);

  enum logic {WAIT, SEND} state, next_state;

  always_comb begin
    send_reply   = 1'b0;
    uart_tx_send = 1'b0;
    unique case (state)
      WAIT: begin
        if (reply_fifo_empty | ~uart_tx_ready) 
          next_state = WAIT;
        else begin
          next_state = SEND;
          send_reply = 1'b1;
        end
      end
      SEND: begin
        next_state   = WAIT;
        uart_tx_send = 1'b1;
      end
    endcase
  end

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) 
      state <= WAIT;
    else 
      state <= next_state;
  end

endmodule : SendReplyFSM

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
      end else if (re && (!empty)) begin
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

//
// Module 'RunwayManager'
// 
// - When lock is asserted, the runway indicated by the runway_id input
//   will be locked and associated with the plane_id input. 
// - When unlock is asserted, the runway indicated by the runway_id input
//   will be unlocked only if the plane_id on the input is equal. 
//
module RunwayManager
  (input  logic       clock, reset_n,
   input  logic [3:0] plane_id,
   input  logic       runway_id,
   input  logic       lock,
   input  logic       unlock,
   output logic [1:0] runway_active);

  // Register that contain the current status of each runway
  // Contains 4 bits of plane ID followed by runway status (1 for lock)
  runway_t [1:0] runway;

  assign runway_active[0] = runway[0].active;
  assign runway_active[1] = runway[1].active;

  always_ff @(posedge clock, negedge reset_n) begin
    if (~reset_n) begin
      runway[0].active <= 0;
      runway[1].active <= 0;
      runway <= 0;
    end else begin
      if (lock && !unlock) begin
        if (runway_id) begin
          runway[1].plane_id <= plane_id;
          runway[1].active <= 1'b1;
        end else begin
          runway[0].plane_id <= plane_id;
          runway[0].active <= 1'b1;
        end
      end else if (!lock && unlock) begin
        if (runway_id) begin
          // Prevent other planes from unlocking runway
          if (plane_id == runway[1].plane_id)
            runway[1].active <= 1'b0;
        end else begin
          if (plane_id == runway[0].plane_id)
            runway[1].active <= 1'b1;
        end
     end
    end
  end

endmodule : RunwayManager