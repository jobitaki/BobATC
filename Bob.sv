////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//   --  --  --  --  --  --  --  --  BobATC  --  --  --  --  --  --  --  --   //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

`default_nettype none

import BobATC::*;

module BobTop (
    input  logic       clock,
    input  logic       reset,
    input  logic       rx,
    output logic       tx,
    output logic       framing_error,
    output logic [1:0] runway_active
);

  logic [8:0] uart_rx_data, uart_tx_data;
  logic uart_rx_valid;
  logic uart_tx_ready;
  logic uart_tx_send;

  UartRX receiver (
      .clock(clock),
      .reset(reset),
      .rx(rx),
      .data(uart_rx_data),
      .done(uart_rx_valid),
      .framing_error(framing_error)
  );

  UartTX transmitter (
      .clock(clock),
      .reset(reset),
      .send(uart_tx_send),
      .data(uart_tx_data),
      .tx(tx),
      .ready(uart_tx_ready)
  );

  Bob bobby (
      .clock(clock),
      .reset(reset),
      .uart_rx_data(uart_rx_data),
      .uart_rx_valid(uart_rx_valid),
      .uart_tx_data(uart_tx_data),
      .uart_tx_ready(uart_tx_ready),
      .uart_tx_send(uart_tx_send),
      .runway_active(runway_active)
  );

endmodule : BobTop

module Bob (
    input  logic       clock,
    input  logic       reset,
    input  logic [8:0] uart_rx_data,   // Data from UART
    input  logic       uart_rx_valid,  // High if data is ready to be read
    output logic [8:0] uart_tx_data,   // Data to write to UART
    input  logic       uart_tx_ready,  // High if ready to write to UART
    output logic       uart_tx_send,   // High if data is ready for transmit
    output logic [1:0] runway_active   // Tracks runway status
);

  // For UART Request Storage FIFO
  msg_t uart_request;
  logic uart_rd_request;
  logic uart_empty;

  // For RunwayManager
  logic runway_id;
  logic lock, unlock;
  logic [3:0] cleared_id_to_lock;
  logic sel_takeoff_id_lock;

  // For Aircraft Takeoff FIFO
  logic queue_takeoff_plane, unqueue_takeoff_plane;
  logic [3:0] cleared_takeoff_id;
  logic takeoff_fifo_full, takeoff_fifo_empty;

  // For Aircraft Landing FIFO
  logic queue_landing_plane, unqueue_landing_plane;
  logic [3:0] cleared_landing_id;
  logic landing_fifo_full, landing_fifo_empty;

  // For Reply Generation
  logic send_hold, send_say_ag, send_divert, send_divert_landing;
  logic send_valid_id, send_invalid_id;
  logic [1:0] send_clear;
  msg_t reply_to_send;

  // For UART Reply Storage FIFO
  logic send_reply, queue_reply;
  logic reply_fifo_full, reply_fifo_empty;

  // For emergency latching
  logic set_emergency, unset_emergency, emergency;
  logic [3:0] emergency_id;

  ///////////////////////////////
  // UART Request Storage FIFO //
  ///////////////////////////////

  FIFO #(
      .WIDTH(9),
      .DEPTH(4)
  ) uart_requests (
      .clock(clock),
      .reset(reset),
      .data_in(uart_rx_data),
      .we(uart_rx_valid),
      .re(uart_rd_request),
      .data_out({uart_request.plane_id, uart_request.msg_type, uart_request.msg_action}),
      .full(),
      .empty(uart_empty)
  );

  ////////////////////////////
  // Aircraft Take-Off FIFO //
  ////////////////////////////

  FIFO #(
      .WIDTH(4),
      .DEPTH(4)
  ) takeoff_fifo (
      .clock(clock),
      .reset(reset),
      .data_in(uart_request.plane_id),
      .we(queue_takeoff_plane),
      .re(unqueue_takeoff_plane),
      .data_out(cleared_takeoff_id),
      .full(takeoff_fifo_full),
      .empty(takeoff_fifo_empty)
  );

  ///////////////////////////
  // Aircraft Landing FIFO //
  ///////////////////////////

  FIFO #(
      .WIDTH(4),
      .DEPTH(4)
  ) landing_fifo (
      .clock(clock),
      .reset(reset),
      .data_in(uart_request.plane_id),
      .we(queue_landing_plane),
      .re(unqueue_landing_plane),
      .data_out(cleared_landing_id),
      .full(landing_fifo_full),
      .empty(landing_fifo_empty)
  );

  ////////////////
  // ID Manager //
  ////////////////

  logic [3:0] new_id, id_in;
  logic sel_diverted_id;
  logic take_id, release_id;
  logic id_full;
  logic [15:0] all_id;

  always_comb
    if (sel_diverted_id) id_in = cleared_landing_id;
    else id_in = uart_request.plane_id;

  AircraftIDManager id_manager (
      .clock(clock),
      .reset(reset),
      .id_in(id_in),
      .release_id(release_id),
      .take_id(take_id),
      .id_out(new_id),
      .all_id(all_id),
      .full(id_full)
  );

  /////////
  // FSM //
  /////////

  ReadRequestFsm fsm (.*);

  /////////////////////
  // Reply Generator //
  /////////////////////

  always_ff @(posedge clock) begin
    if (reset) begin
      reply_to_send <= 0;
    end else if (send_clear[0] ^ send_clear[1]) begin
      // "send_clear" is two signals, one for takeoff and one for landing.
      // It is one-hot encoded, hence the XOR evaluator.
      if (send_clear[0]) begin  // Cleared for takeoff
        reply_to_send.plane_id   <= cleared_takeoff_id;
        reply_to_send.msg_type   <= T_CLEAR;
        reply_to_send.msg_action <= {1'b0, runway_id};
      end else if (send_clear[1]) begin  // Cleared for landing
        reply_to_send.plane_id   <= cleared_landing_id;
        reply_to_send.msg_type   <= T_CLEAR;
        reply_to_send.msg_action <= {1'b1, runway_id};
      end
    end else if (send_hold) begin
      reply_to_send.plane_id   <= uart_request.plane_id;
      reply_to_send.msg_type   <= T_HOLD;
      reply_to_send.msg_action <= 2'b00;
    end else if (send_say_ag) begin
      reply_to_send.plane_id   <= uart_request.plane_id;
      reply_to_send.msg_type   <= T_SAY_AGAIN;
      reply_to_send.msg_action <= 2'b00;
    end else if (send_divert) begin
      reply_to_send.plane_id   <= uart_request.plane_id;
      reply_to_send.msg_type   <= T_DIVERT;
      reply_to_send.msg_action <= 2'b00;
    end else if (send_divert_landing) begin
      reply_to_send.plane_id   <= cleared_landing_id;
      reply_to_send.msg_type   <= T_DIVERT;
      reply_to_send.msg_action <= 2'b00;
    end else if (send_invalid_id) begin
      reply_to_send.plane_id   <= 4'd0;
      reply_to_send.msg_type   <= T_ID_PLEASE;
      reply_to_send.msg_action <= 2'b11;
    end else if (send_valid_id) begin
      reply_to_send.plane_id   <= new_id;
      reply_to_send.msg_type   <= T_ID_PLEASE;
      reply_to_send.msg_action <= 2'b00;
    end
  end

  /////////////////////////////
  // UART Reply Storage FIFO //
  /////////////////////////////

  FIFO #(
      .WIDTH(9),
      .DEPTH(4)
  ) uart_replies (
      .clock(clock),
      .reset(reset),
      .data_in(reply_to_send),
      .we(queue_reply),
      .re(send_reply),
      .data_out(uart_tx_data),
      .full(reply_fifo_full),
      .empty(reply_fifo_empty)
  );

  SendReplyFsm reply_fsm (
      .clock(clock),
      .reset(reset),
      .uart_tx_ready(uart_tx_ready),
      .reply_fifo_empty(reply_fifo_empty),
      .send_reply(send_reply),
      .uart_tx_send(uart_tx_send)
  );

  always_comb
    if (sel_takeoff_id_lock) cleared_id_to_lock = cleared_takeoff_id;
    else cleared_id_to_lock = cleared_landing_id;

  RunwayManager runway_manager (
      .clock(clock),
      .reset(reset),
      .plane_id_unlock(uart_request.plane_id),
      .plane_id_lock(cleared_id_to_lock),
      .runway_id(runway_id),
      .lock(lock),
      .unlock(unlock),
      .runway_active(runway_active)
  );

  always_ff @(posedge clock) begin
    if (reset) begin
      emergency_id <= 4'd0;
      emergency    <= 1'b0;
    end else if (set_emergency) begin
      emergency_id <= uart_request.plane_id;
      emergency    <= 1'b1;
    end else if (unset_emergency) begin
      emergency <= 1'b0;
    end
  end

endmodule : Bob

module ReadRequestFsm (
    input  logic        clock,
    input  logic        reset,
    input  logic        uart_empty,
    input  msg_t        uart_request,
    input  logic        takeoff_fifo_full,
    input  logic        landing_fifo_full,
    input  logic        takeoff_fifo_empty,
    input  logic        landing_fifo_empty,
    input  logic        reply_fifo_full,
    input  logic [ 1:0] runway_active,
    input  logic        emergency,
    input  logic [15:0] all_id,
    input  logic        id_full,
    input  logic [ 3:0] emergency_id,
    output logic        uart_rd_request,
    output logic        queue_takeoff_plane,
    output logic        queue_landing_plane,
    output logic        unqueue_takeoff_plane,
    output logic        unqueue_landing_plane,
    output logic [ 1:0] send_clear,
    output logic        send_hold,
    output logic        send_say_ag,
    output logic        send_divert,
    output logic        send_divert_landing,
    output logic        send_invalid_id,
    output logic        send_valid_id,
    output logic        queue_reply,
    output logic        lock,
    output logic        unlock,
    output logic        runway_id,
    output logic        set_emergency,
    output logic        unset_emergency,
    output logic        take_id,
    output logic        release_id,
    output logic        sel_takeoff_id_lock,
    output logic        sel_diverted_id
);

  logic      [3:0] plane_id;
  msg_type_t       msg_type;
  logic      [1:0] msg_action;
  logic            takeoff_first;

  assign plane_id   = uart_request.plane_id;
  assign msg_type   = uart_request.msg_type;
  assign msg_action = uart_request.msg_action;

  typedef enum logic [2:0] {
    QUIET          = 3'b000,
    INTERPRET      = 3'b001,
    REPLY          = 3'b010,
    CHECK_QUEUES   = 3'b011,
    CLR_TAKEOFF    = 3'b100,
    CLR_LANDING    = 3'b101,
    DIVERT_LANDING = 3'b110,
    QUEUE_CLR      = 3'b111
  } state_t;

  state_t state, next_state;

  always_comb begin
    uart_rd_request       = 1'b0;
    queue_takeoff_plane   = 1'b0;
    queue_landing_plane   = 1'b0;
    unqueue_takeoff_plane = 1'b0;
    unqueue_landing_plane = 1'b0;
    send_clear            = 2'b00;
    send_hold             = 1'b0;
    send_say_ag           = 1'b0;
    send_divert           = 1'b0;
    send_divert_landing   = 1'b0;
    send_invalid_id       = 1'b0;
    send_valid_id         = 1'b0;
    queue_reply           = 1'b0;
    lock                  = 1'b0;
    unlock                = 1'b0;
    runway_id             = 1'b0;
    set_emergency         = 1'b0;
    unset_emergency       = 1'b0;
    take_id               = 1'b0;
    release_id            = 1'b0;
    sel_takeoff_id_lock   = 1'b0;
    sel_diverted_id       = 1'b0;

    case (state)
      QUIET: begin
        if (uart_empty) begin
          if (emergency) begin
            if (!landing_fifo_empty) begin
              next_state            = DIVERT_LANDING;
              unqueue_landing_plane = 1'b1;
            end else next_state = QUIET;
          end else if (runway_active != 2'b11) begin  // If runways aren't full
            if (!takeoff_fifo_empty && !landing_fifo_empty) begin
              if (takeoff_first) begin
                next_state            = CLR_TAKEOFF;
                unqueue_takeoff_plane = 1'b1;
              end else begin
                next_state            = CLR_LANDING;
                unqueue_landing_plane = 1'b1;
              end
            end else if (!takeoff_fifo_empty) begin
              next_state            = CLR_TAKEOFF;
              unqueue_takeoff_plane = 1'b1;
            end else if (!landing_fifo_empty) begin
              next_state            = CLR_LANDING;
              unqueue_landing_plane = 1'b1;
            end else begin
              next_state = QUIET;
            end
          end else begin
            next_state = QUIET;  // If runways are full don't clear anything
          end
        end else begin
          next_state      = INTERPRET;
          uart_rd_request = 1'b1;
        end
      end

      INTERPRET: begin
        if (msg_type == T_REQUEST) begin  // REQUEST
          if (all_id[plane_id]) begin  // If ID is taken
            next_state = REPLY;
            if (msg_action[1] == 1'b0) begin
              // Trying to take off, and if fifo full, just deny.
              if (takeoff_fifo_full) begin
                send_divert = 1'b1;
                release_id  = 1'b1;
              end else begin
                queue_takeoff_plane = 1'b1;
                send_hold           = 1'b1;
              end
            end else if (msg_action[1] == 1'b1) begin
              // Trying to land, and if fifo full or emergency, just deny.
              if (landing_fifo_full || emergency) begin
                send_divert = 1'b1;
                release_id  = 1'b1;
              end else begin
                queue_landing_plane = 1'b1;
                send_hold           = 1'b1;
              end
            end
          end else begin  // ID is unknown, go straight to CHECK_QUEUES
            next_state = CHECK_QUEUES;
          end
        end else if (msg_type == T_DECLARE) begin  // DECLARE
          if (all_id[plane_id]) begin
            next_state = CHECK_QUEUES;
            release_id = 1'b1;  // Release ID
            if (!msg_action[1]) begin
              // Declaring take off
              if (!msg_action[0]) begin
                // Runway 0
                unlock    = 1'b1;
                runway_id = 1'b0;
              end else if (msg_action[0]) begin
                // Runway 1
                unlock    = 1'b1;
                runway_id = 1'b1;
              end
            end else if (msg_action[1]) begin
              // Declaring landing
              if (!msg_action[0]) begin
                // Runway 0
                unlock    = 1'b1;
                runway_id = 1'b0;
              end else if (msg_action[0]) begin
                // Runway 1
                unlock    = 1'b1;
                runway_id = 1'b1;
              end
            end
          end else begin
            // Invalid plane ID
            next_state = CHECK_QUEUES;
          end
        end else if (msg_type == T_EMERGENCY) begin  // EMERGENCY
          // Mayday shouldn't lock any runways but simply prevent
          // clearing of landings and takeoffs. It should send out
          // divertions to all landings.
          // It should still unlock runways normally.
          // It will not send out a special message of any kind
          if (msg_action == 2'b01) begin
            if (!landing_fifo_empty) begin
              next_state            = DIVERT_LANDING;  // Divert all landings
              unqueue_landing_plane = 1'b1;
            end else next_state = QUIET;
            // Declare emergency
            set_emergency = 1'b1;
          end else if (msg_action == 2'b00) begin
            next_state = CHECK_QUEUES;
            // Resolve emergency only if the resolver is the original
            if (emergency_id == plane_id) unset_emergency = 1'b1;
            // Also release ID
            release_id = 1'b1;
          end
        end else if (msg_type == T_ID_PLEASE) begin  // ID_PLEASE
          // When full, send action bits 11 to invalidate ID
          next_state = REPLY;
          if (id_full) begin
            send_invalid_id = 1'b1;
          end else begin
            send_valid_id = 1'b1;
            take_id       = 1'b1;
          end
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
          next_state  = CHECK_QUEUES;
          queue_reply = 1'b1;
        end
      end

      CHECK_QUEUES: begin
        if (emergency) begin
          if (!landing_fifo_empty) begin
            next_state            = DIVERT_LANDING;
            unqueue_landing_plane = 1'b1;
          end else next_state = QUIET;
        end else if (runway_active != 2'b11) begin  // If runways aren't full
          if (!takeoff_fifo_empty && !landing_fifo_empty) begin
            if (takeoff_first) begin
              next_state            = CLR_TAKEOFF;
              unqueue_takeoff_plane = 1'b1;
            end else begin
              next_state            = CLR_LANDING;
              unqueue_landing_plane = 1'b1;
            end
          end else if (!takeoff_fifo_empty) begin
            next_state            = CLR_TAKEOFF;
            unqueue_takeoff_plane = 1'b1;
          end else if (!landing_fifo_empty) begin
            next_state            = CLR_LANDING;
            unqueue_landing_plane = 1'b1;
          end else begin
            next_state = QUIET;
          end
        end else begin
          next_state = QUIET;  // If runways are full don't clear anything
        end
      end

      CLR_TAKEOFF: begin
        next_state = QUEUE_CLR;
        sel_takeoff_id_lock = 1'b1;
        if (!runway_active[0]) begin
          // Place lock on runway 0
          runway_id  = 1'b0;
          lock       = 1'b1;
          send_clear = 2'b01;
        end else if (!runway_active[1]) begin
          runway_id  = 1'b1;
          lock       = 1'b1;
          send_clear = 2'b01;
        end
      end

      CLR_LANDING: begin
        next_state = QUEUE_CLR;
        if (!runway_active[0]) begin
          // Place lock on runway 0
          runway_id  = 1'b0;
          lock       = 1'b1;
          send_clear = 2'b10;
        end else if (!runway_active[1]) begin
          runway_id  = 1'b1;
          lock       = 1'b1;
          send_clear = 2'b10;
        end
      end

      DIVERT_LANDING: begin
        next_state          = QUEUE_CLR;
        send_divert_landing = 1'b1;
        sel_diverted_id     = 1'b1;
        release_id          = 1'b1;
      end

      QUEUE_CLR: begin
        next_state  = QUIET;
        queue_reply = 1'b1;
      end

      default: next_state = QUIET;
    endcase
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      state         <= QUIET;
      takeoff_first <= 1'b0;
    end else begin
      state         <= next_state;
      takeoff_first <= ~takeoff_first;
    end
  end

endmodule : ReadRequestFsm

module SendReplyFsm (
    input  logic clock,
    input  logic reset,
    input  logic uart_tx_ready,
    input  logic reply_fifo_empty,
    output logic send_reply,
    output logic uart_tx_send
);

  typedef enum logic {
    WAIT,
    SEND
  } state_t;

  state_t state, next_state;

  always_comb begin
    send_reply   = 1'b0;
    uart_tx_send = 1'b0;
    unique case (state)
      WAIT: begin
        if (reply_fifo_empty || !uart_tx_ready) next_state = WAIT;
        else begin
          next_state = SEND;
          send_reply = 1'b1;
        end
      end

      SEND: begin
        next_state   = WAIT;
        uart_tx_send = 1'b1;
      end

      default: next_state = WAIT;
    endcase
  end

  always_ff @(posedge clock) begin
    if (reset) state <= WAIT;
    else state <= next_state;
  end

endmodule : SendReplyFsm

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
module FIFO #(
    parameter int WIDTH = 9,
    int DEPTH = 4
) (
    input  logic             clock,
    input  logic             reset,
    input  logic [WIDTH-1:0] data_in,
    input  logic             we,
    input  logic             re,
    output logic [WIDTH-1:0] data_out,
    output logic             full,
    empty
);

  logic [DEPTH-1:0][WIDTH-1:0] queue;
  logic [$clog2(DEPTH):0] count;
  logic [$clog2(DEPTH)-1:0] put_ptr, get_ptr;

  assign empty = (count == 0);
  assign full  = (count == DEPTH);

  always_ff @(posedge clock) begin
    if (reset) begin
      count    <= 0;
      get_ptr  <= 0;
      put_ptr  <= 0;
      data_out <= 0;
    end else begin
      // If reading (and not empty) and writing, first read and then write
      if ((re && !empty) && we) begin
        data_out       <= queue[get_ptr];
        get_ptr        <= get_ptr + 1;
        queue[put_ptr] <= data_in;
        put_ptr        <= put_ptr + 1;
      end else if (re && (!empty)) begin
        data_out <= queue[get_ptr];
        get_ptr  <= get_ptr + 1;
        count    <= count - 1;
      end else if (we && (!full)) begin
        queue[put_ptr] <= data_in;
        put_ptr        <= put_ptr + 1;
        count          <= count + 1;
      end
    end
  end

endmodule : FIFO

//
//  Module 'RunwayManager'
//
//  A set of registers that manage runway activity status to prevent runway
//  usage conflicts.
//    - When lock is asserted, the runway indicated by the runway_id input
//      will be locked and associated with the plane_id input.
//    - When unlock is asserted, the runway indicated by the runway_id input
//      will be unlocked only if the plane_id on the input is equal.
//
module RunwayManager (
    input  logic       clock,
    input  logic       reset,
    input  logic [3:0] plane_id_unlock,
    input  logic [3:0] plane_id_lock,
    input  logic       runway_id,
    input  logic       lock,
    input  logic       unlock,
    output logic [1:0] runway_active
);

  // Register that contain the current status of each runway
  // Contains 4 bits of plane ID followed by runway status (1 for lock)
  runway_t [1:0] runway;

  assign runway_active[0] = runway[0].active;
  assign runway_active[1] = runway[1].active;

  always_ff @(posedge clock) begin
    if (reset) begin
      runway <= 0;
    end else begin
      if (lock && !unlock) begin
        if (runway_id) begin
          runway[1].plane_id <= plane_id_lock;
          runway[1].active   <= 1'b1;
        end else begin
          runway[0].plane_id <= plane_id_lock;
          runway[0].active   <= 1'b1;
        end
      end else if (!lock && unlock) begin
        if (runway_id) begin
          // Prevent other planes from unlocking runway
          if (plane_id_unlock == runway[1].plane_id) runway[1].active <= 1'b0;
        end else begin
          if (plane_id_unlock == runway[0].plane_id) runway[0].active <= 1'b0;
        end
      end
    end
  end

endmodule : RunwayManager

//
//  Module 'AircraftIDManager'
//
//  Keeps track of 16 IDs that can be assigned to aircraft entering the airspace
//
module AircraftIDManager (
    input  logic        clock,       //! Clock signal
    input  logic        reset,       //! Reset signal
    input  logic [ 3:0] id_in,       //! Incoming ID to release
    input  logic        release_id,  //! Assert for one cycle to free id_in ID
    input  logic        take_id,     //! Assert for one cycle to claim id_out ID
    output logic [ 3:0] id_out,      //! ID that is currently available
    output logic [15:0] all_id,      //! Vector indicates which IDs are taken
    output logic        full         //! High if all IDs are taken
);

  // TODO when a request is not made by ID in certain number of interpret
  // cycles, give up ID to someone else. 100 cycles?

  logic [15:0] taken_id;
  logic [ 3:0] id_avail;

  assign all_id = taken_id;
  assign id_out = id_avail;  // always available
  assign full   = taken_id == 16'hFFFF;

  always_comb begin
    id_avail = 4'd0;

    case (1'b0)
      taken_id[0]:  id_avail = 4'd0;
      taken_id[1]:  id_avail = 4'd1;
      taken_id[2]:  id_avail = 4'd2;
      taken_id[3]:  id_avail = 4'd3;
      taken_id[4]:  id_avail = 4'd4;
      taken_id[5]:  id_avail = 4'd5;
      taken_id[6]:  id_avail = 4'd6;
      taken_id[7]:  id_avail = 4'd7;
      taken_id[8]:  id_avail = 4'd8;
      taken_id[9]:  id_avail = 4'd9;
      taken_id[10]: id_avail = 4'd10;
      taken_id[11]: id_avail = 4'd11;
      taken_id[12]: id_avail = 4'd12;
      taken_id[13]: id_avail = 4'd13;
      taken_id[14]: id_avail = 4'd14;
      taken_id[15]: id_avail = 4'd15;
      default:      id_avail = 4'd0;
    endcase
  end

  always_ff @(posedge clock) begin
    if (reset) begin
      taken_id <= '0;
    end else if (release_id) begin
      taken_id[id_in] <= 1'b0;
    end else if (take_id && !full) begin
      taken_id[id_avail] <= 1'b1;
    end
  end

endmodule : AircraftIDManager
