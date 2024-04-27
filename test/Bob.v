`default_nettype none
module BobTop (
	clock,
	reset,
	rx,
	tx,
	framing_error,
	bob_busy,
	runway_active
);
	input wire clock;
	input wire reset;
	input wire rx;
	output wire tx;
	output wire framing_error;
	output wire bob_busy;
	output wire [1:0] runway_active;
	wire [8:0] uart_rx_data;
	wire [8:0] uart_tx_data;
	wire uart_rx_valid;
	wire uart_tx_ready;
	wire uart_tx_send;
	UartRX receiver(
		.clock(clock),
		.reset(reset),
		.rx(rx),
		.data(uart_rx_data),
		.done(uart_rx_valid),
		.framing_error(framing_error)
	);
	UartTX transmitter(
		.clock(clock),
		.reset(reset),
		.send(uart_tx_send),
		.data(uart_tx_data),
		.tx(tx),
		.ready(uart_tx_ready)
	);
	Bob bobby(
		.clock(clock),
		.reset(reset),
		.uart_rx_data(uart_rx_data),
		.uart_rx_valid(uart_rx_valid),
		.uart_tx_data(uart_tx_data),
		.uart_tx_ready(uart_tx_ready),
		.uart_tx_send(uart_tx_send),
		.bob_busy(bob_busy),
		.runway_active(runway_active)
	);
endmodule
module Bob (
	clock,
	reset,
	uart_rx_data,
	uart_rx_valid,
	uart_tx_data,
	uart_tx_ready,
	uart_tx_send,
	bob_busy,
	runway_active
);
	input wire clock;
	input wire reset;
	input wire [8:0] uart_rx_data;
	input wire uart_rx_valid;
	output wire [8:0] uart_tx_data;
	input wire uart_tx_ready;
	output wire uart_tx_send;
	output wire bob_busy;
	output wire [1:0] runway_active;
	wire [8:0] uart_request;
	wire uart_rd_request;
	wire uart_empty;
	wire runway_id;
	wire lock;
	wire unlock;
	wire queue_takeoff_plane;
	wire unqueue_takeoff_plane;
	wire [3:0] cleared_takeoff_id;
	wire takeoff_fifo_full;
	wire takeoff_fifo_empty;
	wire queue_landing_plane;
	wire unqueue_landing_plane;
	wire [3:0] cleared_landing_id;
	wire landing_fifo_full;
	wire landing_fifo_empty;
	wire send_hold;
	wire send_say_ag;
	wire send_divert;
	wire send_divert_landing;
	wire [1:0] send_clear;
	reg [8:0] reply_to_send;
	wire send_reply;
	wire queue_reply;
	wire reply_fifo_full;
	wire reply_fifo_empty;
	wire set_emergency;
	wire unset_emergency;
	reg emergency;
	FIFO #(
		.WIDTH(9),
		.DEPTH(4)
	) uart_requests(
		.clock(clock),
		.reset(reset),
		.data_in(uart_rx_data),
		.we(uart_rx_valid),
		.re(uart_rd_request),
		.data_out({uart_request[8-:4], uart_request[4-:3], uart_request[1-:2]}),
		.full(bob_busy),
		.empty(uart_empty)
	);
	FIFO #(
		.WIDTH(4),
		.DEPTH(4)
	) takeoff_fifo(
		.clock(clock),
		.reset(reset),
		.data_in(uart_request[8-:4]),
		.we(queue_takeoff_plane),
		.re(unqueue_takeoff_plane),
		.data_out(cleared_takeoff_id),
		.full(takeoff_fifo_full),
		.empty(takeoff_fifo_empty)
	);
	FIFO #(
		.WIDTH(4),
		.DEPTH(4)
	) landing_fifo(
		.clock(clock),
		.reset(reset),
		.data_in(uart_request[8-:4]),
		.we(queue_landing_plane),
		.re(unqueue_landing_plane),
		.data_out(cleared_landing_id),
		.full(landing_fifo_full),
		.empty(landing_fifo_empty)
	);
	ReadRequestFsm fsm(
		.clock(clock),
		.reset(reset),
		.uart_empty(uart_empty),
		.uart_request(uart_request),
		.takeoff_fifo_full(takeoff_fifo_full),
		.landing_fifo_full(landing_fifo_full),
		.takeoff_fifo_empty(takeoff_fifo_empty),
		.landing_fifo_empty(landing_fifo_empty),
		.reply_fifo_full(reply_fifo_full),
		.runway_active(runway_active),
		.emergency(emergency),
		.uart_rd_request(uart_rd_request),
		.queue_takeoff_plane(queue_takeoff_plane),
		.queue_landing_plane(queue_landing_plane),
		.unqueue_takeoff_plane(unqueue_takeoff_plane),
		.unqueue_landing_plane(unqueue_landing_plane),
		.send_clear(send_clear),
		.send_hold(send_hold),
		.send_say_ag(send_say_ag),
		.send_divert(send_divert),
		.send_divert_landing(send_divert_landing),
		.queue_reply(queue_reply),
		.lock(lock),
		.unlock(unlock),
		.runway_id(runway_id),
		.set_emergency(set_emergency),
		.unset_emergency(unset_emergency)
	);
	always @(posedge clock)
		if (reset)
			reply_to_send <= 0;
		else if (send_clear[0] ^ send_clear[1]) begin
			if (send_clear[0]) begin
				reply_to_send[8-:4] <= cleared_takeoff_id;
				reply_to_send[4-:3] <= 3'b011;
				reply_to_send[1-:2] <= {1'b0, runway_id};
			end
			else if (send_clear[1]) begin
				reply_to_send[8-:4] <= cleared_landing_id;
				reply_to_send[4-:3] <= 3'b011;
				reply_to_send[1-:2] <= {1'b1, runway_id};
			end
		end
		else if (send_hold) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b100;
			reply_to_send[1-:2] <= 2'b00;
		end
		else if (send_say_ag) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b101;
			reply_to_send[1-:2] <= 2'b00;
		end
		else if (send_divert) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b110;
			reply_to_send[1-:2] <= 2'b00;
		end
		else if (send_divert_landing) begin
			reply_to_send[8-:4] <= cleared_landing_id;
			reply_to_send[4-:3] <= 3'b110;
			reply_to_send[1-:2] <= 2'b00;
		end
	FIFO #(
		.WIDTH(9),
		.DEPTH(4)
	) uart_replies(
		.clock(clock),
		.reset(reset),
		.data_in(reply_to_send),
		.we(queue_reply),
		.re(send_reply),
		.data_out(uart_tx_data),
		.full(reply_fifo_full),
		.empty(reply_fifo_empty)
	);
	SendReplyFsm reply_fsm(
		.clock(clock),
		.reset(reset),
		.uart_tx_ready(uart_tx_ready),
		.reply_fifo_empty(reply_fifo_empty),
		.send_reply(send_reply),
		.uart_tx_send(uart_tx_send)
	);
	RunwayManager manager(
		.clock(clock),
		.reset(reset),
		.plane_id(uart_request[8-:4]),
		.runway_id(runway_id),
		.lock(lock),
		.unlock(unlock),
		.runway_active(runway_active)
	);
	always @(posedge clock)
		if (reset)
			emergency <= 1'b0;
		else if (set_emergency)
			emergency <= 1'b1;
		else if (unset_emergency)
			emergency <= 1'b0;
endmodule
module ReadRequestFsm (
	clock,
	reset,
	uart_empty,
	uart_request,
	takeoff_fifo_full,
	landing_fifo_full,
	takeoff_fifo_empty,
	landing_fifo_empty,
	reply_fifo_full,
	runway_active,
	emergency,
	uart_rd_request,
	queue_takeoff_plane,
	queue_landing_plane,
	unqueue_takeoff_plane,
	unqueue_landing_plane,
	send_clear,
	send_hold,
	send_say_ag,
	send_divert,
	send_divert_landing,
	queue_reply,
	lock,
	unlock,
	runway_id,
	set_emergency,
	unset_emergency
);
	input wire clock;
	input wire reset;
	input wire uart_empty;
	input wire [8:0] uart_request;
	input wire takeoff_fifo_full;
	input wire landing_fifo_full;
	input wire takeoff_fifo_empty;
	input wire landing_fifo_empty;
	input wire reply_fifo_full;
	input wire [1:0] runway_active;
	input wire emergency;
	output reg uart_rd_request;
	output reg queue_takeoff_plane;
	output reg queue_landing_plane;
	output reg unqueue_takeoff_plane;
	output reg unqueue_landing_plane;
	output reg [1:0] send_clear;
	output reg send_hold;
	output reg send_say_ag;
	output reg send_divert;
	output reg send_divert_landing;
	output reg queue_reply;
	output reg lock;
	output reg unlock;
	output reg runway_id;
	output reg set_emergency;
	output reg unset_emergency;
	wire [2:0] msg_type;
	wire [1:0] msg_action;
	reg takeoff_first;
	assign msg_type = uart_request[4-:3];
	assign msg_action = uart_request[1-:2];
	reg [2:0] state;
	reg [2:0] next_state;
	always @(*) begin
		uart_rd_request = 1'b0;
		queue_takeoff_plane = 1'b0;
		queue_landing_plane = 1'b0;
		unqueue_takeoff_plane = 1'b0;
		unqueue_landing_plane = 1'b0;
		send_clear = 2'b00;
		send_hold = 1'b0;
		send_say_ag = 1'b0;
		send_divert = 1'b0;
		send_divert_landing = 1'b0;
		queue_reply = 1'b0;
		lock = 1'b0;
		unlock = 1'b0;
		runway_id = 1'b0;
		set_emergency = 1'b0;
		unset_emergency = 1'b0;
		case (state)
			3'b000:
				if (uart_empty)
					next_state = 3'b011;
				else begin
					next_state = 3'b001;
					uart_rd_request = 1'b1;
				end
			3'b001:
				if (msg_type == 3'b000) begin
					next_state = 3'b010;
					if (msg_action[1] == 1'b0) begin
						if (takeoff_fifo_full)
							send_divert = 1'b1;
						else begin
							queue_takeoff_plane = 1'b1;
							send_hold = 1'b1;
						end
					end
					else if (msg_action[1] == 1'b1) begin
						if (landing_fifo_full || emergency)
							send_divert = 1'b1;
						else begin
							queue_landing_plane = 1'b1;
							send_hold = 1'b1;
						end
					end
				end
				else if (msg_type == 3'b001) begin
					next_state = 3'b011;
					if (!msg_action[1]) begin
						if (!msg_action[0]) begin
							unlock = 1'b1;
							runway_id = 1'b0;
						end
						else if (msg_action[0]) begin
							unlock = 1'b1;
							runway_id = 1'b1;
						end
					end
					else if (msg_action[1]) begin
						if (!msg_action[0]) begin
							unlock = 1'b1;
							runway_id = 1'b0;
						end
						else if (msg_action[0]) begin
							unlock = 1'b1;
							runway_id = 1'b1;
						end
					end
				end
				else if (msg_type == 3'b010) begin
					if (msg_action == 2'b01) begin
						if (!landing_fifo_empty) begin
							next_state = 3'b110;
							unqueue_landing_plane = 1'b1;
						end
						else
							next_state = 3'b000;
						set_emergency = 1'b1;
					end
					else if (msg_action == 2'b00) begin
						next_state = 3'b011;
						unset_emergency = 1'b1;
					end
				end
				else begin
					next_state = 3'b010;
					send_say_ag = 1'b1;
				end
			3'b010:
				if (reply_fifo_full)
					next_state = 3'b010;
				else begin
					next_state = 3'b011;
					queue_reply = 1'b1;
				end
			3'b011:
				if (emergency) begin
					if (!landing_fifo_empty) begin
						next_state = 3'b110;
						unqueue_landing_plane = 1'b1;
					end
					else
						next_state = 3'b000;
				end
				else if (runway_active != 2'b11) begin
					if (!takeoff_fifo_empty && !landing_fifo_empty) begin
						if (takeoff_first) begin
							next_state = 3'b100;
							unqueue_takeoff_plane = 1'b1;
						end
						else begin
							next_state = 3'b101;
							unqueue_landing_plane = 1'b1;
						end
					end
					else if (!takeoff_fifo_empty) begin
						next_state = 3'b100;
						unqueue_takeoff_plane = 1'b1;
					end
					else if (!landing_fifo_empty) begin
						next_state = 3'b101;
						unqueue_landing_plane = 1'b1;
					end
					else
						next_state = 3'b000;
				end
				else
					next_state = 3'b000;
			3'b100: begin
				next_state = 3'b111;
				if (!runway_active[0]) begin
					runway_id = 1'b0;
					lock = 1'b1;
					send_clear = 2'b01;
				end
				else if (!runway_active[1]) begin
					runway_id = 1'b1;
					lock = 1'b1;
					send_clear = 2'b01;
				end
			end
			3'b101: begin
				next_state = 3'b111;
				if (!runway_active[0]) begin
					runway_id = 1'b0;
					lock = 1'b1;
					send_clear = 2'b10;
				end
				else if (!runway_active[1]) begin
					runway_id = 1'b1;
					lock = 1'b1;
					send_clear = 2'b10;
				end
			end
			3'b110: begin
				next_state = 3'b111;
				send_divert_landing = 1'b1;
			end
			3'b111: begin
				next_state = 3'b000;
				queue_reply = 1'b1;
			end
			default: next_state = 3'b000;
		endcase
	end
	always @(posedge clock)
		if (reset) begin
			state <= 3'b000;
			takeoff_first <= 1'b0;
		end
		else begin
			state <= next_state;
			takeoff_first <= ~takeoff_first;
		end
endmodule
module SendReplyFsm (
	clock,
	reset,
	uart_tx_ready,
	reply_fifo_empty,
	send_reply,
	uart_tx_send
);
	input wire clock;
	input wire reset;
	input wire uart_tx_ready;
	input wire reply_fifo_empty;
	output reg send_reply;
	output reg uart_tx_send;
	reg state;
	reg next_state;
	always @(*) begin
		send_reply = 1'b0;
		uart_tx_send = 1'b0;
		case (state)
			1'd0:
				if (reply_fifo_empty || !uart_tx_ready)
					next_state = 1'd0;
				else begin
					next_state = 1'd1;
					send_reply = 1'b1;
				end
			1'd1: begin
				next_state = 1'd0;
				uart_tx_send = 1'b1;
			end
			default: next_state = 1'd0;
		endcase
	end
	always @(posedge clock)
		if (reset)
			state <= 1'd0;
		else
			state <= next_state;
endmodule
module FIFO (
	clock,
	reset,
	data_in,
	we,
	re,
	data_out,
	full,
	empty
);
	parameter signed [31:0] WIDTH = 9;
	parameter signed [31:0] DEPTH = 4;
	input wire clock;
	input wire reset;
	input wire [WIDTH - 1:0] data_in;
	input wire we;
	input wire re;
	output reg [WIDTH - 1:0] data_out;
	output wire full;
	output wire empty;
	reg [(DEPTH * WIDTH) - 1:0] queue;
	reg [$clog2(DEPTH):0] count;
	reg [$clog2(DEPTH) - 1:0] put_ptr;
	reg [$clog2(DEPTH) - 1:0] get_ptr;
	assign empty = count == 0;
	assign full = count == DEPTH;
	always @(posedge clock)
		if (reset) begin
			count <= 0;
			get_ptr <= 0;
			put_ptr <= 0;
			data_out <= 0;
		end
		else if ((re && !empty) && we) begin
			data_out <= queue[get_ptr * WIDTH+:WIDTH];
			get_ptr <= get_ptr + 1;
			queue[put_ptr * WIDTH+:WIDTH] <= data_in;
			put_ptr <= put_ptr + 1;
		end
		else if (re && !empty) begin
			data_out <= queue[get_ptr * WIDTH+:WIDTH];
			get_ptr <= get_ptr + 1;
			count <= count - 1;
		end
		else if (we && !full) begin
			queue[put_ptr * WIDTH+:WIDTH] <= data_in;
			put_ptr <= put_ptr + 1;
			count <= count + 1;
		end
endmodule
module RunwayManager (
	clock,
	reset,
	plane_id,
	runway_id,
	lock,
	unlock,
	runway_active
);
	input wire clock;
	input wire reset;
	input wire [3:0] plane_id;
	input wire runway_id;
	input wire lock;
	input wire unlock;
	output wire [1:0] runway_active;
	reg [9:0] runway;
	assign runway_active[0] = runway[0];
	assign runway_active[1] = runway[5];
	always @(posedge clock)
		if (reset) begin
			runway[0] <= 0;
			runway[5] <= 0;
			runway <= 0;
		end
		else if (lock && !unlock) begin
			if (runway_id) begin
				runway[9-:4] <= plane_id;
				runway[5] <= 1'b1;
			end
			else begin
				runway[4-:4] <= plane_id;
				runway[0] <= 1'b1;
			end
		end
		else if (!lock && unlock) begin
			if (runway_id) begin
				if (plane_id == runway[9-:4])
					runway[5] <= 1'b0;
			end
			else if (plane_id == runway[4-:4])
				runway[0] <= 1'b0;
		end
endmodule
module AircraftIDManager (
	clock,
	reset,
	id_in,
	release_id,
	take_id,
	id_out,
	all_id,
	full
);
	input wire clock;
	input wire reset;
	input wire [3:0] id_in;
	input wire release_id;
	input wire take_id;
	output wire [3:0] id_out;
	output wire [15:0] all_id;
	output wire full;
	reg [15:0] taken_id;
	reg [3:0] id_avail;
	assign id_out = id_avail;
	assign full = taken_id == 16'hffff;
	always @(*) begin
		id_avail = 4'd0;
		case (1'b0)
			taken_id[0]: id_avail = 4'd0;
			taken_id[1]: id_avail = 4'd1;
			taken_id[2]: id_avail = 4'd2;
			taken_id[3]: id_avail = 4'd3;
			taken_id[4]: id_avail = 4'd4;
			taken_id[5]: id_avail = 4'd5;
			taken_id[6]: id_avail = 4'd6;
			taken_id[7]: id_avail = 4'd7;
			taken_id[8]: id_avail = 4'd8;
			taken_id[9]: id_avail = 4'd9;
			taken_id[10]: id_avail = 4'd10;
			taken_id[11]: id_avail = 4'd11;
			taken_id[12]: id_avail = 4'd12;
			taken_id[13]: id_avail = 4'd13;
			taken_id[14]: id_avail = 4'd14;
			taken_id[15]: id_avail = 4'd15;
			default: id_avail = 4'd0;
		endcase
	end
	always @(posedge clock)
		if (reset)
			taken_id <= 1'sb0;
		else if (release_id)
			taken_id[id_in] <= 1'b0;
		else if (take_id && !full)
			taken_id[id_avail] <= 1'b1;
endmodule