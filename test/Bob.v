`default_nettype none
module Bob (
	clock,
	reset_n,
	uart_rx_data,
	uart_rx_valid,
	uart_tx_data,
	uart_tx_en
);
	input wire clock;
	input wire reset_n;
	input wire [8:0] uart_rx_data;
	input wire uart_rx_valid;
	output wire [8:0] uart_tx_data;
	output wire uart_tx_en;
	wire [8:0] uart_request;
	wire uart_rd_request;
	wire uart_empty;
	FIFO #(
		.WIDTH(9),
		.DEPTH(4)
	) uart_requests(
		.clock(clock),
		.reset_n(reset_n),
		.data_in(uart_rx_data),
		.we(uart_rx_valid),
		.re(uart_rd_request),
		.data_out({uart_request[8-:4], uart_request[4-:3], uart_request[1-:2]}),
		.full(),
		.empty(uart_empty)
	);
	wire queue_takeoff_plane;
	wire clear_takeoff_plane;
	wire [3:0] cleared_takeoff_id;
	wire takeoff_fifo_full;
	FIFO #(
		.WIDTH(4),
		.DEPTH(4)
	) takeoff_fifo(
		.clock(clock),
		.reset_n(reset_n),
		.data_in(uart_request[8-:4]),
		.we(queue_takeoff_plane),
		.re(clear_takeoff_plane),
		.data_out(cleared_takeoff_id),
		.full(takeoff_fifo_full),
		.empty()
	);
	wire queue_landing_plane;
	wire clear_landing_plane;
	wire [3:0] cleared_landing_id;
	wire landing_fifo_full;
	FIFO #(
		.WIDTH(4),
		.DEPTH(4)
	) landing_fifo(
		.clock(clock),
		.reset_n(reset_n),
		.data_in(uart_request[8-:4]),
		.we(queue_landing_plane),
		.re(clear_landing_plane),
		.data_out(cleared_landing_id),
		.full(landing_fifo_full),
		.empty()
	);
	wire send_clear;
	wire send_hold;
	wire send_say_ag;
	wire send_divert;
	wire send_reply;
	wire lock;
	wire queue_reply;
	wire unlock;
	ReadRequestFSM fsm(
		.clock(clock),
		.reset_n(reset_n),
		.uart_empty(uart_empty),
		.uart_request(uart_request),
		.takeoff_fifo_full(takeoff_fifo_full),
		.landing_fifo_full(landing_fifo_full),
		.uart_rd_request(uart_rd_request),
		.queue_takeoff_plane(queue_takeoff_plane),
		.queue_landing_plane(queue_landing_plane),
		.send_clear(send_clear),
		.send_hold(send_hold),
		.send_say_ag(send_say_ag),
		.send_divert(send_divert),
		.queue_reply(queue_reply),
		.lock(lock),
		.unlock(unlock)
	);
	reg [8:0] reply_to_send;
	always @(posedge clock or negedge reset_n)
		if (~reset_n)
			reply_to_send <= 0;
		else if (send_clear) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b100;
			reply_to_send[1-:2] <= 2'b00;
		end
		else if (send_hold) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b101;
		end
		else if (send_say_ag) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b110;
		end
		else if (send_divert) begin
			reply_to_send[8-:4] <= uart_request[8-:4];
			reply_to_send[4-:3] <= 3'b111;
		end
	FIFO #(
		.WIDTH(9),
		.DEPTH(4)
	) uart_replies(
		.clock(clock),
		.reset_n(reset_n),
		.data_in(reply_to_send),
		.we(queue_reply),
		.re(send_reply),
		.data_out(uart_tx_data),
		.full(),
		.empty()
	);
	wire runway_id;
	wire [1:0] runway_active;
	RunwayManager manager(
		.clock(clock),
		.reset_n(reset_n),
		.plane_id(uart_request[8-:4]),
		.runway_id(runway_id),
		.lock(lock),
		.unlock(unlock),
		.runway_active(runway_active)
	);
endmodule
module ReadRequestFSM (
	clock,
	reset_n,
	uart_empty,
	uart_request,
	takeoff_fifo_full,
	landing_fifo_full,
	uart_rd_request,
	queue_takeoff_plane,
	queue_landing_plane,
	send_clear,
	send_hold,
	send_say_ag,
	send_divert,
	queue_reply,
	lock,
	unlock
);
	input wire clock;
	input wire reset_n;
	input wire uart_empty;
	input wire [8:0] uart_request;
	input wire takeoff_fifo_full;
	input wire landing_fifo_full;
	output reg uart_rd_request;
	output reg queue_takeoff_plane;
	output reg queue_landing_plane;
	output reg send_clear;
	output reg send_hold;
	output reg send_say_ag;
	output reg send_divert;
	output reg queue_reply;
	output wire lock;
	output wire unlock;
	wire [2:0] msg_type;
	wire [1:0] msg_action;
	reg takeoff_first;
	assign msg_type = uart_request[4-:3];
	assign msg_action = uart_request[1-:2];
	reg [3:0] state;
	reg [3:0] next_state;
	always @(*) begin
		uart_rd_request = 1'b0;
		queue_takeoff_plane = 1'b0;
		queue_landing_plane = 1'b0;
		send_clear = 1'b0;
		send_hold = 1'b0;
		send_say_ag = 1'b0;
		send_divert = 1'b0;
		queue_reply = 1'b0;
		case (state)
			4'd0:
				if (uart_empty)
					next_state = 4'd0;
				else begin
					next_state = 4'd1;
					uart_rd_request = 1'b1;
				end
			4'd1:
				if (msg_type == 3'b000) begin
					if (msg_action == 2'b0x) begin
						if (takeoff_fifo_full) begin
							next_state = 4'd3;
							send_divert = 1'b1;
						end
						else begin
							next_state = 4'd2;
							queue_takeoff_plane = 1'b1;
							send_hold = 1'b1;
						end
					end
					else if (msg_action == 2'b1x) begin
						if (landing_fifo_full) begin
							next_state = 4'd3;
							send_divert = 1'b1;
						end
						else begin
							next_state = 4'd2;
							queue_landing_plane = 1'b1;
							send_hold = 1'b1;
						end
					end
				end
				else if (msg_type == 3'b001)
					;
				else if (msg_type == 3'b010)
					;
				else if (msg_type == 3'b011)
					;
				else begin
					next_state = 4'd4;
					send_say_ag = 1'b1;
				end
			4'd2: begin
				next_state = (takeoff_first ? 4'd6 : 4'd7);
				queue_reply = 1'b1;
			end
			4'd3: begin
				next_state = (takeoff_first ? 4'd6 : 4'd7);
				queue_reply = 1'b1;
			end
			4'd4: begin
				next_state = (takeoff_first ? 4'd6 : 4'd7);
				queue_reply = 1'b1;
			end
			4'd5: begin
				next_state = (takeoff_first ? 4'd6 : 4'd7);
				queue_reply = 1'b1;
			end
			4'd6: next_state = 4'd0;
			4'd7: next_state = 4'd0;
		endcase
	end
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin
			state <= 4'd0;
			takeoff_first <= 1'b0;
		end
		else begin
			state <= next_state;
			takeoff_first <= takeoff_first + 1'b1;
		end
endmodule
module FIFO (
	clock,
	reset_n,
	data_in,
	we,
	re,
	data_out,
	full,
	empty
);
	parameter WIDTH = 9;
	parameter DEPTH = 4;
	input wire clock;
	input wire reset_n;
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
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin
			count <= 0;
			get_ptr <= 0;
			put_ptr <= 0;
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
	reset_n,
	plane_id,
	runway_id,
	lock,
	unlock,
	runway_active
);
	input wire clock;
	input wire reset_n;
	input wire [3:0] plane_id;
	input wire runway_id;
	input wire lock;
	input wire unlock;
	output wire [1:0] runway_active;
	reg [9:0] runway;
	assign runway_active[0] = runway[0];
	assign runway_active[1] = runway[5];
	always @(posedge clock or negedge reset_n)
		if (~reset_n) begin
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
				runway[5] <= 1'b1;
		end
endmodule