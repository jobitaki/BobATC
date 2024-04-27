`default_nettype none
module UartRX (
	clock,
	reset,
	rx,
	data,
	done,
	framing_error
);
	input wire clock;
	input wire reset;
	input wire rx;
	output reg [8:0] data;
	output wire done;
	output wire framing_error;
	wire start;
	wire tick;
	BaudRateGenerator #(
		.CLK_HZ(25000000),
		.BAUD_RATE(9600),
		.SAMPLE_RATE(16)
	) conductor(
		.clock(clock),
		.reset(reset),
		.start_rx(start),
		.start_tx(1'b0),
		.tick(tick)
	);
	wire collect_data;
	wire en_data_counter;
	wire clear_data_counter;
	reg [3:0] data_counter;
	wire done_data;
	always @(posedge clock)
		if (reset)
			data <= 1'sb0;
		else if (collect_data && tick) begin
			data <= data >> 1;
			data[8] <= rx;
		end
	assign done_data = data_counter == 4'd9;
	always @(posedge clock)
		if (reset || clear_data_counter)
			data_counter <= 1'sb0;
		else if (en_data_counter && tick)
			data_counter <= data_counter + 1'b1;
	UartRXFsm fsm(
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
endmodule
module UartRXFsm (
	clock,
	reset,
	tick,
	rx,
	done_data,
	start,
	collect_data,
	en_data_counter,
	clear_data_counter,
	framing_error,
	done
);
	input wire clock;
	input wire reset;
	input wire tick;
	input wire rx;
	input wire done_data;
	output reg start;
	output reg collect_data;
	output reg en_data_counter;
	output reg clear_data_counter;
	output reg framing_error;
	output reg done;
	reg [1:0] state;
	reg [1:0] next_state;
	always @(*) begin
		start = 1'b0;
		collect_data = 1'b0;
		en_data_counter = 1'b0;
		clear_data_counter = 1'b0;
		framing_error = 1'b0;
		done = 1'b0;
		case (state)
			2'd0:
				if (!rx) begin
					next_state = 2'd1;
					start = 1'b1;
				end
				else
					next_state = 2'd0;
			2'd1:
				if (tick && rx)
					next_state = 2'd0;
				else if (tick && !rx)
					next_state = 2'd2;
				else
					next_state = 2'd1;
			2'd2:
				if (tick && !done_data) begin
					next_state = 2'd2;
					collect_data = 1'b1;
					en_data_counter = 1'b1;
				end
				else if (tick && done_data) begin
					if (!rx) begin
						next_state = 2'd3;
						framing_error = 1'b1;
					end
					else begin
						next_state = 2'd0;
						clear_data_counter = 1'b1;
						done = 1'b1;
					end
				end
			2'd3: begin
				if (tick && rx) begin
					next_state = 2'd0;
					clear_data_counter = 1'b1;
				end
				else if (tick && !rx)
					next_state = 2'd3;
				framing_error = 1'b1;
			end
		endcase
	end
	always @(posedge clock)
		if (reset)
			state <= 2'd0;
		else
			state <= next_state;
endmodule
module UartTB (
	clock,
	reset,
	send,
	data_tx,
	tx,
	ready,
	rx,
	data_rx,
	done,
	framing_error
);
	input wire clock;
	input wire reset;
	input wire send;
	input wire [8:0] data_tx;
	output wire tx;
	output wire ready;
	input wire rx;
	output wire [8:0] data_rx;
	output wire done;
	output wire framing_error;
	UartTX dut_tx(
		.data(data_tx),
		.clock(clock),
		.reset(reset),
		.send(send),
		.tx(tx),
		.ready(ready)
	);
	UartRX dut_rx(
		.data(data_rx),
		.clock(clock),
		.reset(reset),
		.rx(rx),
		.done(done),
		.framing_error(framing_error)
	);
endmodule
`default_nettype none
module UartTX (
	clock,
	reset,
	send,
	data,
	tx,
	ready
);
	input wire clock;
	input wire reset;
	input wire send;
	input wire [8:0] data;
	output reg tx;
	output wire ready;
	wire start;
	wire tick;
	BaudRateGenerator #(
		.CLK_HZ(25000000),
		.BAUD_RATE(9600),
		.SAMPLE_RATE(16)
	) conductor(
		.clock(clock),
		.reset(reset),
		.start_rx(1'b0),
		.start_tx(start),
		.tick(tick)
	);
	wire en_data_counter;
	reg [3:0] data_counter;
	wire done_data;
	wire clear_data_counter;
	assign done_data = data_counter == 4'd9;
	always @(posedge clock)
		if (reset || clear_data_counter)
			data_counter <= 1'sb0;
		else if (en_data_counter && tick)
			data_counter <= data_counter + 1;
	reg [8:0] saved_data;
	reg data_bit;
	wire send_data;
	always @(posedge clock)
		if (reset)
			saved_data <= 1'sb0;
		else if (start)
			saved_data <= data;
		else if (send_data && tick)
			saved_data <= saved_data >> 1;
	always @(posedge clock)
		if (reset || start)
			data_bit <= 1'b0;
		else if (send_data && tick)
			data_bit <= saved_data[0];
	wire send_start_bit;
	wire send_stop_bit;
	always @(posedge clock)
		if (reset)
			tx <= 1'b1;
		else if (send_start_bit)
			tx <= 1'b0;
		else if (send_data)
			tx <= data_bit;
		else if (send_stop_bit)
			tx <= 1'b1;
		else
			tx <= 1'b1;
	UartTXFsm fsm(
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
		.ready(ready)
	);
endmodule
module UartTXFsm (
	clock,
	reset,
	send,
	tick,
	done_data,
	start,
	send_start_bit,
	send_data,
	send_stop_bit,
	en_data_counter,
	clear_data_counter,
	ready
);
	input wire clock;
	input wire reset;
	input wire send;
	input wire tick;
	input wire done_data;
	output reg start;
	output reg send_start_bit;
	output reg send_data;
	output reg send_stop_bit;
	output reg en_data_counter;
	output reg clear_data_counter;
	output reg ready;
	reg [1:0] state;
	reg [1:0] next_state;
	always @(*) begin
		start = 1'b0;
		send_start_bit = 1'b0;
		send_data = 1'b0;
		send_stop_bit = 1'b0;
		en_data_counter = 1'b0;
		clear_data_counter = 1'b0;
		ready = 1'b0;
		case (state)
			2'd0:
				if (send) begin
					next_state = 2'd1;
					start = 1'b1;
					send_start_bit = 1'b1;
				end
				else begin
					next_state = 2'd0;
					ready = 1'b1;
				end
			2'd1:
				if (tick) begin
					next_state = 2'd2;
					send_data = 1'b1;
					en_data_counter = 1'b1;
				end
				else begin
					next_state = 2'd1;
					send_start_bit = 1'b1;
				end
			2'd2:
				if (tick && done_data) begin
					next_state = 2'd3;
					send_stop_bit = 1'b1;
					clear_data_counter = 1'b1;
				end
				else begin
					next_state = 2'd2;
					send_data = 1'b1;
					en_data_counter = 1'b1;
				end
			2'd3:
				if (tick) begin
					next_state = 2'd0;
					ready = 1'b1;
				end
				else begin
					next_state = 2'd3;
					send_stop_bit = 1'b1;
				end
		endcase
	end
	always @(posedge clock)
		if (reset)
			state <= 2'd0;
		else
			state <= next_state;
endmodule
`default_nettype none
module BaudRateGenerator (
	clock,
	reset,
	start_rx,
	start_tx,
	tick
);
	parameter signed [31:0] CLK_HZ = 25000000;
	parameter signed [31:0] BAUD_RATE = 9600;
	parameter signed [31:0] SAMPLE_RATE = 16;
	input wire clock;
	input wire reset;
	input wire start_rx;
	input wire start_tx;
	output wire tick;
	parameter signed [31:0] DIVISOR = CLK_HZ / (BAUD_RATE * SAMPLE_RATE);
	reg [$clog2(DIVISOR) + 1:0] clockCount;
	assign tick = clockCount == DIVISOR;
	always @(posedge clock)
		if (reset | tick)
			clockCount <= 1'sb0;
		else if (start_rx)
			clockCount <= DIVISOR / 2;
		else if (start_tx)
			clockCount <= 1'sb0;
		else
			clockCount <= clockCount + 1'b1;
endmodule
module baud_rate_generator_tb;
	reg clock;
	reg reset;
	reg start_rx;
	wire start_tx;
	wire tick;
	baud_rate_generator dut(.*);
	initial begin
		clock = 0;
		forever #(1) clock = ~clock;
	end
	initial begin
		start_rx = 0;
		reset <= 1'b1;
		@(posedge clock)
			;
		reset <= 1'b0;
		begin : sv2v_autoblock_1
			reg signed [31:0] i;
			for (i = 0; i < 1000; i = i + 1)
				@(posedge clock)
					;
		end
		start_rx <= 1'b1;
		@(posedge clock)
			;
		start_rx <= 1'b0;
		begin : sv2v_autoblock_2
			reg signed [31:0] i;
			for (i = 0; i < 1000; i = i + 1)
				@(posedge clock)
					;
		end
		$finish;
	end
endmodule