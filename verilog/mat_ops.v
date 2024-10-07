

module mat_ops#(

	parameter READ    = 3'b000,
	parameter CNN_RUN = 3'b001,
	parameter WRITE   = 3'b010,
	parameter DONE    = 3'b011,
	parameter IDLE	  = 3'b111,

	//parameter READ_READ0 = 4'd0,
	//parameter READ_READ1 = 4'd1,
	//parameter READ_READ2 = 4'd2,
	//parameter READ_READ3 = 4'd3,
	//parameter READ_READ4 = 4'd4,
	//parameter READ_READ5 = 4'd5,
	//parameter READ_READ6 = 4'd6,
	//parameter READ_READ7 = 4'd7,
	//parameter READ_WAIT  = 4'd8,
	parameter READ_DONE  = 4'd14,
	parameter READ_IDLE  = 4'd15,

	// parameter WRITE_WRITE0 = 4'd0,
	// parameter WRITE_WRITE1 = 4'd1,
	// parameter WRITE_WRITE2 = 4'd2,
	// parameter WRITE_WRITE3 = 4'd3,
	// parameter WRITE_WRITE4 = 4'd4,
	// parameter WRITE_WRITE5 = 4'd5,
	// parameter WRITE_WRITE6 = 4'd6,
	// parameter WRITE_WRITE7 = 4'd7,
    parameter WRITE_DONE   = 4'd8,
	parameter WRITE_IDLE   = 4'd15
)(
	input 					 		i_clk,
	input 					 		i_rstn,
	input							i_start,
	///////////// SRAM a //////////
	input   [ROW_SIZE-1:0]	  	  	i_read_data_A,
	output	[ADDRESS_SIZE-1:0]	  	o_address_A,
	output  						o_wr_en_A,
	
	///////////// SRAM b //////////
	input   [ROW_SIZE-1:0]	  	  	i_read_data_B,
	output	[ADDRESS_SIZE-1:0]	  	o_address_B,
	output  						o_wr_en_B,

	output  [ROW_SIZE-1:0]	  	  	o_write_data_B,
	
	///////////// SRAM C //////////
	input   [ROW_SIZE-1:0]	  	  	i_read_data_C,
	output	[ADDRESS_SIZE-1:0]	  	o_address_C,
	output  						o_wr_en_C,

	output  [ROW_SIZE-1:0]	  	  	o_write_data_C,
	
	output  [2:0]					o_state,
	output	[3:0]					o_read_state,
	output							o_cnn_done,
	output	[3:0]					o_write_state,
	output							o_done
);
`include "defines_computer.vh"

	reg		[2:0] state;     
	reg  	[2:0] next_state; 
	
	wire  	[3:0] read_state_a;
	wire  	[3:0] read_state_b;
	wire  	[3:0] read_state_c;
	
	wire  		  w_ot_valid;

	wire		  w_valid;
	
	wire  	[3:0] write_state_b;
	wire  	[3:0] write_state_c;
	
	assign o_read_state 	= (read_state_a && read_state_b && read_state_c);
	assign o_cnn_done  		= w_ot_valid;
	assign o_write_state	= (write_state_b && write_state_c);
	
	wire read_start  ;
	wire read_done  ;
	wire cnn_start;
	wire cnn_done;
	wire write_start ;
	wire write_done ;
	wire read_reset ;
	
	//write data
    wire [ADDRESS_SIZE-1:0] 	write_address_B;
	wire [ADDRESS_SIZE-1:0] 	write_address_C;
	
	//read data
    wire [ADDRESS_SIZE-1:0] 	read_address_A;
	wire [ADDRESS_SIZE-1:0] 	read_address_B;
	wire [ADDRESS_SIZE-1:0] 	read_address_C;
	
	wire w_en_B;
	wire w_en_C;
	
	//concat infmap
	wire [INPUT_SIZE-1:0]			infmap;			//input1 + input2
	wire [(DATA_LEN*IY*IX*ICH)-1:0]	read_B;			//fmap input1
	wire [(DATA_LEN*IY*IX*ICH)-1:0]	read_C;			//fmap input2
	
	assign infmap = {read_B, read_C};
	
	//concat output
	wire [OUTPUT_SIZE-1:0]			cnn_out;		//output from cnn_topCore
	wire [(DATA_LEN*OCH*OY*OX)-1:0]	cnn_out_B;		//output1
	wire [(DATA_LEN*OCH*OY*OX)-1:0]	cnn_out_C;		//output2
	
	assign {cnn_out_B, cnn_out_C} = cnn_out;
	
	wire [DATA_LEN*M*4-1:0]		w_write_cnn_B;		//8x4 matrix (read from memory)
	wire [DATA_LEN*M*4-1:0]		w_write_cnn_C;		//8x4 matrix (read from memory)
	
	//weight
	wire [ROW_SIZE*12-1:0]		read_weight;		//read from memory
	wire [WEIGHT_SIZE-1:0]		weight_A;			//weight
	
	//reconstruct weight data
	//your code here!
	parameter HALF_SIZE = ROW_SIZE / 2;
	// wire last_index_recon;
	// assign last_index_recon = 0;
	genvar recon_weight;
	generate
		for(recon_weight = 0; recon_weight < 6; recon_weight = recon_weight + 1) begin: recongenerate
			assign weight_A[32*12*recon_weight+:32*12] = read_weight[32*16*recon_weight+:32*12];
			// if(recon_weight % 2 == 0) begin
			// 	assign weight_A[(recon_weight/2)*ROW_SIZE + (recon_weight/2)*HALF_SIZE+:ROW_SIZE] = read_weight[ROW_SIZE*recon_weight+:ROW_SIZE];
			// 	// assign last_index_recon = last_index_recon + ROW_SIZE;
			// end
			// else begin
			// 	assign weight_A[((recon_weight+1)/2)*ROW_SIZE + (recon_weight/2)*HALF_SIZE+:HALF_SIZE] = read_weight[ROW_SIZE*recon_weight+:HALF_SIZE];
			// 	// assign last_index_recon = last_index_recon + HALF_SIZE;
			// end
		end
	endgenerate
	// parameter IDLE_WEIGHT = 4'b0000;
	// parameter WEIGHT0	 = 4'b0001;
	// parameter WEIGHT1	 = 4'b0010;
	// parameter WEIGHT2	 = 4'b0011;
	// parameter WEIGHT3	 = 4'b0100;
	// parameter WEIGHT4	 = 4'b0101;
	// parameter WEIGHT5	 = 4'b0110;
	// parameter WEIGHT6	 = 4'b0111;
	// parameter WEIGHT7	 = 4'b1000;
	// parameter WEIGHT8	 = 4'b1001;
	// parameter WEIGHT9	 = 4'b1010;
	// parameter WEIGHT10	 = 4'b1011;
	// parameter WEIGHT11	 = 4'b1100;
	// parameter DONE_WEIGHT = 4'b1101;

	// reg [3:0] weight_state;
	// reg [3:0] weight_next_state;
	// reg [WEIGHT_SIZE-1:0]		r_weight_A;			//weight
	// reg weight_finish; // flag

	// always @(posedge i_clk, negedge i_rstn) begin
	// 	if(!i_rstn) begin
	// 		weight_state <= IDLE_WEIGHT;
	// 	end
	// 	else begin
	// 		weight_state <= weight_next_state;
	// 	end	
	// end

	// always@(*) begin
	// 	case(weight_state)
	// 		IDLE_WEIGHT: begin
	// 			if(read_done) begin
	// 				weight_next_state = WEIGHT0;
	// 				weight_finish = 1'b0;
	// 			end
	// 			else begin
	// 				weight_next_state = IDLE_WEIGHT;
	// 				weight_finish = 1'b0;
	// 			end
	// 		end
	// 		WEIGHT0: begin
	// 			weight_next_state = WEIGHT1;
	// 			r_weight_A[0+:ROW_SIZE] = read_weight[0+:ROW_SIZE];
	// 		end
	// 		WEIGHT1: begin
	// 			weight_next_state = WEIGHT2;
	// 			r_weight_A[1*ROW_SIZE+:HALF_SIZE] = read_weight[1*ROW_SIZE+:HALF_SIZE];
	// 		end
	// 		WEIGHT2: begin
	// 			weight_next_state = WEIGHT3;
	// 			r_weight_A[1*ROW_SIZE+1*HALF_SIZE+:ROW_SIZE] = read_weight[2*ROW_SIZE+:ROW_SIZE];
	// 		end
	// 		WEIGHT3: begin
	// 			weight_next_state = WEIGHT4;
	// 			r_weight_A[2*ROW_SIZE+1*HALF_SIZE+:HALF_SIZE] = read_weight[3*ROW_SIZE+:HALF_SIZE];
	// 		end
	// 		WEIGHT4: begin
	// 			weight_next_state = WEIGHT5;
	// 			r_weight_A[2*ROW_SIZE+2*HALF_SIZE+:ROW_SIZE] = read_weight[4*ROW_SIZE+:ROW_SIZE];
	// 		end
	// 		WEIGHT5: begin
	// 			weight_next_state = WEIGHT6;
	// 			r_weight_A[3*ROW_SIZE+2*HALF_SIZE+:HALF_SIZE] = read_weight[5*ROW_SIZE+:HALF_SIZE];
	// 		end
	// 		WEIGHT6: begin
	// 			weight_next_state = WEIGHT7;
	// 			r_weight_A[3*ROW_SIZE+3*HALF_SIZE+:ROW_SIZE] = read_weight[6*ROW_SIZE+:ROW_SIZE];
	// 		end
	// 		WEIGHT7: begin
	// 			weight_next_state = WEIGHT8;
	// 			r_weight_A[4*ROW_SIZE+3*HALF_SIZE+:HALF_SIZE] = read_weight[7*ROW_SIZE+:HALF_SIZE];
	// 		end
	// 		WEIGHT8: begin
	// 			weight_next_state = WEIGHT9;
	// 			r_weight_A[4*ROW_SIZE+4*HALF_SIZE+:ROW_SIZE] = read_weight[8*ROW_SIZE+:ROW_SIZE];
	// 		end
	// 		WEIGHT9: begin
	// 			weight_next_state = WEIGHT10;
	// 			r_weight_A[5*ROW_SIZE+4*HALF_SIZE+:HALF_SIZE] = read_weight[9*ROW_SIZE+:HALF_SIZE];
	// 		end
	// 		WEIGHT10: begin
	// 			weight_next_state = WEIGHT11;
	// 			r_weight_A[5*ROW_SIZE+5*HALF_SIZE+:ROW_SIZE] = read_weight[10*ROW_SIZE+:ROW_SIZE];
	// 		end
	// 		WEIGHT11: begin
	// 			weight_next_state = DONE_WEIGHT;
	// 			r_weight_A[6*ROW_SIZE+5*HALF_SIZE+:HALF_SIZE] = read_weight[11*ROW_SIZE+:HALF_SIZE];
	// 		end
	// 		DONE_WEIGHT: begin
	// 			//weight_next_state = IDLE_WEIGHT;
	// 			weight_finish = 1'b1;
	// 			r_weight_A <= r_weight_A;
	// 		end
	// 	endcase
	// end

	// assign weight_A = r_weight_A;
	
	//reconstruct output data
	//your code here!

	assign w_write_cnn_B = {192'b0, cnn_out[32*10+:32*10], 192'b0, cnn_out[0+:32*10]};
	assign w_write_cnn_C = {192'b0, cnn_out[32*10*3+:32*10], 192'b0, cnn_out[32*10*2+:32*10]};

	//parameter HALF_HALF_SIZE = ROW_SIZE / 4;
	// parameter LEFT_SIZE = ROW_SIZE - HALF_HALF_SIZE;
	// genvar recon_cnn_out, recon_cnn_chip;
	// generate
	// 	for(recon_cnn_out = 0; recon_cnn_out < 2; recon_cnn_out = recon_cnn_out + 1) begin: cnn_result
	// 		for(recon_cnn_chip = 0; recon_cnn_chip < 4; recon_cnn_chip = recon_cnn_chip + 1) begin: cnn_devide
	// 			if(recon_cnn_out % 2 == 0) begin // B
	// 				if(recon_cnn_chip % 2 == 0) begin
	// 					assign w_write_cnn_B[ROW_SIZE*recon_cnn_chip+:ROW_SIZE] = cnn_out[(recon_cnn_chip/2)*ROW_SIZE+(recon_cnn_chip/2)*HALF_HALF_SIZE+:ROW_SIZE];
	// 					// assign last_index_cnn_out = last_index_cnn_out + ROW_SIZE;
	// 				end
	// 				else begin
	// 					assign w_write_cnn_B[ROW_SIZE*recon_cnn_chip+:ROW_SIZE] = {192'b0, cnn_out[((recon_cnn_chip+1)/2)*ROW_SIZE+(recon_cnn_chip/2)*HALF_HALF_SIZE+:HALF_HALF_SIZE]};
	// 					// assign last_index_cnn_out = last_index_cnn_out + HALF_HALF_SIZE;
	// 				end
	// 			end
	// 			else begin // C
	// 				if(recon_cnn_chip % 2 == 0) begin
	// 					assign w_write_cnn_C[ROW_SIZE*recon_cnn_chip+:ROW_SIZE] = cnn_out[(recon_cnn_out*2+(recon_cnn_chip/2))*ROW_SIZE+(recon_cnn_out*2+(recon_cnn_chip/2))*HALF_HALF_SIZE+:ROW_SIZE];
	// 					// assign last_index_cnn_out = last_index_cnn_out + ROW_SIZE;
	// 				end
	// 				else begin
	// 					assign w_write_cnn_C[ROW_SIZE*recon_cnn_chip+:ROW_SIZE] = {192'b0, cnn_out[(recon_cnn_out*2+((recon_cnn_chip+1)/2))*ROW_SIZE+(recon_cnn_out*2+(recon_cnn_chip/2))*HALF_HALF_SIZE+:HALF_HALF_SIZE]};
	// 					// assign last_index_cnn_out = last_index_cnn_out + HALF_HALF_SIZE;
	// 				end
	// 			end
	// 		end
	// 	end

	// endgenerate


	// parameter IDLE_RESULT = 4'b0000;
	// parameter RESULT0	 = 4'b0001;
	// parameter RESULT1	 = 4'b0010;
	// parameter RESULT2	 = 4'b0011;
	// parameter RESULT3	 = 4'b0100;
	// parameter RESULT4	 = 4'b0101;
	// parameter RESULT5	 = 4'b0110;
	// parameter RESULT6	 = 4'b0111;
	// parameter RESULT7	 = 4'b1000;
	// parameter DONE_RESULT = 4'b1001;

	// reg [3:0] result_state;
	// reg [3:0] result_next_state;
	// reg [DATA_LEN*M*4-1:0]		r_write_cnn_B;
	// reg [DATA_LEN*M*4-1:0]		r_write_cnn_C;
	// reg result_finish; // flag

	// always @(posedge i_clk, negedge i_rstn) begin
	// 	if(!i_rstn) begin
	// 		result_state <= IDLE_RESULT;
	// 	end
	// 	else begin
	// 		result_state <= result_next_state;
	// 	end	
	// end

	// always @(*) begin
	// 	case(result_state)
	// 		IDLE_RESULT: begin
	// 			if(cnn_done) begin
	// 				result_next_state = RESULT0;
	// 				result_finish = 1'b0;
	// 			end
	// 			else begin
	// 				result_next_state = IDLE_RESULT;
	// 				result_finish = 1'b0;
	// 			end
	// 		end
	// 		RESULT0: begin
	// 			result_next_state = RESULT1;
	// 			r_write_cnn_B[0+:ROW_SIZE] = cnn_out[0+:ROW_SIZE];
	// 		end
	// 		RESULT1: begin
	// 			result_next_state = RESULT2;
	// 			r_write_cnn_B[1*ROW_SIZE+:HALF_HALF_SIZE] = cnn_out[1*ROW_SIZE+:HALF_HALF_SIZE];
	// 			//r_write_cnn_B[1*ROW_SIZE+1*HALF_HALF_SIZE +: LEFT_SIZE] = 192'b0;
	// 		end
	// 		RESULT2: begin
	// 			result_next_state = RESULT3;
	// 			r_write_cnn_B[2*ROW_SIZE+:ROW_SIZE] = cnn_out[1*ROW_SIZE+1*HALF_HALF_SIZE+:ROW_SIZE];
	// 		end
	// 		RESULT3: begin
	// 			result_next_state = RESULT4;
	// 			r_write_cnn_B[3*ROW_SIZE+:HALF_HALF_SIZE] = cnn_out[2*ROW_SIZE+1*HALF_HALF_SIZE+:HALF_HALF_SIZE];
	// 			//r_write_cnn_B[3*ROW_SIZE+HALF_HALF_SIZE+:LEFT_SIZE] = 192'b0;
	// 		end
	// 		RESULT4: begin
	// 			result_next_state = RESULT5;
	// 			r_write_cnn_C[0+:ROW_SIZE] = cnn_out[2*ROW_SIZE+2*HALF_HALF_SIZE+:ROW_SIZE];
	// 		end
	// 		RESULT5: begin
	// 			result_next_state = RESULT6;
	// 			r_write_cnn_C[1*ROW_SIZE+:HALF_HALF_SIZE] = cnn_out[3*ROW_SIZE+2*HALF_HALF_SIZE+:HALF_HALF_SIZE];
	// 			//r_write_cnn_C[1*ROW_SIZE+HALF_HALF_SIZE+:LEFT_SIZE] = 192'b0;
	// 		end
	// 		RESULT6: begin
	// 			result_next_state = RESULT7;
	// 			r_write_cnn_C[2*ROW_SIZE+:ROW_SIZE] = cnn_out[3*ROW_SIZE+3*HALF_HALF_SIZE+:ROW_SIZE];
	// 		end
	// 		RESULT7: begin
	// 			result_next_state = DONE_RESULT;
	// 			r_write_cnn_C[3*ROW_SIZE+:HALF_HALF_SIZE] = cnn_out[4*ROW_SIZE+3*HALF_HALF_SIZE+:HALF_HALF_SIZE];
	// 			//r_write_cnn_C[3*ROW_SIZE+HALF_HALF_SIZE+:LEFT_SIZE] = 192'b0;
	// 		end
	// 		DONE_RESULT: begin
	// 			//result_next_state = IDLE_RESULT;
	// 			result_finish = 1'b1;
	// 			r_write_cnn_B <= r_write_cnn_B;
	// 			r_write_cnn_C <= r_write_cnn_C;
	// 		end
	// 	endcase
	// end
	
	// assign w_write_cnn_B = r_write_cnn_B;
	// assign w_write_cnn_C = r_write_cnn_C;

	// read_done_late
	reg [3:0] read_done_late;
	always @(posedge i_clk, negedge i_rstn) begin
		if(!i_rstn) read_done_late <= 4'b0;
		else begin
			if(read_done && read_done_late == 4'b0) 	read_done_late <= 4'b0001;
			else if(read_done_late[0]) read_done_late <= 4'b0010;
			else if(read_done_late[1]) read_done_late <= 4'b0100;
			else if(read_done_late[2]) read_done_late <= 4'b1000;
			else			read_done_late <= 4'b0;
		end
	end

	// cnn_done_late
	// reg [3:0] cnn_done_late;
	// always @(posedge i_clk, negedge i_rstn) begin
	// 	if(!i_rstn) cnn_done_late <= 4'b0;
	// 	else begin
	// 		if(cnn_done) 	cnn_done_late <= 4'b0001;
	// 		else if(cnn_done_late[0]) cnn_done_late <= 4'b0010;
	// 		else if(cnn_done_late[1]) cnn_done_late <= 4'b0100;
	// 		else if(cnn_done_late[2]) cnn_done_late <= 4'b1000;
	// 		else			cnn_done_late <= 4'b0;
	// 	end
	// end
	
	
	//state logic
	always @(posedge i_clk, negedge i_rstn) begin
		if(!i_rstn) begin
			state <= IDLE;
		end
		else begin
			state <= next_state;
		end	
	end
	
	assign o_state = state;
	
	always @(*) begin
		case(state) 
			IDLE:       begin
							if(i_start)  		next_state = READ;
							else 	     		next_state = IDLE;
						end	
			READ:       begin	
							if(read_done_late[3])  		next_state = CNN_RUN; // weight_finish
							else 	     		next_state = READ;		
						end	
			CNN_RUN:    begin			    	     	
							if(cnn_done)		next_state = WRITE; // result_finish
							else 				next_state = CNN_RUN;
						end					
			WRITE:      begin	
							if(write_done)  	next_state = DONE;
							else 	     		next_state = WRITE;
						end	
			DONE:       begin	
												next_state = IDLE;
						end 
		endcase      
	end
	

	assign read_start 	= (state == IDLE   )   &&	(next_state     == READ);
	assign read_done  	= (state == READ   )   &&	(read_state_a   == READ_DONE)	&&	(read_state_b   == READ_DONE)	&&	(read_state_c   == READ_DONE);
	assign cnn_start 	= (state == READ   )   &&	(next_state     == CNN_RUN);
	assign cnn_done 	= (state == CNN_RUN)   &&	(w_ot_valid  == 1'b1);
	assign write_start 	= (state == CNN_RUN)   &&	(next_state     == WRITE);
	assign write_done  	= (state == WRITE  )   &&	(write_state_b 	== WRITE_DONE)	&&	(write_state_c	== WRITE_DONE);
	assign read_reset  	= (state == DONE   );
	assign o_done      	= (state == DONE   );
	assign o_address_A 	= read_address_A;
	assign o_address_B 	= (w_en_B == 1'b1) ? (write_address_B) : read_address_B;
	assign o_address_C 	= (w_en_C == 1'b1) ? (write_address_C) : read_address_C;
	assign o_wr_en_A   	= 1'b0;
	assign o_wr_en_B   	= (w_en_B == 1'b1) ? w_en_B : 1'b0;
	assign o_wr_en_C   	= (w_en_C == 1'b1) ? w_en_C : 1'b0;
	
																	//OFFSET mapping!
M10K_read_buffer #(.DATA_LEN(DATA_LEN), .ADDRESS_SIZE(ADDRESS_SIZE), .OFFSET(READ_A_ADDR_OFFSET)) s1
(
	.i_clk         (i_clk),			
	.i_rstn        (i_rstn),               
	.i_read_reset  (read_reset),           
	.i_read_start  (read_start),           
	.i_read_data   (i_read_data_A),        
				                           
	.o_store_mat   (read_weight),             		//port mapping!
	.o_read_addr   (read_address_A),            
	.o_state       (read_state_a)            
);

																	//OFFSET mapping!
M10K_read_buffer #(.DATA_LEN(DATA_LEN), .ADDRESS_SIZE(ADDRESS_SIZE), .OFFSET(READ_B_ADDR_OFFSET)) s2
(
	.i_clk		   (i_clk),
	.i_rstn  	   (i_rstn),
	.i_read_reset  (read_reset),
	.i_read_start  (read_start),
	.i_read_data   (i_read_data_B),
	
	.o_store_mat   (read_B),					//port mapping!
	.o_read_addr   (read_address_B),
	.o_state       (read_state_b)
);

																	//OFFSET mapping!
M10K_read_buffer #(.DATA_LEN(DATA_LEN), .ADDRESS_SIZE(ADDRESS_SIZE), .OFFSET(READ_C_ADDR_OFFSET)) s3
(
	.i_clk		   (i_clk),
	.i_rstn  	   (i_rstn),
	.i_read_reset  (read_reset),
	.i_read_start  (read_start),
	.i_read_data   (i_read_data_C),
	
	.o_store_mat   (read_C),					//port mapping!
	.o_read_addr   (read_address_C),
	.o_state       (read_state_c)
);


cnn_topCore c0(
	.clk          (i_clk)   ,
    .reset_n      (i_rstn)   ,
    // .i_soft_reset ()   ,
    .i_cnn_weight (weight_A)   ,
    .i_in_valid   (cnn_start)   ,
    .i_in_fmap    (infmap)   ,
    .o_ot_valid   (w_ot_valid)   ,
    .o_ot_fmap    (cnn_out)         

);


																//OFFSET mapping!
M10K_write #(.DATA_LEN(DATA_LEN), .ADDRESS_SIZE(ADDRESS_SIZE), .OFFSET(WRITE_B_ADDR_OFFSET)) w0
(
	.i_clk 		 	(i_clk ),
	.i_rstn 		(i_rstn),
	.i_write_start  (write_start),
	.i_in_mat 		(w_write_cnn_B),				//port mapping!
	 
	.o_write_addr 	(write_address_B),
	.o_write_data 	(o_write_data_B),
	.o_write_start  (w_en_B),
	.o_state 		(write_state_b)
	// .o_done		()
);

																//OFFSET mapping!
M10K_write #(.DATA_LEN(DATA_LEN), .ADDRESS_SIZE(ADDRESS_SIZE), .OFFSET(WRITE_C_ADDR_OFFSET)) w1
(
	.i_clk 		 	(i_clk ),
	.i_rstn 		(i_rstn),
	.i_write_start  (write_start),
	.i_in_mat 		(w_write_cnn_C),				//port mapping!
	 
	.o_write_addr 	(write_address_C),
	.o_write_data 	(o_write_data_C),
	.o_write_start  (w_en_C),
	.o_state 		(write_state_c)
	// .o_done		()
);


endmodule
