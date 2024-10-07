//`include "timescale.vh"

module cnn_kernel (
    // Clock & Reset
    clk             ,
    reset_n         ,
    i_soft_reset    ,
    i_cnn_weight    ,
    i_in_valid      ,
    i_in_fmap       ,
    o_ot_valid      ,
    o_ot_kernel_acc              
    );
//`include "defines_cnn_core.vh"
`include "defines_computer.vh"

localparam LATENCY = 2;
//==============================================================================
// Input/Output declaration
//==============================================================================
input                               		clk         	;
input                               		reset_n     	;
input                               		i_soft_reset	;
input     [KX*KY*DATA_LEN-1 : 0]  			i_cnn_weight 	;
input                               		i_in_valid  	;
input     [KX*KY*DATA_LEN-1 : 0]  			i_in_fmap    	;
output                              		o_ot_valid  	;
output    [DATA_LEN-1 : 0]  				o_ot_kernel_acc ;
		
//==============================================================================
// Data Enable Signals 
//==============================================================================
wire    [LATENCY-1 : 0] 	ce;
reg     [LATENCY-1 : 0] 	r_valid;
reg 						go_acc;
always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        r_valid   <= {LATENCY{1'b0}};
    end else if(i_soft_reset) begin
        r_valid   <= {LATENCY{1'b0}};
    end else begin
        // r_valid[LATENCY-2]  <= i_in_valid;
		r_valid[LATENCY-2]  <= go_acc; // for pipelined version
        r_valid[LATENCY-1]  <= r_valid[LATENCY-2];
    end
end

assign	ce = r_valid;

//==============================================================================
// mul = fmap * weight
//==============================================================================

// original source
// wire      [KY*KX*DATA_LEN-1 : 0]    mul  ;
// reg       [KY*KX*DATA_LEN-1 : 0]    r_mul;

// genvar mul_idx;
// generate
// 	for(mul_idx = 0; mul_idx < KY*KX; mul_idx = mul_idx + 1) begin : gen_mul
// 		assign  mul[mul_idx * DATA_LEN +: DATA_LEN]	= i_in_fmap[mul_idx * DATA_LEN +: DATA_LEN] * i_cnn_weight[mul_idx * DATA_LEN +: DATA_LEN];

// 		always @(posedge clk or negedge reset_n) begin
// 		    if(!reset_n) begin
// 		        r_mul[mul_idx * DATA_LEN +: DATA_LEN] <= {DATA_LEN{1'b0}};
// 		    end else if(i_soft_reset) begin
// 		        r_mul[mul_idx * DATA_LEN +: DATA_LEN] <= {DATA_LEN{1'b0}};
// 		    end else if(i_in_valid)begin
// 		        r_mul[mul_idx *DATA_LEN +: DATA_LEN] <= mul[mul_idx * DATA_LEN +: DATA_LEN];
// 		    end
// 		end
// 	end
// endgenerate

// pipelined version.
// wire      [KY*KX*DATA_LEN-1 : 0]    mul;
reg       [KY*KX*DATA_LEN-1 : 0]    r_mul;
reg		  [KY*KX*DATA_LEN-1 : 0] 		  	r_mul_temp;
reg 	  [3:0]						state, next_state;

parameter IDLE = 4'b0000;
parameter KERNEL0 = 4'b0001;
parameter KERNEL1 = 4'b0010;
parameter KERNEL2 = 4'b0011;
parameter KERNEL3 = 4'b0100;
parameter KERNEL4 = 4'b0101;
parameter KERNEL5 = 4'b0110;
parameter KERNEL6 = 4'b0111;
parameter KERNEL7 = 4'b1000;
parameter KERNEL8 = 4'b1001;
parameter KERNEL9 = 4'b1010;
parameter KERNEL10 = 4'b1011;
parameter KERNEL11 = 4'b1100;
parameter DONE = 4'b1101;

reg my_valid;

// In-valid -> my-valid
// always @(posedge clk or negedge reset_n) begin
// 	if(!reset_n) begin
// 		my_valid <= 1'b0;
// 	end else if(i_soft_reset) begin
// 		my_valid <= 1'b0;
// 	end else if(i_in_valid)begin
// 		my_valid <= 1'b1;
// 	end
// end

// State register
always@(posedge clk or negedge reset_n or posedge i_soft_reset) begin
	if(!reset_n) state <= IDLE;
	else if(i_soft_reset) state <= IDLE;
	else state <= next_state;
end

// Next state logic
always@(*) begin
	if(!reset_n) begin
		//state <= IDLE;
		r_mul_temp <= {KX*KY*DATA_LEN{1'b0}};
		go_acc <= 1'b0;
	end
	else if(i_soft_reset) begin
		//state <= IDLE;
		r_mul_temp <= {KX*KY*DATA_LEN{1'b0}};
		go_acc <= 1'b0;
	end
	else begin
		case(state)
			IDLE: 	begin
						if(i_in_valid)next_state <= KERNEL0;
						else next_state <= IDLE;
						//r_mul_temp <= {KX*KY*DATA_LEN{1'b0}};
						// go_acc <= 1'b0;
					end
			KERNEL0: begin
						next_state <= KERNEL1;
						r_mul_temp[0*DATA_LEN +: DATA_LEN] <= i_in_fmap[0*DATA_LEN +: DATA_LEN] * i_cnn_weight[0*DATA_LEN +: DATA_LEN];
					end
			KERNEL1: begin
						next_state <= KERNEL2;
						r_mul_temp[1*DATA_LEN +: DATA_LEN] <= i_in_fmap[1*DATA_LEN +: DATA_LEN] * i_cnn_weight[1*DATA_LEN +: DATA_LEN];
					end
			KERNEL2: begin
						next_state <= KERNEL3;
						r_mul_temp[2*DATA_LEN +: DATA_LEN] <= i_in_fmap[2*DATA_LEN +: DATA_LEN] * i_cnn_weight[2*DATA_LEN +: DATA_LEN];
					end
			KERNEL3: begin
						next_state <= KERNEL4;
						r_mul_temp[3*DATA_LEN +: DATA_LEN] <= i_in_fmap[3*DATA_LEN +: DATA_LEN] * i_cnn_weight[3*DATA_LEN +: DATA_LEN];
					end
			KERNEL4: begin
						next_state <= KERNEL5;
						r_mul_temp[4*DATA_LEN +: DATA_LEN] <= i_in_fmap[4*DATA_LEN +: DATA_LEN] * i_cnn_weight[4*DATA_LEN +: DATA_LEN];
					end
			KERNEL5: begin
						next_state <= KERNEL6;
						r_mul_temp[5*DATA_LEN +: DATA_LEN] <= i_in_fmap[5*DATA_LEN +: DATA_LEN] * i_cnn_weight[5*DATA_LEN +: DATA_LEN];
					end
			KERNEL6: begin
						next_state <= KERNEL7;
						r_mul_temp[6*DATA_LEN +: DATA_LEN] <= i_in_fmap[6*DATA_LEN +: DATA_LEN] * i_cnn_weight[6*DATA_LEN +: DATA_LEN];
					end
			KERNEL7: begin
						next_state <= KERNEL8;
						r_mul_temp[7*DATA_LEN +: DATA_LEN] <= i_in_fmap[7*DATA_LEN +: DATA_LEN] * i_cnn_weight[7*DATA_LEN +: DATA_LEN];
					end
			KERNEL8: begin
						next_state <= KERNEL9;
						r_mul_temp[8*DATA_LEN +: DATA_LEN] <= i_in_fmap[8*DATA_LEN +: DATA_LEN] * i_cnn_weight[8*DATA_LEN +: DATA_LEN];
					end
			KERNEL9: begin
						next_state <= KERNEL10;
						r_mul_temp[9*DATA_LEN +: DATA_LEN] <= i_in_fmap[9*DATA_LEN +: DATA_LEN] * i_cnn_weight[9*DATA_LEN +: DATA_LEN];
					end
			KERNEL10: begin
						next_state <= KERNEL11;
						r_mul_temp[10*DATA_LEN +: DATA_LEN] <= i_in_fmap[10*DATA_LEN +: DATA_LEN] * i_cnn_weight[10*DATA_LEN +: DATA_LEN];
					end
			KERNEL11: begin
						next_state <= DONE;
						r_mul_temp[11*DATA_LEN +: DATA_LEN] <= i_in_fmap[11*DATA_LEN +: DATA_LEN] * i_cnn_weight[11*DATA_LEN +: DATA_LEN];
						go_acc <= 1'b1;
					end
			DONE: 	begin
						next_state <= IDLE;
						go_acc <= 1'b0;
						r_mul_temp <= {KX*KY*DATA_LEN{1'b0}};
					end
		endcase
	end
end

// always @(posedge clk or negedge reset_n) begin
// 	if(!reset_n) begin
// 		my_valid <= 1'b0;
// 	end else if(i_soft_reset) begin
// 		my_valid <= 1'b0;
// 	end else if(i_in_valid)begin
// 		my_valid <= 1'b1;
// 	end
// end

always @(posedge clk or negedge reset_n or posedge i_soft_reset) begin
	if(!reset_n) begin
		r_mul <= {KX*KY*DATA_LEN{1'b0}};
		my_valid <= 1'b0;
	end else if(i_soft_reset) begin
		r_mul <= {KX*KY*DATA_LEN{1'b0}};
		my_valid <= 1'b0;
	end else if(i_in_valid) begin
		my_valid <= 1'b1;
	end else if(my_valid && next_state == DONE)begin
		r_mul <= r_mul_temp;
		my_valid <= 1'b0;
	end
end




//==============================================================================
// acc = acc + mul
//==============================================================================

reg       [DATA_LEN-1 : 0]    acc_kernel 	;
reg       [DATA_LEN-1 : 0]    r_acc_kernel  ;

// TODO Logic
// to accumulate all multiplication results. if use for-loop, you can use the template below
integer acc_idx;
always @ (*) begin
	acc_kernel[0 +: DATA_LEN]= {DATA_LEN{1'b0}};
	for(acc_idx =0; acc_idx < KY*KX; acc_idx = acc_idx +1) begin
		acc_kernel[0 +: DATA_LEN] = acc_kernel[0 +: DATA_LEN] + r_mul[acc_idx*DATA_LEN +: DATA_LEN]; 
	end
end


// F/F
always @(posedge clk or negedge reset_n) begin
	if(!reset_n) begin
		r_acc_kernel[0 +: DATA_LEN] <= {DATA_LEN{1'b0}};
	end else if(i_soft_reset) begin
		r_acc_kernel[0 +: DATA_LEN] <= {DATA_LEN{1'b0}};
	end else if(ce[LATENCY-2])begin
		r_acc_kernel[0 +: DATA_LEN] <= acc_kernel[0 +: DATA_LEN];
	end
end


assign o_ot_valid = r_valid[LATENCY-1];
assign o_ot_kernel_acc = r_acc_kernel;

endmodule
