//`include "timescale.vh"

module cnn_topCore (
    // Clock & Reset
    clk             ,
    reset_n         ,
    i_soft_reset    ,
    i_cnn_weight    ,
    i_in_valid      ,
    i_in_fmap       ,
    o_ot_valid      ,
    o_ot_fmap             
    );
//`include "defines_cnn_core.vh"
`include "defines_computer.vh"
localparam LATENCY = 1;
//==============================================================================
// Input/Output declaration
//==============================================================================
input                                       clk         	;
input                                       reset_n     	;
input                                       i_soft_reset	;
input     [OCH*ICH*KX*KY*DATA_LEN-1 : 0]    i_cnn_weight 	;
input                                       i_in_valid  	;
input     [IN*ICH*IX*IY*DATA_LEN-1 : 0]     i_in_fmap    	;
output                                      o_ot_valid  	;
output    [IN*OCH*OX*OY*DATA_LEN-1 : 0]     o_ot_fmap    	;

//==============================================================================
// Data Enable Signals 
//==============================================================================
wire    [LATENCY-1 : 0] 	ce;
reg     [LATENCY-1 : 0] 	r_valid;
wire    [IN-1 : 0]          w_ot_valid;
always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        r_valid   <= {LATENCY{1'b0}};
    end else if(i_soft_reset) begin
        r_valid   <= {LATENCY{1'b0}};
    end else begin
        r_valid[LATENCY-1]  <= &w_ot_valid;
    end
end

assign	ce = r_valid;

//==============================================================================
// cnn core instance for stride
//==============================================================================

// wire    [IN-1 : 0]                      w_in_valid;
// wire    [IN*OCH*OX*OY*DATA_LEN-1 : 0]   w_ot_one_fmap;

// TODO Instantiation
// to call cnn_acc_ci instance. if use generate, you can use the template below.
// genvar core_inst;
// generate
// 	for(core_inst = 0; core_inst < IN; core_inst = core_inst+1) begin : gen_core_inst
// 		wire    [OCH*ICH*KX*KY*DATA_LEN-1 : 0]  w_cnn_weight    = i_cnn_weight[0 +: OCH*ICH*KX*KY*DATA_LEN];
// 		wire    [ICH*IX*IY*DATA_LEN-1 : 0]  w_in_fmap           = i_in_fmap[core_inst*ICH*IX*IY*DATA_LEN +: ICH*IX*IY*DATA_LEN];
// 		assign	w_in_valid[core_inst] = i_in_valid; 

// 		cnn_core u_cnn_core(
// 	    .clk             (clk         ),
// 	    .reset_n         (reset_n     ),
// 	    .i_soft_reset    (i_soft_reset),
// 	    .i_cnn_weight    (w_cnn_weight),
// 	    .i_in_valid      (w_in_valid[core_inst]),
// 	    .i_in_fmap       (w_in_fmap),
// 	    .o_ot_valid      (w_ot_valid[core_inst]),
// 	    .o_ot_one_fmap   (w_ot_one_fmap[core_inst*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN])
// 	    );
// 	end
// endgenerate

// My Code here

wire    [OCH*OX*OY*DATA_LEN-1:0]        w_ot_one_fmap;
wire    [OCH*ICH*KX*KY*DATA_LEN-1 : 0]  w_cnn_weight;
reg                                     r_in_valid;
reg                                     r_in_valid_ctrl;
reg                                     r_in_valid_initial;
reg     [ICH*IX*IY*DATA_LEN-1 : 0]      r_in_fmap;
reg    [1:0]                           state;
reg    [1:0]                           next_state;
parameter IDLE      = 2'b00;
parameter CORE0     = 2'b01;
parameter CORE1     = 2'b10;
parameter DONE      = 2'b11;

reg [IN-1:0]    r_ot_valid;
reg [IN*OCH*OX*OY*DATA_LEN-1:0] s_ot_one_fmap;
wire            this_ot_valid;
wire [OCH*OX*OY*DATA_LEN-1:0] this_ot_one_fmap;
reg latency_in_valid;

// state register
always@(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n) state <= IDLE;
    else if (i_soft_reset) state <= IDLE;
    else state <= next_state;
end

// MACHINE
always @(*) begin
    if(!reset_n) begin
        //state <= IDLE;
        r_in_fmap <= {ICH*IX*IY*DATA_LEN{1'b0}};
    end
    else if(i_soft_reset) begin
        //state <= IDLE;
        r_in_fmap <= {ICH*IX*IY*DATA_LEN{1'b0}};
    end
    else begin
        case(state)
            IDLE:  begin
                        next_state <= CORE0;
                        r_in_fmap <= i_in_fmap[0*ICH*IX*IY*DATA_LEN +: ICH*IX*IY*DATA_LEN];
                        r_ot_valid <= {IN{1'b0}};
                        r_in_valid <= i_in_valid;
                    end
            CORE0:  begin
                        if(!this_ot_valid) begin
                            next_state <= CORE0;
                            r_in_fmap <= i_in_fmap[0*ICH*IX*IY*DATA_LEN +: ICH*IX*IY*DATA_LEN];
                            r_in_valid <= i_in_valid;
                        end
                        else begin
                            next_state <= CORE1;
                            r_ot_valid[0] <= this_ot_valid;
                            s_ot_one_fmap[0*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN] <= this_ot_one_fmap;
                            r_in_fmap <= i_in_fmap[1*ICH*IX*IY*DATA_LEN +: ICH*IX*IY*DATA_LEN];
                            r_in_valid <= i_in_valid;
                            r_in_valid_initial <= 1'b1;
                        end
                    end
            CORE1:  begin
                        if(!this_ot_valid) begin
                            next_state <= CORE1;
                            r_in_fmap <= i_in_fmap[1*ICH*IX*IY*DATA_LEN +: ICH*IX*IY*DATA_LEN];
                            // if(r_in_valid_initial) begin
                            //     r_in_valid_initial <= 1'b0;
                            //     r_in_valid <= 1'b1;
                            // end
                            // else begin
                            //     r_in_valid <= 1'b0;
                            // end
                            if(r_in_valid_ctrl) begin
                                r_in_valid_initial <= 1'b0;
                                r_in_valid <= 1'b1;
                            end
                            else r_in_valid <= i_in_valid;
                        end
                        else begin
                            next_state <= DONE;
                            r_ot_valid[1] <= this_ot_valid;
                            s_ot_one_fmap[1*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN] <= this_ot_one_fmap;
                        end
                    end
            DONE:   begin
                        if(!reset_n || i_soft_reset) begin
                            next_state <= IDLE;
                            r_ot_valid[0] <= 1'b0;
                            r_ot_valid[0] <= 1'b0;
                            s_ot_one_fmap[0*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN] <= {OCH*OX*OY*DATA_LEN{1'b0}};
                            s_ot_one_fmap[1*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN] <= {OCH*OX*OY*DATA_LEN{1'b0}};
                        end
                        else begin
                            next_state <= DONE;
                            r_ot_valid[0] <= r_ot_valid[0];
                            r_ot_valid[1] <= r_ot_valid[1];
                            s_ot_one_fmap[0*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN] <= s_ot_one_fmap[0*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN];
                            s_ot_one_fmap[1*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN] <= s_ot_one_fmap[1*OCH*OX*OY*DATA_LEN +: OCH*OX*OY*DATA_LEN];
                        end
                    end
        endcase
    end
end


// IN-valid-control
always@(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n) begin
        r_in_valid_ctrl <= 1'b0;
    end
    else if(i_soft_reset) begin
        r_in_valid_ctrl <= 1'b0;
    end
    else begin
        if(r_in_valid_initial) r_in_valid_ctrl <= 1'b1;
        else r_in_valid_ctrl <= 1'b0;
        // if (r_in_valid_initial) begin
        //     latency_in_valid <= 1'b1;
        //     r_in_valid_initial <= 1'b0;
        // end
        // else if(latency_in_valid) begin
        //     r_in_valid <= 1'b1;
        //     latency_in_valid <= 1'b0;
        // end
        // else r_in_valid <= 1'b0;
    end
end


assign w_ot_valid = r_ot_valid;
assign w_ot_one_fmap = s_ot_one_fmap;

cnn_core u_cnn_core(
            .clk             (clk         ),
            .reset_n         (reset_n     ),
            .i_soft_reset    (i_soft_reset),
            .i_cnn_weight    (i_cnn_weight),
            .i_in_valid      (r_in_valid), // originally, i_in_valid
            .i_in_fmap       (r_in_fmap),
            .o_ot_valid      (this_ot_valid),
            .o_ot_one_fmap   (this_ot_one_fmap)
        );
///////

reg     [IN*OCH*OX*OY*DATA_LEN-1 : 0]   r_ot_one_fmap;

always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        r_ot_one_fmap <= 0;
    end else if(i_soft_reset) begin
        r_ot_one_fmap <= 0;
    end else if(&w_ot_valid) begin
        r_ot_one_fmap <= s_ot_one_fmap;
    end
end

//==============================================================================
// No Activation
//==============================================================================
assign o_ot_valid = r_valid[LATENCY-1];
assign o_ot_fmap  = r_ot_one_fmap;

endmodule

