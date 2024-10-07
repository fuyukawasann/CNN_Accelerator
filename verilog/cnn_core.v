//`include "timescale.vh"

module cnn_core (
    // Clock & Reset
    clk             ,
    reset_n         ,
    i_soft_reset    ,
    i_cnn_weight    ,
    i_in_valid      ,
    i_in_fmap       ,
    o_ot_valid      ,
    o_ot_one_fmap
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
input     [ICH*IX*IY*DATA_LEN-1 : 0]        i_in_fmap    	;
output                                      o_ot_valid  	;
output    [OCH*OX*OY*DATA_LEN-1 : 0]        o_ot_one_fmap   ;

//==============================================================================
// Data Enable Signals 
//==============================================================================
wire    [LATENCY-1 : 0] 	ce;
reg     [LATENCY-1 : 0] 	r_valid;
wire    [OCH-1 : 0]         w_ot_valid;
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
// acc ci instance
//==============================================================================

// wire    [OCH-1 : 0]                     w_in_valid;
// wire    [OCH*OX*OY*DATA_LEN-1 : 0]      w_ot_ci_acc;

// TODO Instantiation
// to call cnn_acc_ci instance. if use generate, you can use the template below.
// genvar ci_inst;
// generate
// 	for(ci_inst = 0; ci_inst < OCH; ci_inst = ci_inst + 1) begin : gen_ci_inst
//         wire    [ICH*KX*KY*DATA_LEN-1 : 0]  w_cnn_weight 	= i_cnn_weight[ci_inst*ICH*KX*KY*DATA_LEN +: ICH*KX*KY*DATA_LEN];
//         wire    [ICH*IX*IY*DATA_LEN-1 : 0]  w_in_fmap    	= i_in_fmap[0 +: ICH*IX*IY*DATA_LEN];
//         assign	w_in_valid[ci_inst] = i_in_valid; 

//         cnn_acc_ci u_cnn_acc_ci(
//         .clk             (clk         ),
//         .reset_n         (reset_n     ),
//         .i_soft_reset    (i_soft_reset),
//         .i_cnn_weight    (w_cnn_weight),
//         .i_in_valid      (w_in_valid[ci_inst]),
//         .i_in_fmap       (w_in_fmap),
//         .o_ot_valid      (w_ot_valid[ci_inst]),
//         .o_ot_ci_acc     (w_ot_ci_acc[ci_inst*OX*OY*(DATA_LEN) +: OX*OY*(DATA_LEN)])
//         );
// 	end
// endgenerate

// My Code Here
wire    [OCH*OX*OY*DATA_LEN-1 : 0]      w_ot_ci_acc;
reg                                     r_in_valid;
reg                                     r_in_valid_ctrl;
reg                                     r_in_valid_initial;
reg     [ICH*KX*KY*DATA_LEN-1 : 0]      r_cnn_weight;
reg     [1:0]                           state;
reg     [1:0]                           next_state;
parameter IDLE = 2'b00;
parameter CNN0 = 2'b01;
parameter CNN1 = 2'b10;
parameter DONE = 2'b11;

reg [OCH-1:0]                               r_ot_valid;
reg [OCH*OX*OY*DATA_LEN-1:0]                s_ot_ci_acc;
wire                                        this_ot_valid;
wire [OX*OY*DATA_LEN-1:0]                   this_ot_ci_acc;

// state register
always@(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n)    state <= IDLE;
    else if(i_soft_reset)    state <= IDLE;
    else            state <= next_state;
end

// Machine
always@(*) begin
    if(!reset_n) begin
        //state <= IDLE;
        r_cnn_weight <= {ICH*KX*KY*DATA_LEN{1'b0}};
        r_in_valid_initial <= 1'b0;
    end
    else if(i_soft_reset) begin
        //state <= IDLE;
        r_cnn_weight <= {ICH*KX*KY*DATA_LEN{1'b0}};
        r_in_valid_initial <= 1'b1;
    end
    else begin
        case(state)
            IDLE:  begin
                        if(i_in_valid) begin
                            next_state <= CNN0;
                            r_in_valid_initial <= 1'b1;
                        end
                        else begin
                            next_state <= IDLE;
                            r_in_valid_initial <= 1'b0;
                        end
                        r_cnn_weight <= i_cnn_weight[0*ICH*KX*KY*DATA_LEN +: ICH*KX*KY*DATA_LEN];
                        r_ot_valid <= {OCH{1'b0}};
                        s_ot_ci_acc <= {OCH*OX*OY*DATA_LEN{1'b0}};
                        r_in_valid <= i_in_valid;
                    end
            CNN0:   begin
                        if(!this_ot_valid) begin
                            if(r_in_valid_ctrl) r_in_valid <= 1'b1;
                            else r_in_valid <= i_in_valid;
                            next_state <= CNN0;
                            r_cnn_weight <= i_cnn_weight[0*ICH*KX*KY*DATA_LEN +: ICH*KX*KY*DATA_LEN];
                            r_in_valid <= i_in_valid;
                            //r_ot_valid[0] <= this_ot_valid;
                            s_ot_ci_acc[0*OX*OY*(DATA_LEN) +: OX*OY*(DATA_LEN)] <= this_ot_ci_acc;
                        end
                        else begin
                            next_state <= CNN1;
                            r_ot_valid[0] <= this_ot_valid;
                            s_ot_ci_acc[0*OX*OY*(DATA_LEN) +: OX*OY*(DATA_LEN)] <= this_ot_ci_acc;
                            r_cnn_weight <= i_cnn_weight[1*ICH*KX*KY*DATA_LEN +: ICH*KX*KY*DATA_LEN];
                            r_in_valid <= i_in_valid;
                            r_in_valid_initial <= 1'b1;
                        end
                    end
            CNN1:   begin
                        if(!this_ot_valid) begin
                            if(r_in_valid_ctrl) r_in_valid <= 1'b1;
                            else r_in_valid <= i_in_valid;
                            r_in_valid_initial <= 1'b0;
                            next_state <= CNN1;
                            r_cnn_weight <= i_cnn_weight[1*ICH*KX*KY*DATA_LEN +: ICH*KX*KY*DATA_LEN];
                            s_ot_ci_acc[1*OX*OY*(DATA_LEN) +: OX*OY*(DATA_LEN)] <= this_ot_ci_acc;
                        end
                        else begin
                            next_state <= DONE;
                            r_ot_valid[1] <= this_ot_valid;
                            s_ot_ci_acc[1*OX*OY*(DATA_LEN) +: OX*OY*(DATA_LEN)] <= this_ot_ci_acc;
                        end
                    end
            DONE:   begin
                        next_state <= IDLE;
                        //r_cnn_weight <= {ICH*KX*KY*DATA_LEN{1'b0}};
                        s_ot_ci_acc <= {OCH*OX*OY*DATA_LEN{1'b0}};
                        r_ot_valid <= {OCH{1'b0}};
                    end
        endcase
        
    end
end

// In-valid-control
always@(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n) begin
        r_in_valid_ctrl <= 1'b0;
    end
    else if(i_soft_reset) begin
        r_in_valid_ctrl <= 1'b0;
    end
    else begin
        if(r_in_valid_initial)  r_in_valid_ctrl <= 1'b1;
        else
        r_in_valid_ctrl <= 1'b0;
    end
end


assign w_ot_valid = r_ot_valid;
assign w_ot_ci_acc = s_ot_ci_acc;



cnn_acc_ci u_cnn_acc_ci(
        .clk             (clk         ),
        .reset_n         (reset_n     ),
        .i_soft_reset    (i_soft_reset),
        .i_cnn_weight    (r_cnn_weight),
        .i_in_valid      (r_in_valid),
        .i_in_fmap       (i_in_fmap),
        .o_ot_valid      (this_ot_valid),
        .o_ot_ci_acc     (this_ot_ci_acc)
        );

//

reg         [OCH*OX*OY*DATA_LEN-1 : 0]  r_ot_ci_acc;

always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        r_ot_ci_acc <= 0;
    end else if(i_soft_reset) begin
        r_ot_ci_acc <= 0;
    end else if(&w_ot_valid) begin
        r_ot_ci_acc <= s_ot_ci_acc;
    end
end

//==============================================================================
// No Activation
//==============================================================================
assign o_ot_valid = r_valid[LATENCY-1];
assign o_ot_one_fmap  = r_ot_ci_acc;

endmodule

