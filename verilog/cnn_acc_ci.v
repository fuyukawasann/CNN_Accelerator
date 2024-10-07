//`include "timescale.vh"

module cnn_acc_ci (
    // Clock & Reset
    clk             ,
    reset_n         ,
    i_soft_reset    ,
    i_cnn_weight    ,
    i_in_valid      ,
    i_in_fmap       ,
    o_ot_valid      ,
    o_ot_ci_acc              
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
input     [ICH*KX*KY*DATA_LEN-1 : 0]        i_cnn_weight 	;
input                                       i_in_valid  	;
input     [ICH*IX*IY*DATA_LEN-1 : 0]        i_in_fmap    	;
output                                      o_ot_valid  	;
output    [OX*OY*DATA_LEN-1 : 0]  		    o_ot_ci_acc 	;

//==============================================================================
// Data Enable Signals 
//==============================================================================
wire    [LATENCY-1 : 0] 	ce;
reg     [LATENCY-1 : 0] 	r_valid;
wire    [ICH*OX*OY-1 : 0]   w_ot_valid;
reg                         go_acc;
always @(posedge clk or negedge reset_n) begin
    if(!reset_n) begin
        r_valid   <= {LATENCY{1'b0}};
    end else if(i_soft_reset) begin
        r_valid   <= {LATENCY{1'b0}};
    end else begin
        r_valid[LATENCY-1]  <= go_acc;
    end
end

assign	ce = r_valid;

//==============================================================================
// mul_acc kenel instance
//==============================================================================

wire    [ICH*OX*OY-1 : 0]               w_in_valid;
// wire    [ICH*OX*OY*DATA_LEN-1 : 0]      w_ot_kernel_acc;

// wire signed [DATA_LEN-1:0] parse_w_in_fmap_kernel[0:ICH-1][0:OY-1][0:OX-1][0:KY-1][0:KX-1];

// genvar ich, oy, ox;
// genvar j;
// generate
//     for (ich = 0; ich < ICH; ich = ich+1) begin : gen_ich
//         wire    [KX*KY*DATA_LEN-1 : 0]      w_cnn_weight    = i_cnn_weight[ich*KX*KY*DATA_LEN +: KX*KY*DATA_LEN];
//         wire    [IX*IY*DATA_LEN-1 : 0]  	w_in_fmap    	= i_in_fmap[ich*IX*IY*DATA_LEN +: IX*IY*DATA_LEN];
//         for (oy = 0; oy < OY; oy = oy+1) begin  : gen_oy
//             for (ox = 0; ox < OX; ox = ox+1) begin : gen_ox
//                 // wire [KX*KY*DATA_LEN-1 : 0] w_in_fmap_kernel   = {w_in_fmap[((oy+0)*IX+ox)*DATA_LEN +: KX*DATA_LEN], w_in_fmap[((oy+1)*IX+ox)*DATA_LEN +: KX*DATA_LEN], w_in_fmap[((oy+2)*IX+ox)*DATA_LEN +: KX*DATA_LEN]};
//                 wire [KX*KY*DATA_LEN-1 : 0] w_in_fmap_kernel   = {w_in_fmap[((oy+2)*IX+ox)*DATA_LEN +: KX*DATA_LEN], w_in_fmap[((oy+1)*IX+ox)*DATA_LEN +: KX*DATA_LEN], w_in_fmap[((oy+0)*IX+ox)*DATA_LEN +: KX*DATA_LEN]};

//                 // for (j = 0; j<KX; j=j+1) begin : PARSE_W_FMAP_KERNEL
//                 //     assign parse_w_in_fmap_kernel[ich][oy][ox][0][j] = w_in_fmap_kernel[(KX*DATA_LEN * 0) + (DATA_LEN * j) +: DATA_LEN];
// 				// 	assign parse_w_in_fmap_kernel[ich][oy][ox][1][j] = w_in_fmap_kernel[(KX*DATA_LEN * 1) + (DATA_LEN * j) +: DATA_LEN];
// 				// 	assign parse_w_in_fmap_kernel[ich][oy][ox][2][j] = w_in_fmap_kernel[(KX*DATA_LEN * 2) + (DATA_LEN * j) +: DATA_LEN];
//                 // end

//                 assign	w_in_valid[ich*OY*OX + oy*OX + ox] = i_in_valid;

//                 cnn_kernel u_cnn_kernel(
//                 .clk             (clk            ),
//                 .reset_n         (reset_n        ),
//                 .i_soft_reset    (i_soft_reset   ),
//                 .i_cnn_weight    (w_cnn_weight   ),
//                 .i_in_valid      (w_in_valid[ich*OY*OX + oy*OX + ox]),
//                 .i_in_fmap       (w_in_fmap_kernel),
//                 .o_ot_valid      (w_ot_valid[ich*OY*OX + oy*OX + ox]),
//                 .o_ot_kernel_acc (w_ot_kernel_acc[(ich*OY*OX + oy*OX + ox)*DATA_LEN +: DATA_LEN])             
//                 );
//             end
//         end
//     end
// endgenerate

// Pipelined
parameter IDLE_ACC = 5'b00000;
parameter ACC0 = 5'b00001;
parameter ACC1 = 5'b00010;
parameter ACC2 = 5'b00011;
parameter ACC3 = 5'b00100;
parameter ACC4 = 5'b00101;
parameter ACC5 = 5'b00110;
parameter ACC6 = 5'b00111;
parameter ACC7 = 5'b01000;
parameter ACC8 = 5'b01001;
parameter ACC9 = 5'b01010;
parameter ACC10 = 5'b01011;
parameter ACC11 = 5'b01100;
parameter ACC12 = 5'b01101;
parameter ACC13 = 5'b01110;
parameter ACC14 = 5'b01111;
parameter ACC15 = 5'b10000;
parameter ACC16 = 5'b10001;
parameter ACC17 = 5'b10010;
parameter ACC18 = 5'b10011;
parameter ACC19 = 5'b10100;
parameter ACC20 = 5'b10101;
parameter ACC21 = 5'b10110;
parameter ACC22 = 5'b10111;
parameter ACC23 = 5'b11000;
parameter ACC24 = 5'b11001;
parameter ACC25 = 5'b11010;
parameter ACC26 = 5'b11011;
parameter ACC27 = 5'b11100;
parameter ACC28 = 5'b11101;
parameter ACC29 = 5'b11110;
parameter DONE_ACC = 5'b11111;

reg [4:0] state_ACC;
reg [4:0] next_state_ACC;

reg     [ICH*OX*OY*DATA_LEN-1 : 0]      r_ot_kernel_acc;
reg     [KX*KY*DATA_LEN-1 : 0]          r_cnn_weight;
reg     [IX*IY*DATA_LEN-1 : 0]  	    r_in_fmap;
reg     [KX*KY*DATA_LEN-1 : 0]          r_in_fmap_kernel;
reg                                     r_in_valid_ctrl_acc;
reg                                     r_in_valid_initial_acc;
reg                                     r_in_valid_acc;
reg     [ICH*OX*OY-1 : 0]               r_ot_valid_acc;
wire                                    w_ot_valid_acc_temp;
// reg     [ICH*IX*IY*DATA_LEN-1 : 0]      r_ot_kernel_acc;
wire    [DATA_LEN-1:0]                  w_ot_kernel_acc_temp;
// reg     [ICH*IX*IY*DATA_LEN-1 : 0]      r_in_fmap_kernel;
wire    [ICH*OX*OY*DATA_LEN-1 : 0]      w_ot_kernel_acc;


// State Update
always @(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n) begin
        state_ACC <= IDLE_ACC;
    end else if(i_soft_reset) begin
        state_ACC <= IDLE_ACC;
    end
    else begin
        state_ACC <= next_state_ACC;
    end
end

// Next State Logic
always @(*) begin
    case(state_ACC)
        IDLE_ACC: begin
            if(i_in_valid) begin
                next_state_ACC <= ACC0;
                r_in_valid_initial_acc <= 1'b1;
            end
            else begin
                next_state_ACC <= IDLE_ACC;
                r_in_valid_initial_acc <= 1'b0;
            end
        end

        ACC0: begin
            r_in_fmap <= i_in_fmap[0*IX*IY*DATA_LEN +: IX*IY*DATA_LEN];
            r_cnn_weight <= i_cnn_weight[0*KX*KY*DATA_LEN +: KX*KY*DATA_LEN];
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+0)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC0;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC1;
                r_ot_valid_acc[0] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[0*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC1: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+1)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC1;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC2;
                r_ot_valid_acc[1] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[1*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC2: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+2)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC2;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC3;
                r_ot_valid_acc[2] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[2*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC3: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+3)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC3;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC4;
                r_ot_valid_acc[3] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[3*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC4: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+4)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC4;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC5;
                r_ot_valid_acc[4] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[4*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC5: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+0)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC5;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC6;
                r_ot_valid_acc[5] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[5*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC6: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+1)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC6;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC7;
                r_ot_valid_acc[6] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[6*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC7: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+2)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC7;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC8;
                r_ot_valid_acc[7] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[7*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC8: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+3)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC8;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC9;
                r_ot_valid_acc[8] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[8*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC9: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+4)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC9;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC10;
                r_ot_valid_acc[9] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[9*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        // ICH <= 1
        ACC10: begin
            r_in_fmap <= i_in_fmap[1*IX*IY*DATA_LEN +: IX*IY*DATA_LEN];
            r_cnn_weight <= i_cnn_weight[1*KX*KY*DATA_LEN +: KX*KY*DATA_LEN];
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+0)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC10;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC11;
                r_ot_valid_acc[10] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[10*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC11: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+1)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC11;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC12;
                r_ot_valid_acc[11] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[11*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC12: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+2)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC12;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC13;
                r_ot_valid_acc[12] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[12*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC13: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+3)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC13;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC14;
                r_ot_valid_acc[13] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[13*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC14: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+4)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC14;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC15;
                r_ot_valid_acc[14] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[14*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC15: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+0)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC15;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC16;
                r_ot_valid_acc[15] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[15*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC16: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+1)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC16;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC17;
                r_ot_valid_acc[16] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[16*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC17: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+2)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC17;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC18;
                r_ot_valid_acc[17] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[17*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC18: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+3)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC18;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC19;
                r_ot_valid_acc[18] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[18*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC19: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+4)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC19;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC20;
                r_ot_valid_acc[19] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[19*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        // ICH = 2
        ACC20: begin
            r_in_fmap <= i_in_fmap[2*IX*IY*DATA_LEN +: IX*IY*DATA_LEN];
            r_cnn_weight <= i_cnn_weight[2*KX*KY*DATA_LEN +: KX*KY*DATA_LEN];
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+0)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC20;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC21;
                r_ot_valid_acc[20] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[20*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC21: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+1)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC21;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC22;
                r_ot_valid_acc[21] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[21*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC22: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+2)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC22;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC23;
                r_ot_valid_acc[22] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[22*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC23: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+3)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC23;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC24;
                r_ot_valid_acc[23] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[23*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC24: begin
            r_in_fmap_kernel <= {r_in_fmap[((0+2)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+1)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((0+0)*IX+4)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC24;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC25;
                r_ot_valid_acc[24] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[24*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC25: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+0)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+0)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC25;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC26;
                r_ot_valid_acc[25] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[25*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC26: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+1)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+1)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC26;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC27;
                r_ot_valid_acc[26] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[26*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC27: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+2)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+2)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC27;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC28;
                r_ot_valid_acc[27] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[27*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC28: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+3)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+3)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC28;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= ACC29;
                r_ot_valid_acc[28] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[28*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
                r_in_valid_initial_acc <= 1'b1;
            end
        end

        ACC29: begin
            r_in_fmap_kernel <= {r_in_fmap[((1+2)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+1)*IX+4)*DATA_LEN +: KX*DATA_LEN], r_in_fmap[((1+0)*IX+4)*DATA_LEN +: KX*DATA_LEN]};
            if(!w_ot_valid_acc_temp) begin
                next_state_ACC <= ACC29;
                if(r_in_valid_ctrl_acc) begin
                    r_in_valid_acc <= 1'b1;
                    r_in_valid_initial_acc <= 1'b0;
                end
                else r_in_valid_acc <= 1'b0;
            end
            else begin
                next_state_ACC <= DONE_ACC;
                r_ot_valid_acc[29] <= w_ot_valid_acc_temp;
                r_ot_kernel_acc[29*DATA_LEN +: DATA_LEN] <= w_ot_kernel_acc_temp;
            end
        end

        DONE_ACC: begin
            next_state_ACC <= IDLE_ACC;
            r_in_valid_acc <= 1'b0;
            r_in_valid_initial_acc <= 1'b0;
            r_ot_valid_acc <= {ICH*OX*OY{1'b0}};
            r_ot_kernel_acc <= {ICH*OX*OY*DATA_LEN{1'b0}};
            // w_ot_kernel_acc_temp = {DATA_LEN{1'b0}};
        end
    endcase

end

assign w_ot_valid = r_ot_valid_acc;

reg     [ICH*OX*OY*DATA_LEN-1 : 0]      r_ot_kernel_acc_real;


always @(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n) begin
        r_ot_kernel_acc_real <= {ICH*OX*OY*DATA_LEN{1'b0}};
        go_acc <= 1'b0;
    end
    else if(i_soft_reset) begin
        r_ot_kernel_acc_real <= {ICH*OX*OY*DATA_LEN{1'b0}};
        go_acc <= 1'b0;
    end
    else if(&w_ot_valid) begin
        r_ot_kernel_acc_real <= r_ot_kernel_acc;
        go_acc <= 1'b1;
    end
    else go_acc <= 1'b0;
end
assign w_ot_kernel_acc = r_ot_kernel_acc_real;


// IN-valid-control
always@(posedge clk or negedge reset_n or posedge i_soft_reset) begin
    if(!reset_n) begin
        r_in_valid_ctrl_acc <= 1'b0;
    end
    else if(i_soft_reset) begin
        r_in_valid_ctrl_acc <= 1'b0;
    end
    else begin
        if(r_in_valid_initial_acc) r_in_valid_ctrl_acc <= 1'b1;
        else r_in_valid_ctrl_acc <= 1'b0;
    end
end



// Module Instance
cnn_kernel u_cnn_kernel(
    .clk             (clk            ),
    .reset_n         (reset_n        ),
    .i_soft_reset    (i_soft_reset   ),
    .i_cnn_weight    (r_cnn_weight   ),
    .i_in_valid      (r_in_valid_acc),
    .i_in_fmap       (r_in_fmap_kernel),
    .o_ot_valid      (w_ot_valid_acc_temp),
    .o_ot_kernel_acc (w_ot_kernel_acc_temp)             
);


//==============================================================================
// ci_acc = ci_acc + kernel_acc
//==============================================================================

wire    [OX*OY*DATA_LEN-1 : 0]  		w_ot_ci_acc;
reg     [OX*OY*DATA_LEN-1 : 0]  		r_ot_ci_acc;
reg     [OX*OY*DATA_LEN-1 : 0]  		ot_ci_acc;


// TODO Logic
// to accumulate the output of each Kernel
integer i;
always @(*) begin
	ot_ci_acc = {OX*OY*DATA_LEN{1'b0}};
    for(i = 0; i < ICH; i = i+1) begin
        ot_ci_acc = ot_ci_acc + w_ot_kernel_acc[i*OX*OY*DATA_LEN +: OX*OY*DATA_LEN];
    end
 
end

assign w_ot_ci_acc = ot_ci_acc;

///
// always@(*) begin
//     if(i_in_valid) r_ot_ci_acc <= {OX*OY*DATA_LEN{1'b0}};
// end
///

// F/F
// always @(posedge clk or negedge reset_n) begin
//     if(!reset_n) begin
//         r_ot_ci_acc[0 +: OX*OY*DATA_LEN] <= {OX*OY*DATA_LEN{1'b0}};
//     end else if(i_soft_reset) begin
//         r_ot_ci_acc[0 +: OX*OY*DATA_LEN] <= {OX*OY*DATA_LEN{1'b0}};
//     end else if(&w_ot_valid)begin
//         r_ot_ci_acc[0 +: OX*OY*DATA_LEN] <= w_ot_ci_acc[0 +: OX*OY*DATA_LEN];
//     end
// end

assign o_ot_valid = r_valid[LATENCY-1];
// assign o_ot_ci_acc = r_ot_ci_acc;
assign o_ot_ci_acc = w_ot_ci_acc;

endmodule
