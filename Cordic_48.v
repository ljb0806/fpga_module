// =============================================================================
// module   : Cordic
// function : 48-bit phase CORDIC sine/cosine generator
//
// Phase format:
//   48-bit signed circular phase. 2*pi corresponds to 2^48 phase codes:
//     48'h0000_0000_0000 =  0
//     48'h4000_0000_0000 = +pi/2
//     48'h8000_0000_0000 = -pi (the same point as +pi)
//     48'hC000_0000_0000 = -pi/2
//
// Output format:
//   Signed Q1.(DOUT_WIDTH-1). The internal x/y datapath is always 48 bits;
//   only the final output is reduced to DOUT_WIDTH bits.
//
// Pipeline:
//   One input can be accepted every clock. The quadrant mapping occupies the
//   first register and is followed by 47 effective CORDIC rotations.
// =============================================================================

module Cordic #(
    parameter integer DOUT_WIDTH = 16
)(
    input  wire                         i_clk,
    input  wire                         i_rst_n,

    input  wire signed [47:0]           i_phase,
    output wire signed [DOUT_WIDTH-1:0] o_cos,
    output wire signed [DOUT_WIDTH-1:0] o_sin
);

localparam integer PHASE_WIDTH = 48;
// For a 48-bit circular phase, atan(2^-46) is the last angle that still
// quantizes to one phase code. atan(2^-47) would quantize to zero.
localparam integer ITERATIONS  = 47;

// 0.6072 is slightly smaller than the ideal inverse CORDIC gain. The small
// amplitude margin prevents fixed-point round-off near +1 from wrapping to -1.
localparam signed [47:0] CORDIC_GAIN = 48'h4DB8_BAC7_10CB;
localparam signed [47:0] PI          = 48'sh8000_0000_0000;
localparam signed [47:0] HALF_PI     = 48'sh4000_0000_0000;

reg signed [PHASE_WIDTH-1:0] x_reg [0:ITERATIONS];
reg signed [PHASE_WIDTH-1:0] y_reg [0:ITERATIONS];
reg signed [PHASE_WIDTH-1:0] z_reg [0:ITERATIONS];

// atan(2^-i) / pi * 2^47
wire signed [PHASE_WIDTH-1:0] atan_table [0:ITERATIONS-1];
assign atan_table[0]  = 48'h2000_0000_0000;
assign atan_table[1]  = 48'h12E4_051D_9DF3;
assign atan_table[2]  = 48'h09FB_385B_5EE4;
assign atan_table[3]  = 48'h0511_11D4_1DDE;
assign atan_table[4]  = 48'h028B_0D43_0E59;
assign atan_table[5]  = 48'h0145_D7E1_5904;
assign atan_table[6]  = 48'h00A2_F61E_5C28;
assign atan_table[7]  = 48'h0051_7C55_11D4;
assign atan_table[8]  = 48'h0028_BE53_46D1;
assign atan_table[9]  = 48'h0014_5F2E_BB31;
assign atan_table[10] = 48'h000A_2F98_0092;
assign atan_table[11] = 48'h0005_17CC_14A8;
assign atan_table[12] = 48'h0002_8BE6_0CE0;
assign atan_table[13] = 48'h0001_45F3_06C1;
assign atan_table[14] = 48'h0000_A2F9_836B;
assign atan_table[15] = 48'h0000_517C_C1B7;
assign atan_table[16] = 48'h0000_28BE_60DC;
assign atan_table[17] = 48'h0000_145F_306E;
assign atan_table[18] = 48'h0000_0A2F_9837;
assign atan_table[19] = 48'h0000_0517_CC1B;
assign atan_table[20] = 48'h0000_028B_E60E;
assign atan_table[21] = 48'h0000_0145_F307;
assign atan_table[22] = 48'h0000_00A2_F983;
assign atan_table[23] = 48'h0000_0051_7CC2;
assign atan_table[24] = 48'h0000_0028_BE61;
assign atan_table[25] = 48'h0000_0014_5F30;
assign atan_table[26] = 48'h0000_000A_2F98;
assign atan_table[27] = 48'h0000_0005_17CC;
assign atan_table[28] = 48'h0000_0002_8BE6;
assign atan_table[29] = 48'h0000_0001_45F3;
assign atan_table[30] = 48'h0000_0000_A2FA;
assign atan_table[31] = 48'h0000_0000_517D;
assign atan_table[32] = 48'h0000_0000_28BE;
assign atan_table[33] = 48'h0000_0000_145F;
assign atan_table[34] = 48'h0000_0000_0A30;
assign atan_table[35] = 48'h0000_0000_0518;
assign atan_table[36] = 48'h0000_0000_028C;
assign atan_table[37] = 48'h0000_0000_0146;
assign atan_table[38] = 48'h0000_0000_00A3;
assign atan_table[39] = 48'h0000_0000_0051;
assign atan_table[40] = 48'h0000_0000_0029;
assign atan_table[41] = 48'h0000_0000_0014;
assign atan_table[42] = 48'h0000_0000_000A;
assign atan_table[43] = 48'h0000_0000_0005;
assign atan_table[44] = 48'h0000_0000_0003;
assign atan_table[45] = 48'h0000_0000_0001;
assign atan_table[46] = 48'h0000_0000_0001;

// Map the full-circle input to the CORDIC convergence interval [-pi/2, pi/2].
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        x_reg[0] <= 48'sd0;
        y_reg[0] <= 48'sd0;
        z_reg[0] <= 48'sd0;
    end
    else if (i_phase > HALF_PI) begin
        x_reg[0] <= -CORDIC_GAIN;
        y_reg[0] <= 48'sd0;
        z_reg[0] <= i_phase - PI;
    end
    else if (i_phase < -HALF_PI) begin
        x_reg[0] <= -CORDIC_GAIN;
        y_reg[0] <= 48'sd0;
        z_reg[0] <= i_phase + PI;
    end
    else begin
        x_reg[0] <= CORDIC_GAIN;
        y_reg[0] <= 48'sd0;
        z_reg[0] <= i_phase;
    end
end

integer i;
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        for (i = 0; i < ITERATIONS; i = i + 1) begin
            x_reg[i+1] <= 48'sd0;
            y_reg[i+1] <= 48'sd0;
            z_reg[i+1] <= 48'sd0;
        end
    end
    else begin
        for (i = 0; i < ITERATIONS; i = i + 1) begin
            if (z_reg[i] < 0) begin
                x_reg[i+1] <= x_reg[i] + (y_reg[i] >>> i);
                y_reg[i+1] <= y_reg[i] - (x_reg[i] >>> i);
                z_reg[i+1] <= z_reg[i] + atan_table[i];
            end
            else begin
                x_reg[i+1] <= x_reg[i] - (y_reg[i] >>> i);
                y_reg[i+1] <= y_reg[i] + (x_reg[i] >>> i);
                z_reg[i+1] <= z_reg[i] - atan_table[i];
            end
        end
    end
end

// Keep the sign and the most significant fractional bits. DOUT_WIDTH must be
// in the range 2..48.
assign o_cos = x_reg[ITERATIONS][PHASE_WIDTH-1 -: DOUT_WIDTH];
assign o_sin = y_reg[ITERATIONS][PHASE_WIDTH-1 -: DOUT_WIDTH];

endmodule
