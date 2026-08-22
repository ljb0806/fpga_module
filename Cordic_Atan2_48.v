`timescale 1ns / 1ps

// =============================================================================
// Vectoring-mode CORDIC atan2 and magnitude generator.
//
// Default interface:
//   i_x/i_y       : signed 16-bit Cartesian inputs
//   o_phase       : signed 48-bit circular phase
//   o_phase_mag   : unsigned-magnitude value carried by a signed 16-bit port
//
// Phase coding is identical to Cordic.v:
//   0                       = 0
//   48'h4000_0000_0000     = +pi/2
//   48'h8000_0000_0000     = -pi (the same point as +pi)
//   48'hC000_0000_0000     = -pi/2
//
// PHASE_WIDTH may be set from 2 to 48. Internally the phase is always
// calculated at 48-bit resolution; a narrower output keeps its upper bits.
// DIN_WIDTH must not exceed 48. MAG_WIDTH must not exceed 2*DIN_WIDTH-1.
// =============================================================================

module Cordic_Atan2 #(
    parameter integer DIN_WIDTH   = 16,
    parameter integer PHASE_WIDTH = 48,
    parameter integer MAG_WIDTH   = 16
)(
    input  wire                          i_clk,
    input  wire                          i_rst_n,

    input  wire signed [DIN_WIDTH-1:0]   i_x,
    input  wire signed [DIN_WIDTH-1:0]   i_y,
    output reg  signed [PHASE_WIDTH-1:0] o_phase,
    output reg  signed [MAG_WIDTH-1:0]   o_phase_mag
);

localparam integer INTERNAL_PHASE_WIDTH = 48;
localparam integer XY_WIDTH             = 50;
localparam integer ITERATIONS           = 47;
localparam integer INPUT_SHIFT          = INTERNAL_PHASE_WIDTH - DIN_WIDTH;
localparam integer MAG_SHIFT            = 2*DIN_WIDTH - MAG_WIDTH - 1;

localparam signed [47:0] PI = 48'sh8000_0000_0000;

// A slightly conservative approximation of 1 / CORDIC vector gain. Keeping
// a small margin prevents the compensated magnitude from overflowing at +1.
localparam signed [DIN_WIDTH-1:0] CORDIC_GAIN =
    48'h4DB8_BAC7_10CB >> (INTERNAL_PHASE_WIDTH-DIN_WIDTH);

localparam signed [MAG_WIDTH-1:0] MAG_MAX =
    {1'b0, {(MAG_WIDTH-1){1'b1}}};

// Sign-extend before shifting. This maps every input to an internal format
// with 47 fractional bits and enough integer guard bits for CORDIC gain.
wire signed [XY_WIDTH-1:0] x_scaled =
    $signed({{(XY_WIDTH-DIN_WIDTH){i_x[DIN_WIDTH-1]}}, i_x}) <<< INPUT_SHIFT;
wire signed [XY_WIDTH-1:0] y_scaled =
    $signed({{(XY_WIDTH-DIN_WIDTH){i_y[DIN_WIDTH-1]}}, i_y}) <<< INPUT_SHIFT;

reg signed [XY_WIDTH-1:0]             x_reg    [0:ITERATIONS];
reg signed [XY_WIDTH-1:0]             y_reg    [0:ITERATIONS];
reg signed [INTERNAL_PHASE_WIDTH-1:0] z_reg    [0:ITERATIONS];
reg                                      zero_reg [0:ITERATIONS];

// atan(2^-i) / pi * 2^47
wire signed [INTERNAL_PHASE_WIDTH-1:0] atan_table [0:ITERATIONS-1];
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

// Rotate vectors in the left half-plane by pi before vectoring so that the
// CORDIC core always starts inside its convergence range.
always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        x_reg[0]    <= {XY_WIDTH{1'b0}};
        y_reg[0]    <= {XY_WIDTH{1'b0}};
        z_reg[0]    <= 48'sd0;
        zero_reg[0] <= 1'b1;
    end
    else begin
        zero_reg[0] <= (i_x == 0) && (i_y == 0);
        if(i_x < 0) begin
            x_reg[0] <= -x_scaled;
            y_reg[0] <= -y_scaled;
            z_reg[0] <= PI;
        end
        else begin
            x_reg[0] <= x_scaled;
            y_reg[0] <= y_scaled;
            z_reg[0] <= 48'sd0;
        end
    end
end

integer i;
always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        for(i = 0; i < ITERATIONS; i = i + 1) begin
            x_reg[i+1]    <= {XY_WIDTH{1'b0}};
            y_reg[i+1]    <= {XY_WIDTH{1'b0}};
            z_reg[i+1]    <= 48'sd0;
            zero_reg[i+1] <= 1'b1;
        end
    end
    else begin
        for(i = 0; i < ITERATIONS; i = i + 1) begin
            zero_reg[i+1] <= zero_reg[i];
            if(y_reg[i] >= 0) begin
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

// Convert the CORDIC-gained magnitude back to the input numeric scale before
// multiplying by 1/K. For DIN_WIDTH=16 this multiplier is 18 x 16 bits.
wire signed [DIN_WIDTH+1:0] mag_with_gain =
    x_reg[ITERATIONS] >>> INPUT_SHIFT;
reg signed [2*DIN_WIDTH+1:0] mag_comp_mult;
reg signed [INTERNAL_PHASE_WIDTH-1:0] phase_delay;
reg zero_delay;

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        mag_comp_mult <= {(2*DIN_WIDTH+2){1'b0}};
        phase_delay   <= 48'sd0;
        zero_delay    <= 1'b1;
    end
    else begin
        mag_comp_mult <= $signed(mag_with_gain) * $signed(CORDIC_GAIN);
        phase_delay   <= z_reg[ITERATIONS];
        zero_delay    <= zero_reg[ITERATIONS];
    end
end

// The compensated product has 2*(DIN_WIDTH-1) fractional-scale bits relative
// to a normalized input. Shift directly into the requested magnitude format.
wire signed [2*DIN_WIDTH+1:0] mag_scaled = mag_comp_mult >>> MAG_SHIFT;

// Registered, aligned phase and magnitude outputs. Magnitude is nonnegative;
// values outside the requested signed output range saturate at its maximum.
always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        o_phase     <= {PHASE_WIDTH{1'b0}};
        o_phase_mag <= {MAG_WIDTH{1'b0}};
    end
    else if(zero_delay) begin
        o_phase     <= {PHASE_WIDTH{1'b0}};
        o_phase_mag <= {MAG_WIDTH{1'b0}};
    end
    else begin
        o_phase <= phase_delay[INTERNAL_PHASE_WIDTH-1 -: PHASE_WIDTH];

        if(mag_scaled < 0)
            o_phase_mag <= {MAG_WIDTH{1'b0}};
        else if(mag_scaled > $signed(MAG_MAX))
            o_phase_mag <= MAG_MAX;
        else
            o_phase_mag <= mag_scaled[MAG_WIDTH-1:0];
    end
end

endmodule
