// =============================================================================
// module   : Cordic_Atan2
// author   : binbin
// function : 基于 CORDIC 向量模式（Vectoring Mode）的反正切/模长计算器。
//            输入向量坐标 (i_x, i_y)（Q23 定点），输出 atan2(y, x) 相位角
//            和向量模长 sqrt(x^2 + y^2)（Q23 定点，unity gain）。
//
// 与 Cordic.v（旋转模式）对比 :
//   - 旋转模式 : 已知角度 → 旋转 (x,y), 驱动 z→0 → 输出 cos/sin
//   - 向量模式 : 已知 (x,y) → 旋转以驱动 y→0 → 输出 atan2 和模长
//   核心迭代公式相同，仅方向判决条件不同:
//     旋转模式 : d_i = sign(z_i)
//     向量模式 : d_i = -sign(y_i)
//
// 实现思路 :
//   - 算法原理 : 通过 24 次微旋转将 y 逼近 0，累积的旋转角度即为 atan2
//       x_{i+1} = x_i - d_i * (y_i >> i)
//       y_{i+1} = y_i + d_i * (x_i >> i)
//       z_{i+1} = z_i - d_i * atan(2^{-i})
//       其中 d_i = -sign(y_i): y_i >= 0 时 d_i=-1, y_i < 0 时 d_i=+1
//   - 增益预补偿 : CORDIC 迭代会使向量模长放大 ≈ 1.647 倍
//       为避免内部寄存器溢出，在输入级乘以 CORDIC_GAIN (≈0.6072) 预补偿，
//       使最终输出模长为 unity gain，且相位不受影响（atan2 比例不变）
//   - 象限预处理 : CORDIC 向量模式收敛范围为 [-π/2, π/2] (即 x >= 0)
//       当 x < 0 时，将向量旋转 180° 映射到 x>=0 半平面:
//         i_xit = -x_comp, i_yit = -y_comp
//         z_init = (i_y >= 0) ? +PI : -PI
//       当 x >= 0 时直接输入，z_init = 0
//   - 定点格式 : Q23（24-bit 有符号），π = 0x800000 = 2^23
//   - 流水线   : 24 级流水线 + 1 级象限预处理，延迟 25 个 i_clk
//
// 端口说明 :
//   i_clk, i_rst_n      : 时钟与异步复位（低有效）
//   i_x, i_y      : 输入向量坐标（Q23）
//   o_phase       : atan2(i_y, i_x) 相位角输出（Q23, 范围 [-π, π]）
//   o_phase_mag   : 向量模长输出（Q23, unity gain）
//
// 典型用途 :
//   - 与 Cordic.v 配合: Cordic 发 cos/sin → Atan2 收回来鉴相，验证全链路
//   - FM 解调中的基带鉴相 (atan2)
//   - 极坐标转换
//
// 复位行为 :
//   - i_rst_n = 0（低有效）: 所有流水线寄存器清零
//   - i_rst_n = 1（正常工作）: 持续流水输出，延迟 25 个周期后输出有效结果
// =============================================================================

module Cordic_Atan2
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,

    input       signed [  23:0]         i_x                        ,
    input       signed [  23:0]         i_y                        ,
    output      signed [  23:0]         o_phase                    ,
    output      signed [  23:0]         o_phase_mag                 
);

localparam signed [23:0] CORDIC_GAIN = 24'd5093751;   // 0.6072 * 2^23
localparam signed [23:0] PI          = 24'h800000;
localparam signed [23:0] HALF_PI     = 24'sd4194304;

// 内部寄存器: x/y 用 25-bit 防溢出, z 用 24-bit (相位不会放大)
reg signed [24:0] x_reg [0:24];
reg signed [24:0] y_reg [0:24];
reg signed [23:0] z_reg [0:24];

// atan(2^-i) 查找表 (Q23 定点)
wire signed [23:0] atan_table [0:23];
assign atan_table[0]  = 24'd2097152;  assign atan_table[1]  = 24'd1238053;
assign atan_table[2]  = 24'd654271;   assign atan_table[3]  = 24'd332205;
assign atan_table[4]  = 24'd166687;   assign atan_table[5]  = 24'd83416;
assign atan_table[6]  = 24'd41716;    assign atan_table[7]  = 24'd20860;
assign atan_table[8]  = 24'd10430;    assign atan_table[9]  = 24'd5215;
assign atan_table[10] = 24'd2608;     assign atan_table[11] = 24'd1304;
assign atan_table[12] = 24'd652;      assign atan_table[13] = 24'd326;
assign atan_table[14] = 24'd163;      assign atan_table[15] = 24'd81;
assign atan_table[16] = 24'd41;       assign atan_table[17] = 24'd20;
assign atan_table[18] = 24'd10;       assign atan_table[19] = 24'd5;
assign atan_table[20] = 24'd3;        assign atan_table[21] = 24'd1;
assign atan_table[22] = 24'd1;        assign atan_table[23] = 24'd0;

// ---- 增益预补偿 + 象限预处理 ----
// 预补偿: 确保 CORDIC 迭代后模长 = sqrt(x^2 + y^2), 不会放大
wire signed [47:0] x_comp_full = i_x * CORDIC_GAIN;
wire signed [47:0] y_comp_full = i_y * CORDIC_GAIN;
wire signed [24:0] x_comp = x_comp_full >>> 23;
wire signed [24:0] y_comp = y_comp_full >>> 23;

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        x_reg[0] <= 0;
        y_reg[0] <= 0;
        z_reg[0] <= 0;
    end
    else begin
        if(i_x >= 0) begin
            x_reg[0] <= x_comp;
            y_reg[0] <= y_comp;
            z_reg[0] <= 0;
        end
        else if(i_y >= 0) begin
            x_reg[0] <= -x_comp;
            y_reg[0] <= -y_comp;
            z_reg[0] <= PI;
        end
        else begin
            x_reg[0] <= -x_comp;
            y_reg[0] <= -y_comp;
            z_reg[0] <= -PI;
        end
    end
end

// ---- 24 级流水线微旋转: 驱动 y → 0, 累积角度 z ----
integer i;

always @(posedge i_clk or negedge i_rst_n) begin
    for(i = 0; i < 24; i = i + 1) begin
        if(!i_rst_n) begin
            x_reg[i+1] <= 0;
            y_reg[i+1] <= 0;
            z_reg[i+1] <= 0;
        end
        else begin
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

// ---- 输出 (x_reg 截回 24-bit, 无溢出时模长 ≤ 2^23) ----
wire signed [24:0] mag_25 = x_reg[24];

assign o_phase     = z_reg[24];
assign o_phase_mag = (mag_25 > 24'sd8388607)  ? 24'sd8388607  :
                       (mag_25 < -24'sd8388608) ? -24'sd8388608 :
                       mag_25[23:0];

endmodule
