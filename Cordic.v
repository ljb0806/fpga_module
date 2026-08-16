// =============================================================================
// module   : Cordic
// author   : binbin
// function : 基于 CORDIC（坐标旋转数字计算）算法的正弦/余弦计算器。
//            输入任意相位角（Q23 定点），输出对应的 cos 和 sin 值。
//
// 实现思路 :
//   - 算法原理 : CORDIC 旋转模式，通过 24 次微旋转将角度 z 逼近 0，
//              旋转结束后 x = cos(θ), y = sin(θ)
//       x_{i+1} = x_i - d_i * (y_i >> i)
//       y_{i+1} = y_i + d_i * (x_i >> i)
//       z_{i+1} = z_i - d_i * atan(2^{-i})
//       其中 d_i = sign(z_i): z_i >= 0 时 d_i=+1, z_i < 0 时 d_i=-1
//   - 象限预处理 : 将输入相位映射到 [-π/2, π/2]
//       第二象限 (phase >  π/2) → phase - π，x 初值取负
//       第三象限 (phase < -π/2) → phase + π，x 初值取负
//       第一、四象限不变
//   - CORDIC 增益补偿 : 24 次旋转的总增益 ≈ 1/0.6072，通过初始值
//      x0 = ±CORDIC_GAIN 预补偿，避免后乘
//   - 定点格式 : Q23（24-bit 有符号），π = 0x800000 = 2^23
//   - 流水线   : 24 级流水线，每级一个时钟周期，延迟 24 + 1 = 25 个 i_clk
//
// 参数说明 :
//   输入 i_phase  : Q23 定点相位角，范围覆盖整个圆周（-π ~ π 映射到全 24-bit 范围）
//   输出 cos/sin   : Q23 定点余弦/正弦值，范围 [-1, 1) 映射到 [-2^23, 2^23)
//
// 端口说明 :
//   i_clk, i_rst_n     : 时钟与异步复位（低有效）
//   i_phase       : 输入相位角（Q23）
//   o_cos : 余弦输出（Q23）
//   o_sin : 正弦输出（Q23）
//
// =============================================================================


module Cordic
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,

    input       signed [  23:0]         i_phase                    ,
    output      signed [  23:0]         o_cos                      ,
    output      signed [  23:0]         o_sin                       
);

// 对于Q23格式定点数，pi 对应量程一半0x800000 = 2 ^ 23
// pi / 2 对应 2^23 / 2 = 4194304, 后面该参数需要被比较，因此声明为sd格式
// 同理 CORDIC_GAIN 为 2 的负数幂角的余弦积，该参数约为 0.6072，转化为定点数为 0.6072 * 2^23 = 5093751
localparam signed [23:0] CORDIC_GAIN = 24'd5093751;
localparam signed [23:0] PI          = 24'h800000;
localparam signed [23:0] HALF_PI     = 24'sd4194304;

reg signed [23:0] x_reg [0:24];
reg signed [23:0] y_reg [0:24];
reg signed [23:0] z_reg [0:24];

// 以下为 2 的负数幂角的反正切值，为 Q23 定点数 
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

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        x_reg[0] <= 0;
        y_reg[0] <= 0;
        z_reg[0] <= 0;
    end
    else begin
        // 第二象限对称到第一象限
        if(i_phase > HALF_PI) begin
            x_reg[0] <= -CORDIC_GAIN;
            y_reg[0] <= 0;
            z_reg[0] <= $signed(i_phase) - PI;
        end
        // 第三象限对称到第四象限
        else if(i_phase < -HALF_PI) begin
            x_reg[0] <= -CORDIC_GAIN;
            y_reg[0] <= 0;
            z_reg[0] <= $signed(i_phase) + PI;
        end
        // 第一、四象限不变
        else begin
            x_reg[0] <= CORDIC_GAIN;
            y_reg[0] <= 0;
            z_reg[0] <= $signed(i_phase);
        end
    end
end

// 通过旋转更新x、y、z寄存器的值，旋转24次，将z向着0逼近，最终x、y寄存器的值即为cos(i_phase)和sin(i_phase)
integer i;

always @(posedge i_clk or negedge i_rst_n) begin
    for(i = 0; i < 24; i = i + 1) begin
        if(!i_rst_n) begin
            x_reg[i+1] <= 0;
            y_reg[i+1] <= 0;
            z_reg[i+1] <= 0;
        end
        else begin
            // z_reg[i] < 0 时，旋转角度为正
            if(z_reg[i] < 0) begin
                x_reg[i+1] <= x_reg[i] + (y_reg[i] >>> i);
                y_reg[i+1] <= y_reg[i] - (x_reg[i] >>> i);
                z_reg[i+1] <= z_reg[i] + atan_table[i];
            end
            // z_reg[i] >= 0 时，旋转角度为负
            else begin
                x_reg[i+1] <= x_reg[i] - (y_reg[i] >>> i);
                y_reg[i+1] <= y_reg[i] + (x_reg[i] >>> i);
                z_reg[i+1] <= z_reg[i] - atan_table[i];
            end
        end
    end
end

assign o_cos = x_reg[24];
assign o_sin = y_reg[24];

endmodule
