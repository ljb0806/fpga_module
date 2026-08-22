// =============================================================================
// 模块名称 : Cordic_Atan2
// 文件名称 : Cordic_Atan2_24.v
// 作者     : binbin
// 功能描述 : 24 位 CORDIC 反正切与模长计算器。输入直角坐标 (x,y)，持续
//            流水输出 atan2(y,x) 和 sqrt(x^2+y^2)。
//
// 实现思路 :
//   1. 输入先乘以约 0.6072 的 CORDIC_GAIN，预补偿向量模式的固有增益。
//   2. 当 x<0 时把向量旋转 180°，映射到向量模式的收敛半平面。
//   3. 进行 24 次向量模式微旋转，每级根据 y 的符号选择旋转方向：
//        x(i+1) = x(i) - d(i) * (y(i) >>> i)
//        y(i+1) = y(i) + d(i) * (x(i) >>> i)
//        z(i+1) = z(i) - d(i) * atan(2^-i)
//   4. 迭代结束后的 z 为相位，x 为补偿后的向量模长，模长输出带饱和保护。
//
// 参数说明 :
//   无可配置模块参数，输入、相位和模长数据通路固定为 24 位。
//
// 端口说明 :
//   i_clk       : 工作时钟，上升沿采样。
//   i_rst_n     : 异步复位，低有效。
//   i_x, i_y    : 24 位有符号直角坐标输入，默认按 Q1.23 解释。
//   o_phase     : 24 位有符号圆周相位，表示 atan2(i_y,i_x)。
//   o_phase_mag : 24 位有符号端口承载的非负模长，满量程时饱和。
//
// 定点与时序 :
//   - 相位采用 24 位圆周编码；π 对应 24'h800000，π/2 对应 24'h400000。
//   - 数据默认采用 Q1.23 格式；内部 x/y 扩展为 25 位以降低溢出风险。
//   - 输入预补偿与象限预处理后接 24 级迭代，可每拍接收一个输入。
//
// 使用注意 :
//   - (0,0) 的数学相位未定义，使用者不应依赖该输入下的 o_phase。
//   - 本文件与 Cordic_Atan2_48.v 模块名相同，应二选一或重命名。
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
