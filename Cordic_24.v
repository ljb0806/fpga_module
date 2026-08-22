// =============================================================================
// 模块名称 : Cordic
// 文件名称 : Cordic_24.v
// 作者     : binbin
// 功能描述 : 24 位 CORDIC 正弦/余弦计算器。输入全圆周相位，持续流水输出
//            对应的 cos 和 sin，不使用 ROM 或乘法器。
//
// 实现思路 :
//   1. 先按输入象限把相位映射到 CORDIC 收敛区间 [-π/2, π/2]。
//   2. 以 ±CORDIC_GAIN 作为 x 初值，预先补偿 CORDIC 固有增益。
//   3. 进行 24 次旋转模式微旋转，每级根据 z 的符号选择旋转方向：
//        x(i+1) = x(i) - d(i) * (y(i) >>> i)
//        y(i+1) = y(i) + d(i) * (x(i) >>> i)
//        z(i+1) = z(i) - d(i) * atan(2^-i)
//   4. 迭代结束后，x、y 分别作为余弦和正弦输出。
//
// 参数说明 :
//   无可配置模块参数，数据通路固定为 24 位。
//
// 端口说明 :
//   i_clk    : 工作时钟，上升沿采样。
//   i_rst_n  : 异步复位，低有效。
//   i_phase  : 24 位有符号圆周相位输入。
//   o_cos    : 24 位有符号余弦输出。
//   o_sin    : 24 位有符号正弦输出。
//
// 定点与时序 :
//   - 相位采用 24 位圆周编码；π 对应 24'h800000，π/2 对应 24'h400000。
//   - o_cos/o_sin 采用 Q1.23 格式，数值范围约为 [-1, 1)。
//   - 象限预处理后接 24 级迭代，可每拍接收一个输入；使用时需对齐流水延迟。
//
// 使用注意 :
//   本文件与 Cordic_48.v 的模块名相同，同一编译库中应二选一或重命名。
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
