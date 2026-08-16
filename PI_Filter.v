// =============================================================================
// module   : PI_Filter
// author   : binbin
// function : 数字锁相环（DPLL）的环路滤波器，采用比例-积分（PI）结构。
//            将鉴相器输出的相位误差 i_err 转换为频率控制字 o_ctrl，驱动 NCO。
//
// 实现思路 :
//   - 结构 : PI 控制器。比例支路 P = err_ext <<< KP，积分支路 I = err_ext >>> KI。
//   - 位宽扩展 : 将 EWIDTH 位误差符号扩展至 UWIDTH 位，避免运算溢出。
//   - 积分饱和 : 积分累加结果 nxt_integ 限幅在 [-LIMIT, +LIMIT]，防止积分器
//              溢出或缠绕。
//   - 输出 : o_ctrl = P + integ，比例支路取当前误差，积分支路取上一周期累积值。
//
// 参数说明 :
//   EWIDTH = 40 : 误差输入位宽
//   UWIDTH = 40 : 控制字输出位宽（内部运算位宽）
//   KP     = 3  : 比例增益，左移 <<< KP（3 = x8）
//   KI     = 9  : 积分增益，右移 >>> KI（衰减 1/2^KI）
//   LIMIT  = 40'sh40_0000_0000 : 积分饱和上限（有符号，= 2^38）
//
// 端口说明 :
//   i_clk, i_rst_n : 时钟与异步复位（低有效）
//   i_err          : 相位误差输入（EWIDTH 位有符号）
//   o_ctrl         : 环路控制字输出（UWIDTH 位有符号）
//
// =============================================================================

`timescale 1ns / 1ps

module PI_Filter#
(
    parameter                           EWIDTH = 40                ,
    parameter                           UWIDTH = 40                ,
    parameter                           KP = 3                     ,   // 比例增益: 左移 <<< KP, 3 = x8
    parameter                           KI = 9                     ,   // 积分增益: 右移 >>> KI, 8 = /256
    parameter signed [39:0]             LIMIT = 40'sh40_0000_0000      // 饱和上限, 有符号 = 2^38
)
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,

    input       signed [EWIDTH-1:0]     i_err                      ,
    output reg  signed [UWIDTH-1:0]     o_ctrl
);

wire signed [UWIDTH-1:0] err_ext = {{(UWIDTH-EWIDTH){i_err[EWIDTH-1]}}, i_err};
wire signed [UWIDTH-1:0] P       = err_ext <<< KP;      // 比例支路
wire signed [UWIDTH-1:0] I       = err_ext >>> KI;      // 积分支路(右移衰减)

reg  signed [UWIDTH-1:0] integ;
wire signed [UWIDTH-1:0] nxt_integ = integ + I;

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        integ  <= 0;
        o_ctrl <= 0;
    end
    else begin
        o_ctrl <= P + integ;                 // 输出 = 比例(当前) + 积分(上一周期)
        if(nxt_integ > LIMIT)
            integ <= LIMIT;
        else if(nxt_integ < -LIMIT)
            integ <= -LIMIT;
        else
            integ <= nxt_integ;
    end
end

endmodule
