// =============================================================================
// module   : NCO
// author   : binbin
// function : 数字锁相环（DPLL）的数控振荡器（Numerically Controlled Oscillator）。
//            根据中心频率控制字与环路滤波器修正量累加相位，查正弦 ROM 表
//            产生正弦波输出。
//
// 实现思路 :
//   - 相位累加 : 每周期相位累加器 phase_acc 累加 step = i_FREQ_INIT + freq_ctrl_ext，
//              即中心频率 + 环路滤波器修正量。
//   - 符号扩展 : 有符号频率修正控制字 i_FREQ_CTRL 符号扩展至 PWIDTH 位后与
//              中心频率相加，得到有效相位步进 step。
//   - 查表输出 : 取相位累加器高 AWIDTH 位作为地址，查 SINROM_4096_16 正弦 ROM。
//   - 有符号输出 : o_nco_out_signed 通过翻转 MSB，将无符号 ROM 输出转为
//              二进制补码有符号数。
//
// 参数说明 :
//   DWIDTH = 16 : 输出数据位宽
//   PWIDTH = 48 : 相位累加器位宽
//   AWIDTH = 12 : ROM 地址位宽（查表深度 2^12 = 4096）
//   CWIDTH = 40 : 环路滤波器控制字位宽（有符号，需 <= PWIDTH）
//
// 端口说明 :
//   i_clk, i_rst_n   : 时钟与异步复位（低有效）
//   i_FREQ_INIT      : 初始/中心振荡频率控制字（无符号）
//   i_FREQ_CTRL      : 环路滤波器输出的频率修正控制字（有符号，二进制补码）
//   o_nco_out        : 正弦波输出（无符号）
//   o_nco_out_signed : 正弦波输出（有符号，二进制补码）
//
// =============================================================================

`timescale 1ns / 1ps

module NCO#
(
    parameter                           DWIDTH = 16                ,
    parameter                           PWIDTH = 48                ,
    parameter                           AWIDTH = 12                ,
    parameter                           CWIDTH = 40                // 环路滤波器控制字位宽（有符号，需 <= PWIDTH）
)
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,

    input              [PWIDTH-1:0]     i_FREQ_INIT                ,   // 初始/中心振荡频率控制字（无符号）
    input              [CWIDTH-1:0]     i_FREQ_CTRL                ,   // 环路滤波器输出的频率修正控制字（有符号，二进制补码）

    output reg         [DWIDTH-1:0]     o_nco_out                  ,
    output reg         [DWIDTH-1:0]     o_nco_out_signed
);

// 将有符号控制字符号扩展至相位累加器位宽
wire [PWIDTH-1:0] freq_ctrl_ext = {{(PWIDTH-CWIDTH){i_FREQ_CTRL[CWIDTH-1]}}, i_FREQ_CTRL};
// 有效相位步进 = 中心频率 + 环路滤波器修正量
wire [PWIDTH-1:0] step = i_FREQ_INIT + freq_ctrl_ext;

wire [AWIDTH-1:0] addr;
reg  [PWIDTH-1:0] phase_acc = 0;
assign addr = phase_acc[PWIDTH-1:PWIDTH-AWIDTH];

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        phase_acc <= 0;
    end
    else begin
        phase_acc <= phase_acc + step;
    end
end

wire [DWIDTH-1:0] douta;
SINROM_4096_16 sinrom
(
    .clka(i_clk),
    .addra(addr),
    .douta(douta)
);

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        o_nco_out        <= 0;
        o_nco_out_signed <= 0;
    end
    else begin
        o_nco_out        <= douta;
        o_nco_out_signed <= {~douta[DWIDTH-1], douta[DWIDTH-2:0]};
    end
end

endmodule
