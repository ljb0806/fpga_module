// =============================================================================
// module   : PD
// author   : binbin
// function : 数字锁相环（DPLL）的鉴相器（Phase Detector）。
//            对两路输入信号做乘法鉴相，并经低通 FIR 滤波提取相位误差 o_phase，
//            供后续环路滤波器（PI_Filter）使用。
//
// 实现思路 :
//   - 鉴相方式 : 乘法型鉴相器（Mixer PD）。两路同频信号相乘，产生和频与
//              差频（含直流/相位差）分量。
//   - 低通滤波 : 乘法结果高位（mul_res[2*DIN_WIDTH-1:DIN_WIDTH]）送入
//              FIR 低通滤波器 IP（fir_compiler_0），滤除和频分量，
//              保留反映相位差的低频分量，得到 o_phase。
//   - 定点格式 : 输入 DIN_WIDTH 位有符号数，乘法结果 2*DIN_WIDTH-1 位，
//              相位误差输出 DOUT_WIDTH 位（由 FIR 滤波器配置决定）。
//
// 参数说明 :
//   DIN_WIDTH  = 16 : 输入数据位宽
//   DOUT_WIDTH = 40 : 相位误差输出位宽（需与 FIR 低通滤波器配置一致）
//
// 端口说明 :
//   i_clk, i_rst_n      : 时钟与异步复位（低有效）
//   i_data_A, i_data_B  : 两路待鉴相输入信号（DIN_WIDTH 位）
//   o_phase             : 鉴相输出（DOUT_WIDTH 位，低通滤波后的相位误差）
//
// =============================================================================

`timescale 1ns / 1ps

module PD#
(
    parameter                           DIN_WIDTH = 16             ,
    parameter                           DOUT_WIDTH = 40             
)
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,

    input              [DIN_WIDTH-1:0]  i_data_A                   ,
    input              [DIN_WIDTH-1:0]  i_data_B                   ,
    
    output             [DOUT_WIDTH-1:0] o_phase                     
);

reg signed [2*DIN_WIDTH-1:0] mul_res;
always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        mul_res <= 0;
    end
    else begin
        mul_res <= $signed(i_data_A) * $signed(i_data_B);
    end
end

// 这个变量的位宽根据 FIR LP Filter 的配置进行宏定义修改
fir_compiler_0 fir_compiler_0_inst
(
    .aclk(i_clk),
    .s_axis_data_tvalid(1'b1),
    .s_axis_data_tready(),
    .s_axis_data_tdata(mul_res[2*DIN_WIDTH-1:DIN_WIDTH]),
    .m_axis_data_tvalid(),
    .m_axis_data_tdata(o_phase)
);

endmodule
