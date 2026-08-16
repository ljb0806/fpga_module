// =============================================================================
// module   : LFSR_Generator
// author   : binbin
// function : 基于 32 位线性反馈移位寄存器（LFSR）的伪随机白噪声发生器。
//            每个时钟周期产生 1 比特的随机判决，并将其复制到整个输出总线。
//
// 实现思路 :
//   - 本原多项式  : x^32 + x^22 + x^2 + x + 1（最大长度序列）
//   - LFSR 位宽   : 32 位
//   - 序列周期    : 2^32 - 1（约 43 亿个时钟周期）
//   - 初始种子    : 32'hA3C5_1F2E（非零值，防止 LFSR 锁定在全零状态）
//   - 抽头位置    : 32, 22, 2, 1，四个抽头异或产生反馈位
//   - 移位方式    : 每个时钟周期，寄存器整体向左移一位，
//                  最高位（MSB）丢弃，反馈位填充到最低位（LSB）。
//   - 输出格式    : {DOUT_WIDTH{lfsr_reg[0]}}，即取 LFSR 的 bit 0 复制
//                  DOUT_WIDTH 次（默认 14 位）。每个周期输出为全 0 或全 1，
//                  适用于抖动注入、扩频时钟或噪声激励等场景。
//
// 端口说明 :
//   i_clk, i_rst_n : 时钟与异步复位（低有效）
//   o_noise_out    : 伪随机噪声输出（DOUT_WIDTH 位，全 0 或全 1）
//
// =============================================================================

module LFSR_Generator#
(
    parameter                           DOUT_WIDTH = 14             
)
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,
    output             [DOUT_WIDTH-1:0] o_noise_out                 
);

localparam TAP_1 = 32;
localparam TAP_2 = 22;
localparam TAP_3 = 2;
localparam TAP_4 = 1;

reg [31:0] lfsr_reg = 32'hA3C5_1F2E;

wire feedback_bit;
assign feedback_bit = lfsr_reg[TAP_1-1] ^ lfsr_reg[TAP_2-1] ^
                      lfsr_reg[TAP_3-1] ^ lfsr_reg[TAP_4-1];

always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        lfsr_reg <= 32'hA3C5_1F2E;
    end else begin
        lfsr_reg <= {lfsr_reg[30:0], feedback_bit};
    end
end

assign o_noise_out = {DOUT_WIDTH{lfsr_reg[0]}};

endmodule
