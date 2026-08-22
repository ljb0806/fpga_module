// =============================================================================
// 模块名称 : LFSR_Generator
// 文件名称 : LFSR_Generator.v
// 作者     : binbin
// 功能描述 : 基于 32 位线性反馈移位寄存器的伪随机序列发生器。每个时钟产生
//            一个伪随机比特，并将该比特复制到整个输出总线。
//
// 实现思路 :
//   1. 使用非零种子 32'hA3C5_1F2E 初始化 32 位 LFSR，避免全零锁死状态。
//   2. 按多项式 x^32+x^22+x^2+x+1 对第 32、22、2、1 位做异或反馈。
//   3. 每拍把寄存器左移一位，并把反馈结果填入最低位。
//   4. 取 lfsr_reg[0]，复制 DOUT_WIDTH 次后输出全 0 或全 1 码字。
//
// 参数说明 :
//   DOUT_WIDTH = 14 : 输出总线位宽，不改变内部 32 位 LFSR 的长度和周期。
//
// 端口说明 :
//   i_clk       : 工作时钟，上升沿更新序列。
//   i_rst_n     : 异步复位，低有效；复位时重新装载固定非零种子。
//   o_noise_out : DOUT_WIDTH 位伪随机输出，每拍为全 0 或全 1。
//
// 定点与时序 :
//   - 理论序列周期为 2^32-1，每个时钟输出一个新的序列比特。
//   - 输出总线各位完全相同，不是 DOUT_WIDTH 路相互独立的随机比特。
//
// 使用注意 :
//   该输出适合一比特激励、抖动或扩频控制；若作为有符号多位数据解释，
//   全 1 码字为 -1 而不是负满量程。多位白噪声需要另做取位或幅度映射。
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
