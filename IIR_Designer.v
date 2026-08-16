// =============================================================================
// 模块名称 : IIR_Designer
// 作者     : binbin
// 功能描述 : 固定系数直接型 IIR 滤波器，根据外部输入的系数对输入信号进行滤波。
//            可配合 IIR_Designer.m（MATLAB）使用，由 MATLAB 设计IIR滤波器系数，
//            导入到本模块进行滤波。
//
// 实现思路 :
//   - 滤波器结构 : 直接型 IIR
//       y(n) = Σ a_k * x(n-k) + Σ b_k * y(n-k)
//       前馈支路 : i_xn → 延迟线 (i_xn_reg) → i_a0,i_a1,i_a2 加权求和
//       反馈支路 : o_yn → 延迟线 (o_yn_reg) → i_b0,i_b1 加权求和（反馈取自自身输出）
//   - 定点格式 :
//       数据 (i_xn,o_yn) : Q1.13 有符号数（DWIDTH=14）
//       系数 (a,b)   : Q3.28 有符号数（CWIDTH=32, FRACBITS=28）
//       内部累加器   : FWIDTH 位宽，包含防溢出扩展位
//   - 输出限幅 : o_yn 限制在 [-8192, 8191]（14-bit 有符号满量程），防止定点溢出
//
// 参数说明 :
//   FEEDFORWARD_TAPS = 3 : 前馈抽头数（i_a0, i_a1, i_a2）
//   FEEDBACK_TAPS   = 2 : 反馈抽头数（i_b0, i_b1）
//   DWIDTH          = 14: 数据位宽
//   CWIDTH          = 32: 系数位宽
//   FRACBITS        = 28: 系数小数位数
//
// 端口说明 :
//   i_clk, i_rst_n : 时钟与异步复位（低有效）
//   i_xn         : 滤波器输入信号
//   o_yn         : 滤波器输出信号（限幅后）
//   i_a0,i_a1,i_a2   : 前馈系数输入
//   i_b0,i_b1      : 反馈系数输入
//
// =============================================================================

module IIR_Designer #
(
    parameter                           FEEDFORWARD_TAPS = 3       ,
    parameter                           FEEDBACK_TAPS   = 2        ,
    parameter                           DWIDTH  = 14               ,
    parameter                           CWIDTH  = 32               ,
    parameter                           FRACBITS = 28               
)
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,

    input       signed [DWIDTH-1:0]     i_xn                       ,
    output      signed [DWIDTH-1:0]     o_yn                       ,

    input       signed [  31:0]         i_a0                       ,
    input       signed [  31:0]         i_a1                       ,
    input       signed [  31:0]         i_a2                       ,

    input       signed [  31:0]         i_b0                       ,
    input       signed [  31:0]         i_b1                        
);

    localparam TOTAL_TAPS = FEEDFORWARD_TAPS + FEEDBACK_TAPS;
    localparam FWIDTH = DWIDTH + CWIDTH + $clog2(TOTAL_TAPS) + 1;
    wire signed [CWIDTH-1:0] A_COEFF [0:FEEDFORWARD_TAPS-1];
    wire signed [CWIDTH-1:0] B_COEFF [0:FEEDBACK_TAPS-1];

    assign A_COEFF[0] = i_a0;  
    assign A_COEFF[1] = i_a1;  
    assign A_COEFF[2] = i_a2;  

    assign B_COEFF[0] = i_b0;  
    assign B_COEFF[1] = i_b1;  


    reg signed [DWIDTH-1:0] i_xn_reg [0:FEEDFORWARD_TAPS-1];
    reg signed [DWIDTH-1:0] o_yn_reg [0:FEEDBACK_TAPS-1];
    integer i;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            for (i = 0; i < FEEDFORWARD_TAPS; i = i + 1)
                i_xn_reg[i] <= 0;
            for (i = 0; i < FEEDBACK_TAPS; i = i + 1)
                o_yn_reg[i] <= 0;
        end else begin
            i_xn_reg[0] <= i_xn;
            for (i = 1; i < FEEDFORWARD_TAPS; i = i + 1)
                i_xn_reg[i] <= i_xn_reg[i-1];

            o_yn_reg[0] <= o_yn;
            for (i = 1; i < FEEDBACK_TAPS; i = i + 1)
                o_yn_reg[i] <= o_yn_reg[i-1];
        end
    end


    reg signed [FWIDTH-1:0] y_full;
    integer k;

    always @(*) begin
        y_full = 0;

        for (k = 0; k < FEEDFORWARD_TAPS; k = k + 1)
            y_full = y_full + A_COEFF[k] * i_xn_reg[k];

        for (k = 0; k < FEEDBACK_TAPS; k = k + 1)
            y_full = y_full + B_COEFF[k] * o_yn_reg[k];
    end


    localparam signed [FWIDTH-1:0] o_yn_MAX =  (2**(DWIDTH-1)) - 1;
    localparam signed [FWIDTH-1:0] o_yn_MIN = -(2**(DWIDTH-1));

    wire signed [FWIDTH-1:0] o_yn_shifted;
    assign o_yn_shifted = y_full >>> FRACBITS;

    assign o_yn = (o_yn_shifted > o_yn_MAX) ?  {1'i_b0, {(DWIDTH-1){1'i_b1}}} :
                (o_yn_shifted < o_yn_MIN) ?  {1'i_b1, {(DWIDTH-1){1'i_b0}}} :
                                         o_yn_shifted[DWIDTH-1:0];

endmodule
