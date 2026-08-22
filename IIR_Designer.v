// =============================================================================
// 模块名称 : IIR_Designer
// 文件名称 : IIR_Designer.v
// 作者     : binbin
// 功能描述 : 外部系数驱动的固定系数直接型 IIR 滤波器。前馈使用输入延迟样本，
//            反馈使用自身输出延迟样本，可配合 iir_designer.m 生成量化系数。
//
// 实现思路 :
//   1. 使用输入延迟线保存 x(n-k)，使用输出延迟线保存 y(n-k)。
//   2. 按以下直接型结构完成前馈和反馈乘加：
//        y(n)=Σa_k*x(n-k)+Σb_k*y(n-k)
//   3. 将扩展累加结果算术右移 FRACBITS，恢复到数据定点格式。
//   4. 对移位结果做上下限判断，再输出 DWIDTH 位饱和结果。
//
// 参数说明 :
//   FEEDFORWARD_TAPS = 3  : 前馈抽头数；当前固定端口映射要求保持为 3。
//   FEEDBACK_TAPS    = 2  : 反馈抽头数；当前固定端口映射要求保持为 2。
//   DWIDTH           = 14 : 输入和输出数据位宽。
//   CWIDTH           = 32 : 系数位宽。
//   FRACBITS         = 28 : 系数小数位数。
//
// 端口说明 :
//   i_clk            : 工作时钟，上升沿采样。
//   i_rst_n          : 异步复位，低有效；复位时清空输入、输出延迟线。
//   i_xn             : DWIDTH 位有符号滤波输入。
//   o_yn             : DWIDTH 位有符号饱和滤波输出。
//   i_a0～i_a2       : 32 位有符号前馈系数输入。
//   i_b0～i_b1       : 32 位有符号反馈系数输入；符号约定为已取反的分母系数。
//
// 定点与时序 :
//   - 默认数据为 Q1.13，系数为 Q3.28，内部累加位宽按总抽头数扩展。
//   - 输出为组合乘加、移位和饱和结果，延迟线在每个时钟上升沿更新。
//   - 反馈系数必须经过稳定性和量化误差检查，防止量化后极点越出单位圆。
//
// 使用注意 :
//   当前饱和表达式中的 1'i_b0、1'i_b1 不是合法 Verilog 数字字面量，综合前
//   应分别修正为 1'b0、1'b1；本次仅统一文件头，不修改原有 RTL 逻辑。
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
