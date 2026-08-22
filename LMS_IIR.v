// =============================================================================
// 模块名称 : LMS_IIR
// 文件名称 : LMS_IIR.v
// 作者     : binbin
// 功能描述 : 基于方程误差法的 LMS 自适应 IIR 系数辨识器。模块在线估计三路
//            前馈系数和两路反馈系数，并输出当前误差和系数供后级使用。
//
// 实现思路 :
//   1. 分别保存输入 x(n-k) 和期望信号 d(n-k) 的延迟样本。
//   2. 按 y(n)=Σa_k*x(n-k)+Σb_k*d(n-k) 计算方程误差模型输出。
//   3. 计算 e(n)=d(n)-y(n)，再用 e*x 和 e*d 更新前馈、反馈系数。
//   4. i_mu 作为算术右移指数控制有效步长，i_freeze 可冻结全部系数。
//   5. 输出结果右移 FRACBITS 后饱和到数据位宽，同时直接导出 a/b 系数。
//
// 参数说明 :
//   FEEDFORWARD_TAPS = 3  : 前馈抽头数；当前固定端口映射要求保持为 3。
//   FEEDBACK_TAPS    = 2  : 反馈抽头数；当前固定端口映射要求保持为 2。
//   DWIDTH           = 14 : 输入、期望和输出数据位宽。
//   CWIDTH           = 32 : 前馈、反馈系数位宽。
//   FRACBITS         = 28 : 系数小数位数。
//
// 端口说明 :
//   i_clk            : 工作时钟，上升沿采样。
//   i_rst_n          : 异步复位，低有效；复位时清空样本和系数。
//   i_freeze         : 系数冻结控制，高有效。
//   i_mu             : 8 位无符号步长指数。
//   i_xn             : DWIDTH 位有符号模型输入。
//   i_dn             : DWIDTH 位有符号期望信号，同时用于方程误差反馈支路。
//   o_yn             : DWIDTH 位有符号饱和模型输出。
//   o_en             : DWIDTH+1 位有符号误差，等于 i_dn-o_yn。
//   o_a0～o_a2       : CWIDTH 位有符号前馈系数输出。
//   o_b0～o_b1       : CWIDTH 位有符号反馈系数输出。
//
// 定点与时序 :
//   - 默认数据为 Q1.13，系数为 Q3.28；内部使用扩展累加器。
//   - 默认 14 位输出饱和到 [-8192,8191]。
//   - 系数更新和多抽头求和含组合乘加路径，目标频率较高时应评估流水化。
//
// 使用注意 :
//   - 本模块反馈使用 i_dn 而不是自身 o_yn，适合辨识系数，不等价于冻结后的
//     标准递归 IIR；收敛系数应交给 IIR_Designer 完成实际滤波。
//   - 参数组合必须保证 SHIFT_BASE+i_mu 是合法、合理的移位量。
// =============================================================================

module LMS_IIR #
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
    input                               i_freeze                   ,

    input              [   7:0]         i_mu                       ,

    input       signed [DWIDTH-1:0]     i_xn                       ,
    input       signed [DWIDTH-1:0]     i_dn                       ,
    output      signed [DWIDTH-1:0]     o_yn                       ,
    output      signed [DWIDTH:0]       o_en                       ,

    output      signed [CWIDTH-1:0]     o_a0                       ,
    output      signed [CWIDTH-1:0]     o_a1                       ,
    output      signed [CWIDTH-1:0]     o_a2                       ,

    output      signed [CWIDTH-1:0]     o_b0                       ,
    output      signed [CWIDTH-1:0]     o_b1                        
);

    localparam TOTAL_TAPS = FEEDFORWARD_TAPS + FEEDBACK_TAPS;
    localparam FWIDTH = DWIDTH + CWIDTH + $clog2(TOTAL_TAPS) + 1;
    localparam SHIFT_BASE = 2 * DWIDTH - 1 - FRACBITS;

    reg signed [DWIDTH-1:0] xn_reg [0:FEEDFORWARD_TAPS-1];
    reg signed [DWIDTH-1:0] dn_reg [0:FEEDBACK_TAPS-1];

    integer i;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            for (i = 0; i < FEEDFORWARD_TAPS; i = i + 1)
                xn_reg[i] <= 0;
            for (i = 0; i < FEEDBACK_TAPS; i = i + 1)
                dn_reg[i] <= 0;
        end else begin
            xn_reg[0] <= i_xn;
            for (i = 1; i < FEEDFORWARD_TAPS; i = i + 1)
                xn_reg[i] <= xn_reg[i-1];

            dn_reg[0] <= i_dn;
            for (i = 1; i < FEEDBACK_TAPS; i = i + 1)
                dn_reg[i] <= dn_reg[i-1];
        end
    end

    reg signed [CWIDTH-1:0] a_reg [0:FEEDFORWARD_TAPS-1];
    reg signed [CWIDTH-1:0] b_reg [0:FEEDBACK_TAPS-1];

    assign o_a0 = a_reg[0];
    assign o_a1 = a_reg[1];
    assign o_a2 = a_reg[2];
    assign o_b0 = b_reg[0];
    assign o_b1 = b_reg[1];

    integer j;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            for (j = 0; j < FEEDFORWARD_TAPS; j = j + 1)
                a_reg[j] <= 0;
            for (j = 0; j < FEEDBACK_TAPS; j = j + 1)
                b_reg[j] <= 0;
        end else if (!i_freeze) begin
            for (j = 0; j < FEEDFORWARD_TAPS; j = j + 1)
                a_reg[j] <= a_reg[j] + ((o_en * xn_reg[j]) >>> (SHIFT_BASE + i_mu));

            for (j = 0; j < FEEDBACK_TAPS; j = j + 1)
                b_reg[j] <= b_reg[j] + ((o_en * dn_reg[j]) >>> (SHIFT_BASE + i_mu));
        end else if (i_freeze) begin
            for (j = 0; j < FEEDFORWARD_TAPS; j = j + 1)
                a_reg[j] <= a_reg[j];
            for (j = 0; j < FEEDBACK_TAPS; j = j + 1)
                b_reg[j] <= b_reg[j];
        end
    end

    reg signed [FWIDTH-1:0] y_acc;
    integer k;

    always @(*) begin
        y_acc = 0;

        for (k = 0; k < FEEDFORWARD_TAPS; k = k + 1)
            y_acc = y_acc + a_reg[k] * xn_reg[k];

        for (k = 0; k < FEEDBACK_TAPS; k = k + 1)
            y_acc = y_acc + b_reg[k] * dn_reg[k];
    end

    localparam signed [FWIDTH-1:0] YN_MAX =  8191;
    localparam signed [FWIDTH-1:0] YN_MIN = -8192;

    wire signed [FWIDTH-1:0] yn_shifted;
    assign yn_shifted = y_acc >>> FRACBITS;

    assign o_yn = (yn_shifted > YN_MAX) ?  {1'b0, {(DWIDTH-1){1'b1}}} :
                (yn_shifted < YN_MIN) ?  {1'b1, {(DWIDTH-1){1'b0}}} :
                                         yn_shifted[DWIDTH-1:0];

    assign o_en = $signed({i_dn[DWIDTH-1], i_dn}) - $signed({o_yn[DWIDTH-1], o_yn});

endmodule
