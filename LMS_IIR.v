
// =============================================================================
// module   : LMS_IIR
// author   : binbin
// function : 基于 LMS（最小均方）算法的自适应 IIR 滤波器。
//            采用方程误差（Equation-Error）结构，将期望信号 i_dn 作为反馈回路的
//            参考输入，使误差曲面为二次型，保证 LMS 收敛到全局最优。
//
// 实现思路 :
//   - 滤波器结构 : 直接型 IIR，包含前馈（FIR）和反馈（IIR）两条支路
//       y(n) = Σ a_k * x(n-k) + Σ b_k * d(n-k)
//       前馈支路 : i_xn → 延迟线 → a0,a1,a2 加权求和
//       反馈支路 : i_dn → 延迟线 → b0,b1 加权求和（方程误差法）
//   - 系数更新 : LMS 迭代（可冻结）
//       a_k(n+1) = a_k(n) + i_mu * o_en * x(n-k)
//       b_k(n+1) = b_k(n) + i_mu * o_en * d(n-k)
//   - 定点格式 :
//       数据 (i_xn,i_dn,o_yn) : Q1.13 有符号数（DWIDTH=14）
//       系数 (a,b)      : Q3.28 有符号数（CWIDTH=32, FRACBITS=28）
//       内部累加器      : FWIDTH 位宽，包含防溢出扩展位
//   - 步长 i_mu        : 8-bit 无符号数，作为移位量加到 SHIFT_BASE 上，
//                        实现 2^{-(SHIFT_BASE+i_mu)} 的等效步长衰减
//   - 输出限幅         : o_yn 限制在 [-8192, 8191]（14-bit 有符号满量程），
//                        防止定点溢出
//   - i_freeze 信号    : 高有效时冻结系数更新，滤波器退化为固定系数 IIR
//
// 参数说明 :
//   FEEDFORWARD_TAPS = 3 : 前馈抽头数（a0, a1, a2）
//   FEEDBACK_TAPS   = 2 : 反馈抽头数（b0, b1）
//   DWIDTH          = 14: 数据位宽
//   CWIDTH          = 32: 系数位宽
//   FRACBITS        = 28: 系数小数位数
//
// 端口说明 :
//   i_clk, i_rst_n : 时钟与异步复位（低有效）
//   i_freeze     : 系数冻结（高有效），冻结时停止 LMS 更新
//   i_mu         : LMS 步长控制字（指数部分），决定收敛速度与稳态误差
//   i_xn         : 滤波器输入信号
//   i_dn         : 期望信号（同时作为反馈回路的参考输入）
//   o_yn         : 滤波器输出
//   o_en         : 误差信号 = i_dn - o_yn（扩展 1 bit 防溢出）
//   o_a0,o_a1,o_a2 : 前馈系数（可观测，用于调试/监控）
//   o_b0,o_b1    : 反馈系数（可观测，用于调试/监控）
//
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
