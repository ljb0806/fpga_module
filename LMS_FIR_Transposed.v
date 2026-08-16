

// =============================================================================
// module   : LMS_Transposed
// author   : binbin
// function : 基于 LMS（最小均方）算法的自适应 FIR 滤波器，采用转置结构。
//            每个时钟周期完成一次 FIR 滤波 + 一次系数更新，系数收敛后
//            滤波器输出 o_yn 逼近期望信号 i_dn。
//
// 实现思路 :
//   - 滤波器结构 : TAPS 阶 FIR
//       y(n) = Σ w_k * x(n-k),  k = 0 .. TAPS-1
//       输入 i_xn → 延迟线 (xn_reg) → 各抽头与对应权值 wn_reg 乘加
//   - LMS 权值更新 (可冻结) :
//       w_k(n+1) = w_k(n) + i_mu * o_en * x(n-k)
//       步长通过算术右移实现: >>> (2*DWIDTH-1-FRACBITS + i_mu)
//   - 定点格式 :
//       数据 (i_xn,i_dn,o_yn) : Q1.13 有符号数（DWIDTH=14）
//       权值 (wn_reg)    : Q1.29 有符号数（CWIDTH=32, FRACBITS=30）
//   - 输出限幅 : o_yn 限制在 [-8192, 8191]（14-bit 有符号满量程）
//   - i_freeze 信号 : 高有效时冻结权值更新，滤波器退化为固定系数 FIR
//
// 参数说明 :
//   TAPS    = 32 : FIR 抽头数
//   DWIDTH  = 14 : 数据位宽
//   CWIDTH  = 32 : 系数位宽
//   FRACBITS= 30 : 系数小数位数
//
// 端口说明 :
//   i_clk, i_rst_n : 时钟与异步复位（低有效）
//   i_freeze     : 系数冻结（高有效），冻结时停止 LMS 更新
//   i_mu         : LMS 步长控制字（指数部分）
//   i_xn         : 滤波器输入信号
//   i_dn         : 期望信号（训练参考）
//   o_yn         : 滤波器输出（限幅后）
//   o_en         : 误差信号 = i_dn - o_yn（扩展 1 bit 防溢出）
//
// =============================================================================

module LMS_Transposed #
(
    parameter                           TAPS = 32                  ,
    parameter                           DWIDTH = 14                ,
    parameter                           CWIDTH = 32                ,
    parameter                           FRACBITS = 30               
)
(
    input                               i_clk                      ,
    input                               i_rst_n                    ,
    input                               i_freeze                   ,

    input              [   7:0]         i_mu                       ,

    input       signed [DWIDTH-1:0]     i_xn                       ,
    input       signed [DWIDTH-1:0]     i_dn                       ,
    output      signed [DWIDTH-1:0]     o_yn                       ,
    output      signed [DWIDTH:0]       o_en                        
);

reg signed [DWIDTH-1:0] xn_reg [0:TAPS-1];
integer i;

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        for(i=0; i<TAPS; i=i+1) begin
            xn_reg[i] <= 0;
        end
    end
    else begin
        for(i=1; i<TAPS; i=i+1) begin
            xn_reg[i] <= xn_reg[i-1];
        end
        xn_reg[0] <= i_xn;
    end
end

reg signed [2*DWIDTH:0] mul_res [0:TAPS-1];
reg signed [CWIDTH-1:0] wn_reg [0:TAPS-1];
integer j;
integer m;
always @(*) begin
    for(m=0; m<TAPS; m=m+1) begin
        mul_res[m] = o_en * xn_reg[m];
    end
end

always @(posedge i_clk or negedge i_rst_n) begin
    if(!i_rst_n) begin
        for(j=0; j<TAPS; j=j+1) begin
            wn_reg[j] <= 0;
        end
    end
    else if(!i_freeze) begin
        for(j=0; j<TAPS; j=j+1) begin
            wn_reg[j] <= wn_reg[j] + ((o_en * xn_reg[j]) >>> (2*DWIDTH-1-FRACBITS + i_mu));
        end
    end
    else if(i_freeze) begin
        for(j=0; j<TAPS; j=j+1) begin
            wn_reg[j] <= wn_reg[j];
        end
    end
end

integer k;
reg signed [DWIDTH + CWIDTH + $clog2(TAPS):0] y_acc;

always @(*) begin
    y_acc = 0;
    for (k = 0; k < TAPS; k = k + 1)
        y_acc = y_acc + wn_reg[k] * xn_reg[k];
end

localparam signed [DWIDTH + CWIDTH + $clog2(TAPS):0] YN_MAX = 8191;
localparam signed [DWIDTH + CWIDTH + $clog2(TAPS):0] YN_MIN = -8192;

wire signed [DWIDTH + CWIDTH + $clog2(TAPS):0] yn_shifted;
assign yn_shifted = y_acc >>> FRACBITS;

assign o_yn = (yn_shifted > YN_MAX) ? {1'b0, {(DWIDTH-1){1'b1}}} :
            (yn_shifted < YN_MIN) ? {1'b1, {(DWIDTH-1){1'b0}}} :
            yn_shifted[DWIDTH-1:0];

assign o_en = $signed({i_dn[DWIDTH-1], i_dn}) - $signed({o_yn[DWIDTH-1], o_yn});

endmodule
