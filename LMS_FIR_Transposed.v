// =============================================================================
// 模块名称 : LMS_Transposed
// 文件名称 : LMS_FIR_Transposed.v
// 作者     : binbin
// 功能描述 : TAPS 抽头 LMS 自适应 FIR 滤波器。模块根据输入 i_xn、期望信号
//            i_dn 和误差 o_en 在线更新权值，使输出 o_yn 逐渐逼近期望信号。
//
// 实现思路 :
//   1. 使用 TAPS 级输入延迟线保存 x(n-k)。
//   2. 计算 y(n)=Σw_k(n)*x(n-k)，右移 FRACBITS 后做输出饱和。
//   3. 计算误差 e(n)=d(n)-y(n)，并按以下关系更新每个权值：
//        w_k(n+1)=w_k(n)+e(n)*x(n-k) >>> (2*DWIDTH-1-FRACBITS+i_mu)
//   4. i_freeze 拉高时保持所有权值不变，滤波数据通路继续工作。
//
// 参数说明 :
//   TAPS     = 32 : FIR 抽头数。
//   DWIDTH   = 14 : 输入、期望和输出数据位宽。
//   CWIDTH   = 32 : 自适应权值位宽。
//   FRACBITS = 30 : 权值的小数位数。
//
// 端口说明 :
//   i_clk    : 工作时钟，上升沿采样。
//   i_rst_n  : 异步复位，低有效；复位时清空延迟线和全部权值。
//   i_freeze : 权值冻结控制，高有效。
//   i_mu     : 8 位无符号步长指数；数值越大，有效更新步长越小。
//   i_xn     : DWIDTH 位有符号滤波输入。
//   i_dn     : DWIDTH 位有符号期望信号。
//   o_yn     : DWIDTH 位有符号饱和滤波输出。
//   o_en     : DWIDTH+1 位有符号误差输出，等于 i_dn-o_yn。
//
// 定点与时序 :
//   - 默认数据为 Q1.13，权值为 32 位且含 30 个小数位。
//   - 默认 14 位输出饱和到 [-8192,8191]；修改 DWIDTH 时应同步检查限幅常量。
//   - 抽头乘加与误差乘法包含较长组合路径，高频设计应考虑流水加法树。
//
// 使用注意 :
//   参数组合必须保证移位量 2*DWIDTH-1-FRACBITS+i_mu 非负且处于合理范围。
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
