clear; close all; clc;

% ---- 用户可修改参数 ----
fs        = 50e6;          % 采样率 [Hz]
filt_type = 'low';        % 滤波器类型: 'low' | 'high' | 'bandpass' | 'stop'
order     = 2;            % IIR 阶数
filt_kind = 'butter';     % 滤波器种类: 'butter' | 'cheby1' | 'cheby2' | 'ellip'
Rp        = 1;            % 通带纹波 [dB] (仅 cheby1 / ellip)
Rs        = 60;           % 阻带衰减 [dB] (仅 cheby2 / ellip)
fcut      = 100e3;         % 截止频率 [Hz], band/stop 时用两元素向量如 [50e3, 200e3]

% ---- 定点参数 (与 IIR_Designer.v 保持一致) ----
DWIDTH   = 14;
CWIDTH   = 32;
FRACBITS = 28;

Wn = fcut / (fs/2);

switch lower(filt_kind)
    case 'butter'
        [b, a] = butter(order, Wn, filt_type);
    case 'cheby1'
        [b, a] = cheby1(order, Rp, Wn, filt_type);
    case 'cheby2'
        [b, a] = cheby2(order, Rs, Wn, filt_type);
    case 'ellip'
        [b, a] = ellip(order, Rp, Rs, Wn, filt_type);
end

% MATLAB:  H(z) = (b(1) + b(2)*z^-1 + ...) / (1 + a(2)*z^-1 + ...)
% Verilog: H(z) = (a0 + a1*z^-1 + ...) / (1 - b1*z^-1 - ...)
% 对应: A_COEFF = b,  B_COEFF = -a(2:end)

Q = 2^FRACBITS;
A_q = round(b * Q);
B_q = round(-a(2:end) * Q);

b_q = A_q / Q;
a_q = [1, -B_q / Q];

fprintf('// 前馈系数 (A_COEFF)\n');
for i = 1:length(A_q)
    fprintf('// a%d = %d\n', i-1, A_q(i));
end
fprintf('// 反馈系数 (B_COEFF, 已取负)\n');
for i = 1:length(B_q)
    fprintf('// b%d = %d\n', i, B_q(i));
end
fprintf('\n');

nfft = 8192;
[H_float, w] = freqz(b, a, nfft, fs);
[H_fixed, ~] = freqz(b_q, a_q, nfft, fs);

figure;
subplot(2,1,1);
plot(w/1e3, 20*log10(abs(H_float)), 'b-', 'LineWidth', 1.5); hold on;
plot(w/1e3, 20*log10(abs(H_fixed)), 'r--', 'LineWidth', 1.0);
grid on; xlabel('频率 [kHz]'); ylabel('幅度 [dB]');
title(sprintf('%s %s-pass  Order=%d  fs=%.1f MHz', filt_kind, filt_type, order, fs/1e6));
legend('浮点', sprintf('Q%d', FRACBITS), 'Location', 'best');
ylim([-80, 5]);

subplot(2,1,2);
plot(w/1e3, unwrap(angle(H_float))*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(w/1e3, unwrap(angle(H_fixed))*180/pi, 'r--', 'LineWidth', 1.0);
grid on; xlabel('频率 [kHz]'); ylabel('相位 [deg]');
legend('浮点', sprintf('Q%d', FRACBITS), 'Location', 'best');
