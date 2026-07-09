%% FOC_Gimbal_SpeedLoop_Tune.m
% GM4108 云台 — 电压模式 FOC 速度环 PI 自动整定（与 FOC_bsp 固件结构一致）
%
% 控制链（1 kHz，同 foc_gimbal.c）：
%   ω_ref(RPM) → 速度 PI → Iq_ref(A) ─┐
%                                     ├→ Uq = R·Iq + ω·Ke → 斜率/限幅 → SVPWM(q轴)
%   ω_fb(RPM)  ← 测速 LPF ←──────────┘
%
% 被控对象简化（q 轴电压模式，无电流环）：
%   L·di/dt = Uq - R·i - Ke·ω
%   J·dω/dt = Ke·i - B·ω - Tc·sign(ω)     % 力矩 ≈ Ke·Iq (SI)
%
% 用法：
%   1. 在 MATLAB 中 cd 到本文件目录，运行本脚本
%   2. 修改下方「可调机械参数」J、B、Tc（若与实机差较大）
%   3. 查看整定结果，将 FOC_SPEED_PI_P / I 写入 foc_config.h
%
% 需要：MATLAB R2019b+（fminsearch）；无 Simulink 依赖

clear; clc; close all;

%% ========== 1. 参数（与 foc_config.h 同步，改 config 后请同步此处） ==========
Pcfg.R           = 11.2;          % MOTOR_R_OHM
Pcfg.L           = 3.0e-3;        % MOTOR_L_H
Pcfg.Ke          = 0.340;         % MOTOR_KE_VS_RAD  (V·s/rad)
Pcfg.Iq_max      = 0.50;          % MOTOR_IQ_MAX_A
Pcfg.Vbus        = 14.8;          % MOTOR_BUS_VOLTAGE
Pcfg.Vq_max      = Pcfg.Vbus * 0.85;
Pcfg.Vq_slew     = 0.5;           % FOC_SPEED_VQ_SLEW_V  (V/ms)
Pcfg.Uq_sign     = -1.0;          % FOC_MOTOR_UQ_SIGN

Pcfg.fs          = 1000;          % FOC_SPEED_LOOP_HZ
Pcfg.Ts          = 1 / Pcfg.fs;
Pcfg.tau_spd     = 0.04;          % FOC_SPEED_LPF_TAU
Pcfg.spd_dead    = 1.0;           % FOC_SPEED_DEADZONE_RPM

% 当前固件 PI（仿真对比用）
Pcfg.P0          = 0.040;
Pcfg.I0          = 0.004;

% 机械参数（MATLAB 专用，需按实机微调）
Pcfg.J           = 2.5e-5;        % kg·m²  转动惯量（估）
Pcfg.B           = 1.0e-5;        % N·m·s/rad 粘滞
Pcfg.Tc          = 0.002;         % N·m  库仑摩擦幅值

% 整定目标
Tune.rpm_step    = 30;            % 阶跃目标 RPM（从 0 起）
Tune.t_end       = 2.0;           % 仿真时长 (s)
Tune.use_ff_fb   = true;          % true=前馈用 ω_fb（同固件）；false=用 ω_ref（对比）

% 自动整定搜索范围 [P, I]
Tune.P_bounds    = [0.001, 0.120];   % A/RPM
Tune.I_bounds    = [0.000, 0.030];   % A/(RPM·s) 离散 PI: I*Ts 累加
Tune.weights     = struct('itae', 1.0, 'overshoot', 0.3, 'sat', 0.05);

%% ========== 2. 线性化模型 + pidtune 参考（快速初值） ==========
% 小信号：Iq → ω，一阶近似 G(s) = K / (τ_m s + 1)
omega_op = rpm2rad(Tune.rpm_step * 0.5);
K_plant  = Pcfg.Ke / (Pcfg.R * Pcfg.B + Pcfg.Ke^2);   % rad/s per A 粗估
tau_m    = Pcfg.J / Pcfg.B;
G_plant  = tf(K_plant, [tau_m, 1]);
G_plant_rpm = G_plant * (60/(2*pi));                    % RPM per A

% 测速 LPF 在环内
G_lpf = tf(1, [Pcfg.tau_spd, 1]);
G_open = G_plant_rpm * G_lpf;

fprintf('=== 线性化参考（Iq→RPM，含测速 LPF）===\n');
fprintf('  K≈%.3f RPM/A,  τ_m≈%.1f ms,  τ_lpf=%.0f ms\n', ...
    dcgain(G_plant_rpm), tau_m*1e3, Pcfg.tau_spd*1e3);

try
    [C_pid, info] = pidtune(G_open, 'PI', 0.707); %#ok<ASGLU>
    P_pidtune = C_pid.Kp / Pcfg.R;      % 近似换算到 Iq 域（仅供参考）
    I_pidtune = C_pid.Ki / Pcfg.R;
    fprintf('  pidtune 参考（需按 Iq 限幅再缩放）: P≈%.4f, I≈%.4f\n\n', ...
        P_pidtune, I_pidtune);
catch
    fprintf('  pidtune 跳过（无 Control System Toolbox 时可忽略）\n\n');
    P_pidtune = 0.01; I_pidtune = 0.002;
end

%% ========== 3. 非线性离散仿真（与固件一致） ==========
fprintf('=== 非线性离散仿真整定（fminsearch）===\n');
x0 = [0.5*(Tune.P_bounds(1)+Tune.P_bounds(2)), ...
      0.5*(Tune.I_bounds(1)+Tune.I_bounds(2))];
cost_fun = @(x) sim_cost(x(1), x(2), Pcfg, Tune);

options = optimset('Display', 'iter', 'TolX', 1e-4, 'TolFun', 1e-3, ...
                   'MaxFunEvals', 80, 'MaxIter', 40);
[x_opt, fval] = fminsearch(cost_fun, x0, options);
P_opt = x_opt(1);
I_opt = x_opt(2);

fprintf('\n--- 整定结果（写入 foc_config.h）---\n');
fprintf('#define FOC_SPEED_PI_P              %.4ff\n', P_opt);
fprintf('#define FOC_SPEED_PI_I              %.4ff\n\n', I_opt);

%% ========== 4. 对比波形：当前 PI vs 整定 PI ==========
[t0, y0, u0] = sim_step(Pcfg.P0, Pcfg.I0, Pcfg, Tune);
[t1, y1, u1] = sim_step(P_opt, I_opt, Pcfg, Tune);

figure('Name', 'FOC Speed Loop PI Tune', 'Color', 'w');
subplot(3,1,1);
plot(t0, y0.ref, 'k--', 'LineWidth', 1.2); hold on;
plot(t0, y0.rpm_true, 'Color', [0.6 0.6 0.6]);
plot(t0, y0.rpm_meas, 'b', 'LineWidth', 1.2);
plot(t1, y1.rpm_meas, 'r', 'LineWidth', 1.2);
ylabel('RPM'); legend('ref', '\omega true', 'meas 当前PI', 'meas 整定PI', ...
    'Location', 'best'); grid on;
title(sprintf('速度环阶跃 0→%d RPM  |  当前 P=%.4f I=%.4f  →  整定 P=%.4f I=%.4f', ...
    Tune.rpm_step, Pcfg.P0, Pcfg.I0, P_opt, I_opt));

subplot(3,1,2);
plot(t0, u0.iq, 'b'); hold on;
plot(t1, u1.iq, 'r');
ylabel('Iq (A)'); legend('当前', '整定'); grid on;

subplot(3,1,3);
plot(t0, u0.vq, 'b'); hold on;
plot(t1, u1.vq, 'r');
ylabel('Uq (V)'); xlabel('时间 (s)'); legend('当前', '整定'); grid on;

%% ========== 5. 阶梯给定测试（20→40 RPM，同 foc_test） ==========
TuneStep = Tune;
TuneStep.rpm_step = 0;
TuneStep.t_end    = 25;
TuneStep.stair    = true;
TuneStep.rpm_min  = 20;
TuneStep.rpm_max  = 40;
TuneStep.rpm_delta= 1;
TuneStep.hold_s   = 3.0;

[~, ys] = sim_step(P_opt, I_opt, Pcfg, TuneStep);
figure('Name', 'Stair 20-40 RPM', 'Color', 'w');
plot(ys.t, ys.ref, 'm', 'LineWidth', 1.2); hold on;
plot(ys.t, ys.rpm_meas, 'g', 'LineWidth', 1.0);
ylabel('RPM'); xlabel('时间 (s)');
legend('target', 'measured'); grid on;
title('阶梯给定 20→40 RPM（整定 PI）');

fprintf('完成。请将上方 #define 复制到 foc_config.h 后实机验证。\n');

%% ======================== 本地函数 ========================

function cost = sim_cost(P, I, Pcfg, Tune)
    if P < Tune.P_bounds(1) || P > Tune.P_bounds(2) || ...
       I < Tune.I_bounds(1) || I > Tune.I_bounds(2)
        cost = 1e6;
        return;
    end
    [~, y] = sim_step(P, I, Pcfg, Tune);
    e = y.ref - y.rpm_meas;
    t = y.t;
    cost = Tune.weights.itae * trapz(t, t .* abs(e));
    if max(y.rpm_meas) > Tune.rpm_step * 1.25 + 1
        cost = cost + Tune.weights.overshoot * (max(y.rpm_meas) - Tune.rpm_step)^2;
    end
    sat_frac = mean(abs(y.iq) >= Pcfg.Iq_max - 1e-6);
    cost = cost + Tune.weights.sat * sat_frac * 100;
end

function [t, y, u] = sim_step(P, I, Pcfg, Tune)
    N  = ceil(Tune.t_end / Pcfg.Ts);
    t  = (0:N-1)' * Pcfg.Ts;
    rpm_ref   = zeros(N, 1);
    rpm_true  = zeros(N, 1);
    rpm_meas  = zeros(N, 1);
    iq_log    = zeros(N, 1);
    vq_log    = zeros(N, 1);

    if isfield(Tune, 'stair') && Tune.stair
        rpm_ref = stair_profile(t, Tune.rpm_min, Tune.rpm_max, ...
                                Tune.rpm_delta, Tune.hold_s);
    else
        rpm_ref(:) = Tune.rpm_step;
    end

    iq    = 0;
    omega = 0;
    rpm_m = 0;
    sum_e = 0;
    vq_ap = 0;

    for k = 1:N
        e = rpm_ref(k) - rpm_m;
        sum_e = sum_e + e;
        iq_unsat = P * e + I * sum_e * Pcfg.Ts;
        iq = clamp(iq_unsat, -Pcfg.Iq_max, Pcfg.Iq_max);
        if abs(iq_unsat) > Pcfg.Iq_max
            sum_e = sum_e - (iq_unsat - iq) / max(I, 1e-9) / Pcfg.Ts;  % 回算抗饱和
        end

        omega_fb = rpm2rad(rpm_m);
        if Tune.use_ff_fb
            omega_ff = omega_fb;
        else
            omega_ff = rpm2rad(rpm_ref(k));
        end
        vq_ff = Pcfg.R * iq + Pcfg.Ke * omega_ff;
        vq_tgt = clamp(vq_ff, -Pcfg.Vq_max, Pcfg.Vq_max);
        vq_ap  = slew(vq_ap, vq_tgt, Pcfg.Vq_slew);
        Uq     = Pcfg.Uq_sign * vq_ap;

        [iq, omega] = plant_step(iq, omega, Uq, Pcfg);

        rpm_true_k = rad2rpm(omega);
        alpha = Pcfg.Ts / Pcfg.tau_spd;
        rpm_inst = rpm_true_k;
        rpm_m = rpm_m + alpha * (rpm_inst - rpm_m);
        if abs(rpm_m) < Pcfg.spd_dead
            rpm_m = 0;
        end

        rpm_true(k) = rpm_true_k;
        rpm_meas(k) = rpm_m;
        iq_log(k)   = iq;
        vq_log(k)   = vq_ap;
    end

    y = struct('t', t, 'ref', rpm_ref, 'rpm_true', rpm_true, ...
               'rpm_meas', rpm_meas, 'iq', iq_log, 'vq', vq_log);
    u = struct('iq', iq_log, 'vq', vq_log);
end

function [iq, omega] = plant_step(iq, omega, Uq, Pcfg)
    % q 轴：L di/dt = Uq - R i - Ke ω
    di = (Uq - Pcfg.R * iq - Pcfg.Ke * omega) / Pcfg.L;
    iq = iq + di * Pcfg.Ts;

    Tc = Pcfg.Tc * sign_nonzero(omega);
    domega = (Pcfg.Ke * iq - Pcfg.B * omega - Tc) / Pcfg.J;
    omega = omega + domega * Pcfg.Ts;
end

function y = slew(y, tgt, dv_max)
    dv = tgt - y;
    if dv > dv_max
        y = y + dv_max;
    elseif dv < -dv_max
        y = y - dv_max;
    else
        y = tgt;
    end
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function rpm = rad2rpm(w)
    rpm = w * 60 / (2*pi);
end

function w = rpm2rad(rpm)
    w = rpm * (2*pi) / 60;
end

function s = sign_nonzero(x)
    if x > 0
        s = 1;
    elseif x < 0
        s = -1;
    else
        s = 0;
    end
end

function ref = stair_profile(t, rpm_min, rpm_max, delta, hold_s)
    ref = zeros(size(t));
    level = rpm_min;
    t0 = 0;
    for i = 1:numel(t)
        if (t(i) - t0) >= hold_s
            t0 = t(i);
            level = level + delta;
            if level > rpm_max
                level = rpm_min;
            end
        end
        ref(i) = level;
    end
end
