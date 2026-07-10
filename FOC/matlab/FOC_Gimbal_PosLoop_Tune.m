%% FOC_Gimbal_PosLoop_Tune.m
% GM4108 云台 — 串级位置环 P(I) 自动整定（与 foc_gimbal.c / foc_config.h 一致）
%
% 控制链：
%   pitch_tgt(deg) → 位置 P(I) @500Hz → ω_ref(RPM) ─┐
%                                                   ├→ 速度 PI @1kHz → Iq → Uq → Plant
%   pitch_fb(deg)  ← 角度积分 ← ω ←─────────────────┘
%
% 固件特性（仿真内建模）：
%   - 位置环分频、死区、OMEGA_MIN / OMEGA_MAX
%   - 位置环模式下 Ke 前馈用 rpm_fb（非 target）
%   - 到位：死区 + |rpm|<STOP + |ω_ref|<0.5 → 零力矩
%
% 用法：
%   cd 到 FOC/matlab，运行本脚本
%   1. 确认下方 Pcfg 与 foc_config.h 同步（尤其已整定好的速度 PI）
%   2. 修改 Tune.profile = 'ramp' 或 'step'
%   3. 将输出的 #define 写入 foc_config.h，实机验证
%
% 需要：MATLAB R2019b+（fminsearch）；无 Simulink 依赖

clear; clc; close all;
rng(42);   % 角度噪声可重复，整定结果稳定

%% ========== 1. 参数（与 foc_config.h 同步） ==========
Pcfg.R           = 11.2;
Pcfg.L           = 3.0e-3;
Pcfg.Ke          = 0.340;
Pcfg.Iq_max      = 0.50;
Pcfg.Vbus        = 14.8;
Pcfg.Vq_max      = Pcfg.Vbus * 0.85;
Pcfg.Vq_slew     = 0.5;
Pcfg.Uq_sign     = -1.0;

Pcfg.fs_spd      = 1000;
Pcfg.Ts_spd      = 1 / Pcfg.fs_spd;
Pcfg.fs_pos      = 500;           % FOC_POS_LOOP_HZ
Pcfg.Ts_pos      = 1 / Pcfg.fs_pos;
Pcfg.pos_div     = Pcfg.fs_spd / Pcfg.fs_pos;

Pcfg.tau_spd     = 0.04;
Pcfg.spd_dead    = 1.0;

% 已整定速度内环（整定位置环时固定）
Pcfg.spd_P       = 0.0505;
Pcfg.spd_I       = 0.0020;

% 位置环当前值（对比用）
Pcfg.pos_P0      = 5.5;
Pcfg.pos_I0      = 0.0;

Pcfg.omega_max   = 30.0;
Pcfg.omega_min   = 12.0;
Pcfg.min_omega_err = 2.0;
Pcfg.deadband    = 0.30;
Pcfg.stop_rpm    = 2.0;
Pcfg.fine_rpm    = 5.0;

% 机械（与 SpeedLoop_Tune 一致，可按实机改）
Pcfg.J           = 2.5e-5;
Pcfg.B           = 1.0e-5;
Pcfg.Tc          = 0.002;

% 角度测量噪声（deg RMS，0=理想编码器）
Pcfg.pitch_noise = 0.02;

%% ========== 2. 整定任务 ==========
Tune.profile     = 'ramp';        % 'ramp' | 'step'
Tune.t_end       = 35.0;

% 斜坡（同 foc_test 默认）
Tune.ramp_min    = -15.0;
Tune.ramp_max    = 15.0;
Tune.ramp_rate   = 3.0;           % deg/s

% 阶跃
Tune.step_deg    = 10.0;

% 搜索 [P_pos RPM/deg, I_pos RPM/(deg·s)]
Tune.P_bounds    = [0.5, 12.0];
Tune.I_bounds    = [0.0, 0.15];
Tune.weights     = struct( ...
    'itae',       1.0, ...
    'overshoot',  0.4, ...
    'tail_osc',   0.8, ...        % 到位段振荡惩罚
    'sat',        0.05);

%% ========== 3. fminsearch 整定 ==========
fprintf('=== 位置环串级仿真整定 ===\n');
fprintf('  内环速度 PI 固定: P=%.4f  I=%.4f\n', Pcfg.spd_P, Pcfg.spd_I);
fprintf('  外环频率: %d Hz  测试: %s\n\n', Pcfg.fs_pos, Tune.profile);

x0 = [Pcfg.pos_P0, max(Pcfg.pos_I0, 0.005)];
cost_fun = @(x) sim_pos_cost(x(1), x(2), Pcfg, Tune);

options = optimset('Display', 'iter', 'TolX', 1e-3, 'TolFun', 1e-2, ...
                   'MaxFunEvals', 100, 'MaxIter', 50);
[x_opt, ~] = fminsearch(cost_fun, x0, options);
P_opt = x_opt(1);
I_opt = x_opt(2);

fprintf('\n--- 整定结果（写入 foc_config.h）---\n');
fprintf('#define FOC_POS_PI_P                %.4ff\n', P_opt);
fprintf('#define FOC_POS_PI_I                %.4ff\n\n', I_opt);

%% ========== 4. 波形对比 ==========
[y0, u0] = sim_position(Pcfg.pos_P0, Pcfg.pos_I0, Pcfg, Tune);
[y1, u1] = sim_position(P_opt, I_opt, Pcfg, Tune);

figure('Name', 'FOC Position Loop Tune', 'Color', 'w');
subplot(4,1,1);
plot(y0.t, y0.pitch_tgt, 'm--', 'LineWidth', 1.2); hold on;
plot(y0.t, y0.pitch_true, 'Color', [0.75 0.75 0.75]);
plot(y0.t, y0.pitch_fb, 'b', 'LineWidth', 1.0);
plot(y1.t, y1.pitch_fb, 'r', 'LineWidth', 1.2);
ylabel('Pitch (deg)');
legend('target', 'true', 'fb 当前', 'fb 整定', 'Location', 'best');
title(sprintf('位置环  当前 P=%.2f I=%.3f  →  整定 P=%.2f I=%.3f', ...
    Pcfg.pos_P0, Pcfg.pos_I0, P_opt, I_opt));
grid on;

subplot(4,1,2);
plot(y0.t, y0.omega_ref, 'b'); hold on;
plot(y1.t, y1.omega_ref, 'r');
ylabel('\omega_{ref} (RPM)'); legend('当前', '整定'); grid on;

subplot(4,1,3);
plot(y0.t, u0.rpm_meas, 'b'); hold on;
plot(y1.t, u1.rpm_meas, 'r');
ylabel('rpm meas'); legend('当前', '整定'); grid on;

subplot(4,1,4);
plot(y0.t, u0.iq, 'b'); hold on;
plot(y1.t, u1.iq, 'r');
ylabel('Iq (A)'); xlabel('时间 (s)'); legend('当前', '整定'); grid on;

%% ========== 5. 误差统计 ==========
print_metrics('当前 PI', y0);
print_metrics('整定 PI', y1);

fprintf('\n完成。请将 #define 复制到 foc_config.h；实机仍须 VOFA 验证。\n');
fprintf('提示：若整定 P 很小，可检查 OMEGA_MIN=%.0f 与 MIN_ERR=%.1f 是否匹配。\n', ...
    Pcfg.omega_min, Pcfg.min_omega_err);

%% ======================== 本地函数 ========================

function cost = sim_pos_cost(P_pos, I_pos, Pcfg, Tune)
    Pcfg_n = Pcfg;
    Pcfg_n.pitch_noise = 0;   % 整定代价不用随机噪声
    if P_pos < Tune.P_bounds(1) || P_pos > Tune.P_bounds(2) || ...
       I_pos < Tune.I_bounds(1) || I_pos > Tune.I_bounds(2)
        cost = 1e6;
        return;
    end
    y = sim_position(P_pos, I_pos, Pcfg_n, Tune);
    e = y.pitch_tgt - y.pitch_fb;
    t = y.t;

    cost = Tune.weights.itae * trapz(t, t .* abs(e));

    % 超调（相对最终段目标）
    tgt_final = y.pitch_tgt(end);
    if strcmp(Tune.profile, 'step')
        overshoot = max(abs(y.pitch_fb - tgt_final)) - abs(Tune.step_deg);
        if overshoot > 0
            cost = cost + Tune.weights.overshoot * overshoot^2;
        end
    end

    % 末段 20%% 时间：抑制到位抖动
    i0 = max(1, floor(0.8 * numel(t)));
    tail_e = y.pitch_fb(i0:end) - y.pitch_tgt(i0:end);
    cost = cost + Tune.weights.tail_osc * std(tail_e)^2;

    sat_frac = mean(abs(y.iq) >= Pcfg.Iq_max - 1e-6);
    cost = cost + Tune.weights.sat * sat_frac * 100;
end

function [y, u] = sim_position(P_pos, I_pos, Pcfg, Tune)
    N  = ceil(Tune.t_end / Pcfg.Ts_spd);
    t  = (0:N-1)' * Pcfg.Ts_spd;

    pitch_tgt  = zeros(N, 1);
    pitch_true = zeros(N, 1);
    pitch_fb   = zeros(N, 1);
    omega_ref_l= zeros(N, 1);
    rpm_meas   = zeros(N, 1);
    iq_log     = zeros(N, 1);

    if strcmp(Tune.profile, 'ramp')
        pitch_tgt = ramp_pingpong(t, Tune.ramp_min, Tune.ramp_max, Tune.ramp_rate);
    else
        pitch_tgt(:) = Tune.step_deg;
    end

    iq         = 0;
    omega      = 0;
    rpm_m      = 0;
    pitch_deg  = 0;
    sum_pos_e  = 0;
    sum_spd_e  = 0;
    vq_ap      = 0;
    omega_ref  = 0;
    pos_in_db  = false;

    for k = 1:N
        tk = t(k);

        % 位置环 @ fs_pos
        if mod(k - 1, Pcfg.pos_div) == 0
            pitch_meas = pitch_deg + Pcfg.pitch_noise * randn();
            err = pitch_tgt(k) - pitch_meas;
            sum_pos_e = sum_pos_e + err;

            omega_unsat = P_pos * err + I_pos * sum_pos_e * Pcfg.Ts_pos;
            omega_ref = clamp(omega_unsat, -Pcfg.omega_max, Pcfg.omega_max);

            if abs(err) < Pcfg.deadband
                pos_in_db = true;
                if I_pos > 0
                    omega_ref = clamp(omega_ref, -Pcfg.fine_rpm, Pcfg.fine_rpm);
                    if abs(omega_ref) < 0.3
                        omega_ref = 0;
                    end
                else
                    omega_ref = 0;
                end
            else
                pos_in_db = false;
                if (Pcfg.omega_min > 0) && (abs(err) > Pcfg.min_omega_err) && ...
                        (abs(omega_ref) < Pcfg.omega_min)
                    omega_ref = sign_nonzero(err) * Pcfg.omega_min;
                end
            end
        end

        target_speed = omega_ref;

        % 速度环 @ 1 kHz（位置模式）
        if pos_in_db && (abs(rpm_m) < Pcfg.stop_rpm) && (abs(target_speed) < 0.5)
            iq = 0;
            sum_spd_e = 0;
            vq_ap = 0;
            Uq = 0;
        else
            e_spd = target_speed - rpm_m;
            sum_spd_e = sum_spd_e + e_spd;
            iq_unsat = Pcfg.spd_P * e_spd + Pcfg.spd_I * sum_spd_e * Pcfg.Ts_spd;
            iq = clamp(iq_unsat, -Pcfg.Iq_max, Pcfg.Iq_max);
            if abs(iq_unsat) > Pcfg.Iq_max
                sum_spd_e = sum_spd_e - (iq_unsat - iq) / max(Pcfg.spd_I, 1e-9) / Pcfg.Ts_spd;
            end

            vq_ff = Pcfg.R * iq + Pcfg.Ke * rpm2rad(rpm_m);   % 位置环：Ke 用 rpm_fb
            vq_tgt = clamp(vq_ff, -Pcfg.Vq_max, Pcfg.Vq_max);
            vq_ap  = slew(vq_ap, vq_tgt, Pcfg.Vq_slew);
            Uq     = Pcfg.Uq_sign * vq_ap;
        end

        [iq, omega] = plant_step(iq, omega, Uq, Pcfg);

        pitch_deg = pitch_deg + rad2deg(omega) * Pcfg.Ts_spd;
        rpm_true_k = rad2rpm(omega);
        alpha = Pcfg.Ts_spd / Pcfg.tau_spd;
        rpm_m = rpm_m + alpha * (rpm_true_k - rpm_m);
        if abs(rpm_m) < Pcfg.spd_dead
            rpm_m = 0;
        end

        pitch_true(k)  = pitch_deg;
        pitch_fb(k)    = pitch_deg + Pcfg.pitch_noise * randn();
        omega_ref_l(k) = omega_ref;
        rpm_meas(k)    = rpm_m;
        iq_log(k)      = iq;
    end

    y = struct('t', t, 'pitch_tgt', pitch_tgt, 'pitch_true', pitch_true, ...
               'pitch_fb', pitch_fb, 'omega_ref', omega_ref_l, 'iq', iq_log);
    u = struct('rpm_meas', rpm_meas, 'iq', iq_log);
end

function pitch = ramp_pingpong(t, deg_min, deg_max, rate_dps)
    pitch = zeros(size(t));
    span = deg_max - deg_min;
    leg_t = span / rate_dps;
    period = 2 * leg_t;
    for i = 1:numel(t)
        tm = mod(t(i), period);
        if tm < leg_t
            pitch(i) = deg_min + rate_dps * tm;
        else
            pitch(i) = deg_max - rate_dps * (tm - leg_t);
        end
    end
end

function print_metrics(name, y)
    e = y.pitch_tgt - y.pitch_fb;
    fprintf('[%s]  MAE=%.3f deg  Max|e|=%.3f deg  std(末段)=%.4f deg\n', ...
        name, mean(abs(e)), max(abs(e)), std(e(floor(0.8*end):end)));
end

function [iq, omega] = plant_step(iq, omega, Uq, Pcfg)
    di = (Uq - Pcfg.R * iq - Pcfg.Ke * omega) / Pcfg.L;
    iq = iq + di * Pcfg.Ts_spd;
    Tc = Pcfg.Tc * sign_nonzero(omega);
    domega = (Pcfg.Ke * iq - Pcfg.B * omega - Tc) / Pcfg.J;
    omega = omega + domega * Pcfg.Ts_spd;
end

function y = slew(y0, tgt, dv_max)
    dv = tgt - y0;
    if dv > dv_max
        y = y0 + dv_max;
    elseif dv < -dv_max
        y = y0 - dv_max;
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
