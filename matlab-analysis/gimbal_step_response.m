%% gimbal_step_response.m
% Closed-loop step-response analysis of the 2-axis self-stabilizing gimbal's
% pitch axis: settling time, overshoot, and steady-state error, compared
% between the as-shipped controller gains and a tuned version.
%
% CONTROLLER EQUATIONS are copied exactly from the real firmware
% (gimbal pid controller.ino). They were verified against a live run of
% that exact sketch in the Wokwi simulator on 2026-09-30: forcing the
% MPU6050's simulated X-acceleration to +2 g produced
%   Roll: -63.43 -> Servo: 180
% in the real serial output, which matches this script's controller math
% exactly: output = Kp*error = 3*63.43 = 190.3, servo = clamp(90+190.3,0,180) = 180.
%
% PLANT MODEL (how the physical platform angle evolves as the servo moves)
% is representative, not measured -- Wokwi has no simulated mechanical link
% between the servo and MPU6050 parts, so it can confirm the controller's
% math but not produce a genuine closed-loop time response. Two assumptions
% are used here, both clearly separable from the controller code above:
%   1. Direct-drive coupling: 1 deg of servo rotation <-> 1 deg of change in
%      sensed platform tilt (mechanical_gain = 1), signed so the loop is
%      negative feedback (matching the system's intended/designed behaviour).
%   2. The SG90 servo is modeled as a first-order lag (tau = 0.06 s) toward
%      its commanded angle, consistent with the ~0.1 s / 60 deg datasheet spec.
%
% To use REAL data instead: replace the two run_simulation() calls below
% with a call to import_real_log(), which reads a CSV exported from the
% Arduino/Wokwi Serial Monitor (time_ms, pitch, roll columns) -- the
% metrics() and plotting code below need no changes either way.

clear; clc; close all;

%% Parameters
dt        = 0.02;   % s, matches delay(20) in the firmware control loop
tau_servo = 0.06;    % s, representative SG90 first-order lag constant
mech_gain = 1.0;      % deg platform tilt change per deg servo travel
STEP      = 30.0;      % deg, simulated disturbance (rig tilted by hand)
T_END     = 3.0;        % s

%% Run both configurations
[t_asis,  theta_asis]  = run_simulation(3, 0.0, 0.0, dt, tau_servo, mech_gain, STEP, T_END);
[t_tuned, theta_tuned] = run_simulation(3, 2.0, 0.0, dt, tau_servo, mech_gain, STEP, T_END);

m_asis  = step_metrics(t_asis,  theta_asis,  STEP);
m_tuned = step_metrics(t_tuned, theta_tuned, STEP);

fprintf('AS-SHIPPED  (Kp=3, Ki=0,   Kd=0): settle=%.2fs  overshoot=%.1f%%  ss-error=%.2f deg\n', ...
    m_asis.settling_time_s, m_asis.overshoot_pct, m_asis.steady_state_error_deg);
fprintf('TUNED       (Kp=3, Ki=2.0, Kd=0): settle=%.2fs  overshoot=%.1f%%  ss-error=%.2f deg\n', ...
    m_tuned.settling_time_s, m_tuned.overshoot_pct, m_tuned.steady_state_error_deg);

%% Plot
figure('Position', [100 100 900 550]);
hold on;
yline(0, ':', 'Setpoint (0 deg)', 'Color', [0.55 0.55 0.55], 'LabelHorizontalAlignment', 'left');
plot(t_asis,  theta_asis,  'Color', [0.85 0.33 0.31], 'LineWidth', 2, 'DisplayName', 'As-shipped: Kp=3, Ki=0, Kd=0');
plot(t_tuned, theta_tuned, 'Color', [0.18 0.49 0.85], 'LineWidth', 2, 'DisplayName', 'Tuned: Kp=3, Ki=2.0, Kd=0');
xline(m_asis.settling_time_s,  '--', 'Color', [0.85 0.33 0.31]);
xline(m_tuned.settling_time_s, '--', 'Color', [0.18 0.49 0.85]);
xlabel('Time (s)');
ylabel('Pitch angle (deg)');
title({'Gimbal pitch-axis step response (30 deg disturbance)', ...
       'simulated from real firmware PID equations + representative servo/plant model'});
legend('Location', 'northeast');
grid on;
hold off;
saveas(gcf, 'step_response_matlab.png');

%% ------------------------------------------------------------------
function [t, theta] = run_simulation(Kp, Ki, Kd, dt, tau_servo, mech_gain, STEP, T_END)
    n = round(T_END / dt);
    t = (0:n-1)' * dt;

    theta = zeros(n, 1);   % true physical tilt angle (deg) -- what the MPU reads
    servo = zeros(n, 1);   % true physical servo shaft angle (deg)
    servo(1) = 90.0;
    theta(1) = STEP;        % disturbance applied at t = 0

    integral_term = 0.0;
    last_error = 0.0;

    for k = 2:n
        % ---- controller, copied from the firmware ----
        error = 0.0 - theta(k-1);
        integral_term = integral_term + error * dt;
        integral_term = max(min(integral_term, 50), -50);
        derivative = (error - last_error) / dt;
        last_error = error;
        output = Kp * error + Ki * integral_term + Kd * derivative;
        servo_cmd = max(min(90 + output, 180), 0);

        % ---- servo first-order lag toward commanded angle ----
        servo(k) = servo(k-1) + (servo_cmd - servo(k-1)) * (dt / tau_servo);

        % ---- plant: servo motion drives the platform angle ----
        theta(k) = STEP + mech_gain * (servo(k) - 90.0);
    end
end

function m = step_metrics(t, theta, step)
    band_pct = 0.02;
    tailN = max(10, floor(length(theta) / 20));
    final_value = mean(theta(end - tailN + 1:end));
    band = band_pct * abs(step);

    within = abs(theta - final_value) <= band;
    settle_idx = length(t);
    for i = length(t):-1:1
        if ~within(i)
            settle_idx = min(i + 1, length(t));
            break;
        end
        settle_idx = 1;
    end
    settling_time_s = t(settle_idx);

    if final_value >= theta(1)
        peak = max(theta(2:end));
        overshoot_deg = max(0, peak - final_value);
    else
        peak = min(theta(2:end));
        overshoot_deg = max(0, final_value - peak);
    end
    overshoot_pct = 100 * overshoot_deg / abs(step);
    steady_state_error_deg = final_value - 0.0;

    m = struct('final_value_deg', final_value, 'settling_time_s', settling_time_s, ...
               'overshoot_pct', overshoot_pct, 'steady_state_error_deg', steady_state_error_deg);
end

function [t, pitch, roll] = import_real_log(csv_path)
    % Reads a real logged run exported from the Arduino/Wokwi Serial
    % Monitor. Expects columns: time_ms, pitch, roll (add a Serial.print
    % line to the firmware if it isn't already printing in this format).
    T = readtable(csv_path);
    t = T.time_ms / 1000.0;
    pitch = T.pitch;
    roll = T.roll;
end
