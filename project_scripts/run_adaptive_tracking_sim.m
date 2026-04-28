clc; clear; close all;

%% ============================================================
%  RBE 502 Final Project
%  Simulation 6: Adaptive Control - Trajectory Tracking
%  OpenManipulator-X
%
%  Safe adaptive version:
%  - smooth trajectory with zero initial velocity
%  - normalized adaptation
%  - parameter projection
%  - torque saturation
%  - instability stop condition
% ============================================================

clc; clear; close all;

%% Robust repo-root detection
% This script lives in project_scripts.
% The repo root is one folder above this script.
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);
addpath(genpath(repo_root));
results_dir = fullfile(repo_root, 'project_results', 'sim_adaptive_tracking_safe');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Load true model parameters
[p_true, theta_true, geom_params, g_val] = load_openmanipulator_params();

%% Simulation settings
dt = 0.001;
T  = 12.0;
t  = 0:dt:T;
N  = length(t);

%% Desired trajectory parameters
% Smooth trajectory with zero initial velocity.
% qd(t) = q_offset + A*(1 - cos(omega*t))
q_offset = [0.10; -0.25; 0.25; 0.10];     % rad
A        = [0.10;  0.08; 0.08; 0.05];     % rad
omega    = 0.35;                           % rad/s

%% Initial state
% At t = 0, qd = q_offset and qd_dot = 0
q    = q_offset;
qdot = zeros(4,1);

%% Adaptive controller gains
Lambda = diag([2.5, 2.5, 2.0, 1.5]);
Kd     = diag([0.25, 0.25, 0.18, 0.10]);

Gamma = diag([0.08, 0.08, 0.08, 0.08, ...
              0.01, 0.01, 0.01, ...
              0.01, 0.01, 0.01, ...
              0.01, 0.01, 0.01, ...
              0.01, 0.01, 0.01]);

sigma = 0.02;

%% Initial parameter estimate
theta_nom = theta_true;
theta_hat = 0.75 * theta_true;

%% Parameter projection bounds
theta_margin = 0.8 * abs(theta_true) + 1e-6;
theta_lower = theta_true - theta_margin;
theta_upper = theta_true + theta_margin;

% Keep mass estimates positive
for i = 1:4
    theta_lower(i) = max(theta_lower(i), 0.01);
end

%% Torque limits
tau_limit = [1.5; 1.5; 1.0; 0.6];  % Nm

%% Storage
q_hist          = zeros(4,N);
qdot_hist       = zeros(4,N);
qd_hist         = zeros(4,N);
qd_dot_hist     = zeros(4,N);
qd_ddot_hist    = zeros(4,N);
tau_hist        = zeros(4,N);
s_hist          = zeros(4,N);
e_hist          = zeros(4,N);
e_norm_hist     = zeros(1,N);
theta_hat_hist  = zeros(16,N);

unstable_flag = false;
stop_index = N;

%% Main simulation loop
for k = 1:N

    tk = t(k);

    % Desired trajectory
    qd      = q_offset + A .* (1 - cos(omega*tk));
    qd_dot  = A .* omega .* sin(omega*tk);
    qd_ddot = A .* omega^2 .* cos(omega*tk);

    % Conventional tracking error
    e    = qd - q;
    edot = qd_dot - qdot;

    % Reference velocity and acceleration
    qdot_r  = qd_dot  + Lambda * e;
    qddot_r = qd_ddot + Lambda * edot;

    % Filtered tracking error
    s = qdot_r - qdot;

    % Build numeric regressor
    Y = build_numeric_regressor(q, qdot, qdot_r, qddot_r, geom_params, g_val);

    % Adaptive control law
    tau = Y * theta_hat + Kd * s;

    % Torque saturation
    tau = max(min(tau, tau_limit), -tau_limit);

    % Normalized adaptation law with leakage
    normalization = 1.0 + norm(Y, 'fro')^2;
    theta_hat_dot = Gamma * (Y') * s / normalization ...
                    - sigma * (theta_hat - theta_nom);

    % True plant dynamics
    M = M_fun(q, p_true);
    C = C_fun(q, qdot, p_true);
    G = G_fun(q, p_true);

    qddot = M \ (tau - C*qdot - G);

    % Store data
    q_hist(:,k)         = q;
    qdot_hist(:,k)      = qdot;
    qd_hist(:,k)        = qd;
    qd_dot_hist(:,k)    = qd_dot;
    qd_ddot_hist(:,k)   = qd_ddot;
    tau_hist(:,k)       = tau;
    s_hist(:,k)         = s;
    e_hist(:,k)         = e;
    e_norm_hist(k)      = norm(e, 2);
    theta_hat_hist(:,k) = theta_hat;

    % Stop if simulation becomes unstable
    if any(~isfinite(q)) || any(~isfinite(qdot)) || any(~isfinite(theta_hat)) || ...
       e_norm_hist(k) > 5 || norm(qdot) > 20

        unstable_flag = true;
        stop_index = k;

        fprintf('\nAdaptive tracking stopped early at t = %.4f s due to instability.\n', t(k));
        break;
    end

    % Integrate
    if k < N
        theta_hat = theta_hat + dt * theta_hat_dot;

        % Projection
        theta_hat = max(min(theta_hat, theta_upper), theta_lower);

        qdot = qdot + dt * qddot;
        q    = q + dt * qdot;
    end
end

%% Trim arrays if stopped early
t_plot = t(1:stop_index);

q_hist          = q_hist(:,1:stop_index);
qdot_hist       = qdot_hist(:,1:stop_index);
qd_hist         = qd_hist(:,1:stop_index);
qd_dot_hist     = qd_dot_hist(:,1:stop_index);
qd_ddot_hist    = qd_ddot_hist(:,1:stop_index);
tau_hist        = tau_hist(:,1:stop_index);
s_hist          = s_hist(:,1:stop_index);
e_hist          = e_hist(:,1:stop_index);
e_norm_hist     = e_norm_hist(1:stop_index);
theta_hat_hist  = theta_hat_hist(:,1:stop_index);

%% Save numerical results
save(fullfile(results_dir, 'adaptive_tracking_results.mat'), ...
    't_plot', 'q_hist', 'qdot_hist', 'qd_hist', 'qd_dot_hist', ...
    'qd_ddot_hist', 'tau_hist', 's_hist', 'e_hist', 'e_norm_hist', ...
    'theta_hat_hist', 'theta_true', 'theta_nom', ...
    'Lambda', 'Kd', 'Gamma', 'sigma', ...
    'q_offset', 'A', 'omega', 'unstable_flag', 'tau_limit');

%% Plot 1: Desired and actual joint positions
fig1 = figure('Name', 'Adaptive Tracking - Joint Positions');

for i = 1:4
    subplot(4,1,i);
    plot(t_plot, qd_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t_plot, q_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['q_', num2str(i), ' (rad)']);
    legend('desired', 'actual');
end

xlabel('Time (s)');
sgtitle('Adaptive Control - Trajectory Tracking: Joint Positions');
drawnow;
saveas(fig1, fullfile(results_dir, 'adaptive_tracking_positions.png'));

%% Plot 2: Desired and actual joint velocities
fig2 = figure('Name', 'Adaptive Tracking - Joint Velocities');

for i = 1:4
    subplot(4,1,i);
    plot(t_plot, qd_dot_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t_plot, qdot_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['dq_', num2str(i), ' (rad/s)']);
    legend('desired', 'actual');
end

xlabel('Time (s)');
sgtitle('Adaptive Control - Trajectory Tracking: Joint Velocities');
drawnow;
saveas(fig2, fullfile(results_dir, 'adaptive_tracking_velocities.png'));

%% Plot 3: Control torques
fig3 = figure('Name', 'Adaptive Tracking - Control Torques');

for i = 1:4
    subplot(4,1,i);
    plot(t_plot, tau_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['\tau_', num2str(i), ' (Nm)']);
end

xlabel('Time (s)');
sgtitle('Adaptive Control - Trajectory Tracking: Control Torques');
drawnow;
saveas(fig3, fullfile(results_dir, 'adaptive_tracking_torques.png'));

%% Plot 4: Tracking error norm
fig4 = figure('Name', 'Adaptive Tracking - Error Norm');

plot(t_plot, e_norm_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('||e(t)||_2');
title('Adaptive Control - Trajectory Tracking: Tracking Error Norm');
drawnow;
saveas(fig4, fullfile(results_dir, 'adaptive_tracking_error_norm.png'));

%% Plot 5: Estimated parameters
fig5 = figure('Name', 'Adaptive Tracking - Estimated Parameters');

for i = 1:16
    subplot(4,4,i);
    plot(t_plot, theta_hat_hist(i,:), 'LineWidth', 1.2); hold on;
    yline(theta_true(i), '--');
    grid on;
    title(['\theta_', num2str(i)]);
end

sgtitle('Adaptive Control - Trajectory Tracking: Parameter Estimates');
drawnow;
saveas(fig5, fullfile(results_dir, 'adaptive_tracking_parameters.png'));

%% Print summary
fprintf('\nAdaptive Tracking Simulation Complete.\n');
fprintf('Unstable flag: %d\n', unstable_flag);
fprintf('Final joint error norm: %.6f rad\n', e_norm_hist(end));
fprintf('Maximum joint error norm: %.6f rad\n', max(e_norm_hist));
fprintf('Mean joint error norm: %.6f rad\n', mean(e_norm_hist));
fprintf('Results saved in:\n%s\n', results_dir);