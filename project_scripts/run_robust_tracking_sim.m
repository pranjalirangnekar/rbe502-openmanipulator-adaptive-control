%% ============================================================
%  RBE 502 Final Project
%  Simulation 4: Robust Control - Trajectory Tracking
%  OpenManipulator-X
% ============================================================

%% Setup paths
clc; clear; close all;

%% Robust repo-root detection
% This script lives in project_scripts.
% The repo root is one folder above this script.
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);
addpath(genpath(repo_root));

results_dir = fullfile(repo_root, 'project_results', 'sim_robust_tracking');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Load identified model parameters
R = load(fullfile('Identification', 'identification_result.mat'));
x_opt_vec = R.x_opt_vec(:);

p = [R.p(1:6); ...
     x_opt_vec(1); x_opt_vec(2); x_opt_vec(3); x_opt_vec(4); ...
     x_opt_vec(5); x_opt_vec(6); x_opt_vec(7); ...
     x_opt_vec(8); x_opt_vec(9); x_opt_vec(10); ...
     x_opt_vec(11); x_opt_vec(12); x_opt_vec(13); ...
     x_opt_vec(14); x_opt_vec(15); x_opt_vec(16); ...
     R.id_info.g];

%% Simulation settings
dt = 0.001;
T  = 12.0;
t  = 0:dt:T;
N  = length(t);

%% Desired trajectory parameters
q_offset = [0.10; -0.30; 0.30; 0.10];     % rad
A        = [0.20;  0.15; 0.15; 0.10];     % rad
omega    = 0.6;                            % rad/s

%% Initial state
q    = q_offset;
qdot = zeros(4,1);

%% Robust controller gains
Lambda = diag([4, 4, 3, 2]);

Ks = diag([0.25, 0.25, 0.18, 0.10]);

rho = [0.04; 0.04; 0.03; 0.02];
epsilon = 0.02;

%% Storage
q_hist       = zeros(4,N);
qdot_hist    = zeros(4,N);
qd_hist      = zeros(4,N);
qd_dot_hist  = zeros(4,N);
qd_ddot_hist = zeros(4,N);
tau_hist     = zeros(4,N);
s_hist       = zeros(4,N);
e_hist       = zeros(4,N);
e_norm_hist  = zeros(1,N);

%% Main simulation loop
for k = 1:N

    tk = t(k);

    % Desired trajectory
    qd      = q_offset + A .* sin(omega*tk);
    qd_dot  = A .* omega .* cos(omega*tk);
    qd_ddot = -A .* omega^2 .* sin(omega*tk);

    % Tracking errors
    e    = qd - q;
    edot = qd_dot - qdot;

    % Sliding/filtered error variable
    s = edot + Lambda*e;

    % Reference acceleration
    v = qd_ddot + Lambda*edot;

    % Robot dynamics
    M = M_fun(q, p);
    C = C_fun(q, qdot, p);
    G = G_fun(q, p);

    % Nominal model-based torque
    tau_nom = M*v + C*qdot + G;

    % Smooth robust term
    tau_robust = Ks*s + rho .* tanh(s/epsilon);

    % Total robust control input
    tau = tau_nom + tau_robust;

    % Forward dynamics
    qddot = M \ (tau - C*qdot - G);

    % Store data
    q_hist(:,k)       = q;
    qdot_hist(:,k)    = qdot;
    qd_hist(:,k)      = qd;
    qd_dot_hist(:,k)  = qd_dot;
    qd_ddot_hist(:,k) = qd_ddot;
    tau_hist(:,k)     = tau;
    s_hist(:,k)       = s;
    e_hist(:,k)       = e;
    e_norm_hist(k)    = norm(e, 2);

    % Integrate dynamics
    if k < N
        qdot = qdot + dt*qddot;
        q    = q + dt*qdot;
    end
end

%% Save numerical results
save(fullfile(results_dir, 'robust_tracking_results.mat'), ...
    't', 'q_hist', 'qdot_hist', 'qd_hist', 'qd_dot_hist', ...
    'qd_ddot_hist', 'tau_hist', 's_hist', 'e_hist', 'e_norm_hist', ...
    'Lambda', 'Ks', 'rho', 'epsilon', 'q_offset', 'A', 'omega');

%% Plot 1: Desired and actual joint positions
fig1 = figure('Name', 'Robust Tracking - Joint Positions');

for i = 1:4
    subplot(4,1,i);
    plot(t, qd_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t, q_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['q_', num2str(i), ' (rad)']);
    legend('desired', 'actual');
end

xlabel('Time (s)');
sgtitle('Robust Control - Trajectory Tracking: Joint Positions');
drawnow;
saveas(fig1, fullfile(results_dir, 'robust_tracking_positions.png'));

%% Plot 2: Desired and actual joint velocities
fig2 = figure('Name', 'Robust Tracking - Joint Velocities');

for i = 1:4
    subplot(4,1,i);
    plot(t, qd_dot_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t, qdot_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['dq_', num2str(i), ' (rad/s)']);
    legend('desired', 'actual');
end

xlabel('Time (s)');
sgtitle('Robust Control - Trajectory Tracking: Joint Velocities');
drawnow;
saveas(fig2, fullfile(results_dir, 'robust_tracking_velocities.png'));

%% Plot 3: Control torques
fig3 = figure('Name', 'Robust Tracking - Control Torques');

for i = 1:4
    subplot(4,1,i);
    plot(t, tau_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['\tau_', num2str(i), ' (Nm)']);
end

xlabel('Time (s)');
sgtitle('Robust Control - Trajectory Tracking: Control Torques');
drawnow;
saveas(fig3, fullfile(results_dir, 'robust_tracking_torques.png'));

%% Plot 4: Tracking error norm
fig4 = figure('Name', 'Robust Tracking - Error Norm');

plot(t, e_norm_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('||e(t)||_2');
title('Robust Control - Trajectory Tracking: Tracking Error Norm');

drawnow;
saveas(fig4, fullfile(results_dir, 'robust_tracking_error_norm.png'));

%% Plot 5: Sliding variable
fig5 = figure('Name', 'Robust Tracking - Sliding Variable');

for i = 1:4
    subplot(4,1,i);
    plot(t, s_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['s_', num2str(i)]);
end

xlabel('Time (s)');
sgtitle('Robust Control - Trajectory Tracking: Sliding Variable');
drawnow;
saveas(fig5, fullfile(results_dir, 'robust_tracking_sliding_variable.png'));

%% Print summary
fprintf('\nRobust Tracking Simulation Complete.\n');
fprintf('Final joint error norm: %.6f rad\n', e_norm_hist(end));
fprintf('Maximum joint error norm: %.6f rad\n', max(e_norm_hist));
fprintf('Mean joint error norm: %.6f rad\n', mean(e_norm_hist));
fprintf('Results saved in:\n%s\n', results_dir);