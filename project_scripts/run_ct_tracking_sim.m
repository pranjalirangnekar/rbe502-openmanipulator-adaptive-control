clc; clear; close all;

%% ============================================================
%  RBE 502 Final Project
%  Simulation 2: Computed Torque Control - Trajectory Tracking
%  OpenManipulator-X
% ============================================================

%% Robust repo-root detection
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);
addpath(genpath(repo_root));

results_dir = fullfile(repo_root, 'project_results', 'sim_computed_torque_tracking');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Load identified model parameters
[p, ~, ~, ~] = load_openmanipulator_params();

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

%% Controller gains
Kp = diag([30, 30, 25, 15]);
Kd = diag([10, 10, 8,  5]);

%% Storage
q_hist       = zeros(4,N);
qdot_hist    = zeros(4,N);
qd_hist      = zeros(4,N);
qd_dot_hist  = zeros(4,N);
qd_ddot_hist = zeros(4,N);
tau_hist     = zeros(4,N);
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

    % Computed torque auxiliary input
    v = qd_ddot + Kd*edot + Kp*e;

    % Robot dynamics
    M = M_fun(q, p);
    C = C_fun(q, qdot, p);
    G = G_fun(q, p);

    % Computed torque control law
    tau = M*v + C*qdot + G;

    % Forward dynamics
    qddot = M \ (tau - C*qdot - G);

    % Store data
    q_hist(:,k)       = q;
    qdot_hist(:,k)    = qdot;
    qd_hist(:,k)      = qd;
    qd_dot_hist(:,k)  = qd_dot;
    qd_ddot_hist(:,k) = qd_ddot;
    tau_hist(:,k)     = tau;
    e_hist(:,k)       = e;
    e_norm_hist(k)    = norm(e, 2);

    % Integrate dynamics
    if k < N
        qdot = qdot + dt*qddot;
        q    = q + dt*qdot;
    end
end

%% Save numerical results
save(fullfile(results_dir, 'ct_tracking_results.mat'), ...
    't', 'q_hist', 'qdot_hist', 'qd_hist', 'qd_dot_hist', ...
    'qd_ddot_hist', 'tau_hist', 'e_hist', 'e_norm_hist', ...
    'Kp', 'Kd', 'q_offset', 'A', 'omega');

%% Plot 1: Desired and actual joint positions
fig1 = figure('Visible','off', 'Name', 'Computed Torque Tracking - Joint Positions');

for i = 1:4
    subplot(4,1,i);
    plot(t, qd_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t, q_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['q_', num2str(i), ' (rad)']);
    legend('desired', 'actual', 'Location', 'best');
end

xlabel('Time (s)');
sgtitle('Computed Torque Control - Trajectory Tracking: Joint Positions');
save_plot_local(fig1, fullfile(results_dir, 'ct_tracking_positions.png'));

%% Plot 2: Desired and actual joint velocities
fig2 = figure('Visible','off', 'Name', 'Computed Torque Tracking - Joint Velocities');

for i = 1:4
    subplot(4,1,i);
    plot(t, qd_dot_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t, qdot_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['dq_', num2str(i), ' (rad/s)']);
    legend('desired', 'actual', 'Location', 'best');
end

xlabel('Time (s)');
sgtitle('Computed Torque Control - Trajectory Tracking: Joint Velocities');
save_plot_local(fig2, fullfile(results_dir, 'ct_tracking_velocities.png'));

%% Plot 3: Control torques
fig3 = figure('Visible','off', 'Name', 'Computed Torque Tracking - Control Torques');

for i = 1:4
    subplot(4,1,i);
    plot(t, tau_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['\tau_', num2str(i), ' (Nm)']);
end

xlabel('Time (s)');
sgtitle('Computed Torque Control - Trajectory Tracking: Control Torques');
save_plot_local(fig3, fullfile(results_dir, 'ct_tracking_torques.png'));

%% Plot 4: Tracking error norm
fig4 = figure('Visible','off', 'Name', 'Computed Torque Tracking - Error Norm');

plot(t, e_norm_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('||e(t)||_2 (rad)');
title('Computed Torque Control - Trajectory Tracking: Tracking Error Norm');

save_plot_local(fig4, fullfile(results_dir, 'ct_tracking_error_norm.png'));

%% Print summary
fprintf('\nComputed Torque Tracking Simulation Complete.\n');
fprintf('Final joint error norm: %.8f rad\n', e_norm_hist(end));
fprintf('Maximum joint error norm: %.8f rad\n', max(e_norm_hist));
fprintf('Mean joint error norm: %.8f rad\n', mean(e_norm_hist));
fprintf('Results saved in:\n%s\n', results_dir);

%% ============================================================
% Local helper function
% ============================================================
function save_plot_local(fig, filename)
    drawnow;

    try
        exportgraphics(fig, filename, 'Resolution', 300);
    catch
        try
            print(fig, filename, '-dpng', '-r300');
        catch
            saveas(fig, filename);
        end
    end

    close(fig);
end