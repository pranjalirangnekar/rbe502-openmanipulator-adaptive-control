clc; clear; close all;

%% ============================================================
%  RBE 502 Final Project
%  Simulation 1: Computed Torque Control - Pose Regulation
%  OpenManipulator-X
% ============================================================

%% Robust repo-root detection
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);
addpath(genpath(repo_root));

results_dir = fullfile(repo_root, 'project_results', 'sim_computed_torque_regulation');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Load identified model parameters
[p, ~, ~, ~] = load_openmanipulator_params();

%% Simulation settings
dt = 0.001;
T  = 8.0;
t  = 0:dt:T;
N  = length(t);

%% Desired pose regulation target
qd_const = [0.25; -0.35; 0.35; 0.15];   % rad

qd_dot_const  = zeros(4,1);
qd_ddot_const = zeros(4,1);

%% Initial state
q    = [0.0; 0.0; 0.0; 0.0];
qdot = zeros(4,1);

%% Controller gains
Kp = diag([25, 25, 20, 10]);
Kd = diag([8,  8,  6,  4]);

%% Storage
q_hist       = zeros(4,N);
qdot_hist    = zeros(4,N);
qd_hist      = zeros(4,N);
qd_dot_hist  = zeros(4,N);
tau_hist     = zeros(4,N);
e_hist       = zeros(4,N);
e_norm_hist  = zeros(1,N);

%% Main simulation loop
for k = 1:N

    % Desired state
    qd      = qd_const;
    qd_dot  = qd_dot_const;
    qd_ddot = qd_ddot_const;

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
    q_hist(:,k)      = q;
    qdot_hist(:,k)   = qdot;
    qd_hist(:,k)     = qd;
    qd_dot_hist(:,k) = qd_dot;
    tau_hist(:,k)    = tau;
    e_hist(:,k)      = e;
    e_norm_hist(k)   = norm(e, 2);

    % Integrate dynamics
    if k < N
        qdot = qdot + dt*qddot;
        q    = q + dt*qdot;
    end
end

%% Save numerical results
save(fullfile(results_dir, 'ct_regulation_results.mat'), ...
    't', 'q_hist', 'qdot_hist', 'qd_hist', 'qd_dot_hist', ...
    'tau_hist', 'e_hist', 'e_norm_hist', 'Kp', 'Kd', 'qd_const');

%% Plot 1: Desired and actual joint positions
fig1 = figure('Visible','off', 'Name', 'Computed Torque Regulation - Joint Positions');

for i = 1:4
    subplot(4,1,i);
    plot(t, qd_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t, q_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['q_', num2str(i), ' (rad)']);
    legend('desired', 'actual', 'Location', 'best');
end

xlabel('Time (s)');
sgtitle('Computed Torque Control - Pose Regulation: Joint Positions');
save_plot_local(fig1, fullfile(results_dir, 'ct_regulation_positions.png'));

%% Plot 2: Desired and actual joint velocities
fig2 = figure('Visible','off', 'Name', 'Computed Torque Regulation - Joint Velocities');

for i = 1:4
    subplot(4,1,i);
    plot(t, qd_dot_hist(i,:), '--', 'LineWidth', 1.5); hold on;
    plot(t, qdot_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['dq_', num2str(i), ' (rad/s)']);
    legend('desired', 'actual', 'Location', 'best');
end

xlabel('Time (s)');
sgtitle('Computed Torque Control - Pose Regulation: Joint Velocities');
save_plot_local(fig2, fullfile(results_dir, 'ct_regulation_velocities.png'));

%% Plot 3: Control torques
fig3 = figure('Visible','off', 'Name', 'Computed Torque Regulation - Control Torques');

for i = 1:4
    subplot(4,1,i);
    plot(t, tau_hist(i,:), 'LineWidth', 1.2);
    grid on;
    ylabel(['\tau_', num2str(i), ' (Nm)']);
end

xlabel('Time (s)');
sgtitle('Computed Torque Control - Pose Regulation: Control Torques');
save_plot_local(fig3, fullfile(results_dir, 'ct_regulation_torques.png'));

%% Plot 4: Tracking error norm
fig4 = figure('Visible','off', 'Name', 'Computed Torque Regulation - Error Norm');

plot(t, e_norm_hist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('||e(t)||_2 (rad)');
title('Computed Torque Control - Pose Regulation: Tracking Error Norm');

save_plot_local(fig4, fullfile(results_dir, 'ct_regulation_error_norm.png'));

%% Print summary
fprintf('\nComputed Torque Regulation Simulation Complete.\n');
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