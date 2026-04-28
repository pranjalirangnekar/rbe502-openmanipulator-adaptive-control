clc; clear; close all;

%% Robust repo-root detection
% This script lives in project_scripts.
% The repo root is one folder above this script.
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);
addpath(genpath(repo_root));

results_root = fullfile(repo_root, 'project_results');
plot_root = fullfile(results_root, 'final_plots');

if ~exist(plot_root, 'dir')
    mkdir(plot_root);
end

%% Cases to process
cases = {
    'computed_torque_regulation', fullfile(results_root, 'sim_computed_torque_regulation', 'ct_regulation_results.mat'), 'Computed Torque', 'Regulation';
    'computed_torque_tracking',   fullfile(results_root, 'sim_computed_torque_tracking',   'ct_tracking_results.mat'),   'Computed Torque', 'Tracking';
    'robust_regulation',          fullfile(results_root, 'sim_robust_regulation',          'robust_regulation_results.mat'), 'Robust', 'Regulation';
    'robust_tracking',            fullfile(results_root, 'sim_robust_tracking',            'robust_tracking_results.mat'),   'Robust', 'Tracking';
    'adaptive_regulation',        fullfile(results_root, 'sim_adaptive_regulation_safe',   'adaptive_regulation_results.mat'), 'Adaptive', 'Regulation';
    'adaptive_tracking',          fullfile(results_root, 'sim_adaptive_tracking_safe',     'adaptive_tracking_results.mat'),   'Adaptive', 'Tracking';
};

%% Regenerate required plots for every case
for c = 1:size(cases,1)

    case_id = cases{c,1};
    file_path = cases{c,2};
    controller_name = cases{c,3};
    task_name = cases{c,4};

    fprintf('\nProcessing: %s\n', case_id);

    if ~exist(file_path, 'file')
        warning('Missing file: %s', file_path);
        continue;
    end

    S = load(file_path);

    if isfield(S, 't')
        time = S.t;
    elseif isfield(S, 't_plot')
        time = S.t_plot;
    else
        error('No time vector found for %s', case_id);
    end

    out_dir = fullfile(plot_root, case_id);
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    %% Plot 1: joint positions
    fig = figure('Visible','off', 'Name', [case_id '_positions']);
    for i = 1:4
        subplot(4,1,i);
        plot(time, S.qd_hist(i,:), '--', 'LineWidth', 1.4); hold on;
        plot(time, S.q_hist(i,:), 'LineWidth', 1.1);
        grid on;
        ylabel(['q_', num2str(i), ' (rad)']);
        legend('desired', 'actual', 'Location', 'best');
    end
    xlabel('Time (s)');
    sgtitle([controller_name ' Control - ' task_name ': Joint Positions']);
    save_plot(fig, fullfile(out_dir, [case_id '_positions.png']));

    %% Plot 2: joint velocities
    fig = figure('Visible','off', 'Name', [case_id '_velocities']);
    for i = 1:4
        subplot(4,1,i);
        plot(time, S.qd_dot_hist(i,:), '--', 'LineWidth', 1.4); hold on;
        plot(time, S.qdot_hist(i,:), 'LineWidth', 1.1);
        grid on;
        ylabel(['dq_', num2str(i), ' (rad/s)']);
        legend('desired', 'actual', 'Location', 'best');
    end
    xlabel('Time (s)');
    sgtitle([controller_name ' Control - ' task_name ': Joint Velocities']);
    save_plot(fig, fullfile(out_dir, [case_id '_velocities.png']));

    %% Plot 3: control torques
    fig = figure('Visible','off', 'Name', [case_id '_torques']);
    for i = 1:4
        subplot(4,1,i);
        plot(time, S.tau_hist(i,:), 'LineWidth', 1.1);
        grid on;
        ylabel(['\tau_', num2str(i), ' (Nm)']);
    end
    xlabel('Time (s)');
    sgtitle([controller_name ' Control - ' task_name ': Control Torques']);
    save_plot(fig, fullfile(out_dir, [case_id '_torques.png']));

    %% Plot 4: tracking error norm
    fig = figure('Visible','off', 'Name', [case_id '_error_norm']);
    plot(time, S.e_norm_hist, 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('||e(t)||_2 (rad)');
    title([controller_name ' Control - ' task_name ': Tracking Error Norm']);
    save_plot(fig, fullfile(out_dir, [case_id '_error_norm.png']));

    %% Plot 5: adaptive parameter estimates
    if contains(case_id, 'adaptive') && isfield(S, 'theta_hat_hist')

        fig = figure('Visible','off', 'Name', [case_id '_parameters']);
        for i = 1:16
            subplot(4,4,i);
            plot(time, S.theta_hat_hist(i,:), 'LineWidth', 1.1); hold on;
            if isfield(S, 'theta_true')
                yline(S.theta_true(i), '--');
            end
            grid on;
            title(['\theta_', num2str(i)]);
        end
        sgtitle([controller_name ' Control - ' task_name ': Estimated Parameters']);
        save_plot(fig, fullfile(out_dir, [case_id '_parameters.png']));
    end
end

%% Create comparison plots
summary_file = fullfile(results_root, 'summary', 'simulation_summary.csv');

if exist(summary_file, 'file')
    T = readtable(summary_file);

    comparison_dir = fullfile(plot_root, 'comparison');
    if ~exist(comparison_dir, 'dir')
        mkdir(comparison_dir);
    end

    %% Mean error comparison
    fig = figure('Visible','off', 'Name', 'mean_error_comparison');
    bar(T.MeanError);
    grid on;
    ylabel('Mean ||e(t)||_2 (rad)');
    title('Simulation Mean Tracking Error Comparison');
    xticks(1:height(T));
    xticklabels(strcat(T.Controller, " - ", T.Task));
    xtickangle(35);
    save_plot(fig, fullfile(comparison_dir, 'simulation_mean_error_comparison.png'));

    %% Final error comparison
    fig = figure('Visible','off', 'Name', 'final_error_comparison');
    bar(T.FinalError);
    grid on;
    ylabel('Final ||e(t)||_2 (rad)');
    title('Simulation Final Tracking Error Comparison');
    xticks(1:height(T));
    xticklabels(strcat(T.Controller, " - ", T.Task));
    xtickangle(35);
    save_plot(fig, fullfile(comparison_dir, 'simulation_final_error_comparison.png'));

    %% Max error comparison
    fig = figure('Visible','off', 'Name', 'max_error_comparison');
    bar(T.MaxError);
    grid on;
    ylabel('Maximum ||e(t)||_2 (rad)');
    title('Simulation Maximum Tracking Error Comparison');
    xticks(1:height(T));
    xticklabels(strcat(T.Controller, " - ", T.Task));
    xtickangle(35);
    save_plot(fig, fullfile(comparison_dir, 'simulation_max_error_comparison.png'));
end

fprintf('\nAll final simulation plots regenerated in:\n%s\n', plot_root);

%% ============================================================
% Local helper function
% ============================================================
function save_plot(fig, filename)
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