clc; clear; close all;

%% ============================================================
%  RBE 502 Final Project
%  Check simulation output completeness
% ============================================================

%% Robust repo-root detection
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);

results_root = fullfile(repo_root, 'project_results');
plot_root    = fullfile(results_root, 'final_plots');

required_result_files = {
    fullfile(results_root, 'sim_computed_torque_regulation', 'ct_regulation_results.mat')
    fullfile(results_root, 'sim_computed_torque_tracking', 'ct_tracking_results.mat')
    fullfile(results_root, 'sim_robust_regulation', 'robust_regulation_results.mat')
    fullfile(results_root, 'sim_robust_tracking', 'robust_tracking_results.mat')
    fullfile(results_root, 'sim_adaptive_regulation_safe', 'adaptive_regulation_results.mat')
    fullfile(results_root, 'sim_adaptive_tracking_safe', 'adaptive_tracking_results.mat')
    fullfile(results_root, 'summary', 'simulation_summary.csv')
};

required_plot_folders = {
    'computed_torque_regulation', 4
    'computed_torque_tracking',   4
    'robust_regulation',          4
    'robust_tracking',            4
    'adaptive_regulation',        5
    'adaptive_tracking',          5
    'comparison',                 3
};

fprintf('\n================ Simulation Output Check ================\n');

%% Check numerical result files
fprintf('\nChecking result files:\n');

for i = 1:length(required_result_files)
    if exist(required_result_files{i}, 'file')
        fprintf('[OK]      %s\n', required_result_files{i});
    else
        fprintf('[MISSING] %s\n', required_result_files{i});
    end
end

%% Check final plot folders
fprintf('\nChecking final plot folders:\n');

for i = 1:size(required_plot_folders,1)

    folder_name = required_plot_folders{i,1};
    expected_n  = required_plot_folders{i,2};

    folder_path = fullfile(plot_root, folder_name);
    pngs = dir(fullfile(folder_path, '*.png'));

    if length(pngs) >= expected_n
        fprintf('[OK]      %s : %d png files\n', folder_name, length(pngs));
    else
        fprintf('[CHECK]   %s : %d png files, expected at least %d\n', ...
            folder_name, length(pngs), expected_n);
    end
end

fprintf('\nCheck complete.\n');
fprintf('Repo root checked:\n%s\n', repo_root);