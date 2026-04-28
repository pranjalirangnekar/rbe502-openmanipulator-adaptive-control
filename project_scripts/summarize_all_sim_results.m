clc; clear; close all;

%% ============================================================
%  RBE 502 Final Project
%  Summary of all simulation results
% ============================================================

%% Robust repo-root detection
script_path = mfilename('fullpath');
script_dir  = fileparts(script_path);
repo_root   = fileparts(script_dir);

cd(repo_root);
addpath(genpath(repo_root));

results_root = fullfile(repo_root, 'project_results');

summary_dir = fullfile(results_root, 'summary');
if ~exist(summary_dir, 'dir')
    mkdir(summary_dir);
end

%% Helper function
get_metrics = @(e) struct( ...
    'final_error', e(end), ...
    'max_error', max(e), ...
    'mean_error', mean(e));

%% Load all result files
ct_reg = load(fullfile(results_root, 'sim_computed_torque_regulation', 'ct_regulation_results.mat'));
ct_trk = load(fullfile(results_root, 'sim_computed_torque_tracking', 'ct_tracking_results.mat'));

rb_reg = load(fullfile(results_root, 'sim_robust_regulation', 'robust_regulation_results.mat'));
rb_trk = load(fullfile(results_root, 'sim_robust_tracking', 'robust_tracking_results.mat'));

ad_reg = load(fullfile(results_root, 'sim_adaptive_regulation_safe', 'adaptive_regulation_results.mat'));
ad_trk = load(fullfile(results_root, 'sim_adaptive_tracking_safe', 'adaptive_tracking_results.mat'));

%% Compute metrics
m_ct_reg = get_metrics(ct_reg.e_norm_hist);
m_ct_trk = get_metrics(ct_trk.e_norm_hist);

m_rb_reg = get_metrics(rb_reg.e_norm_hist);
m_rb_trk = get_metrics(rb_trk.e_norm_hist);

m_ad_reg = get_metrics(ad_reg.e_norm_hist);
m_ad_trk = get_metrics(ad_trk.e_norm_hist);

%% Build summary table
Controller = {
    'Computed Torque'
    'Computed Torque'
    'Robust'
    'Robust'
    'Adaptive'
    'Adaptive'
    };

Task = {
    'Regulation'
    'Tracking'
    'Regulation'
    'Tracking'
    'Regulation'
    'Tracking'
    };

FinalError = [
    m_ct_reg.final_error
    m_ct_trk.final_error
    m_rb_reg.final_error
    m_rb_trk.final_error
    m_ad_reg.final_error
    m_ad_trk.final_error
    ];

MaxError = [
    m_ct_reg.max_error
    m_ct_trk.max_error
    m_rb_reg.max_error
    m_rb_trk.max_error
    m_ad_reg.max_error
    m_ad_trk.max_error
    ];

MeanError = [
    m_ct_reg.mean_error
    m_ct_trk.mean_error
    m_rb_reg.mean_error
    m_rb_trk.mean_error
    m_ad_reg.mean_error
    m_ad_trk.mean_error
    ];

T = table(Controller, Task, FinalError, MaxError, MeanError);

disp('===========================================================');
disp('Simulation Summary Table');
disp('===========================================================');
disp(T);

%% Save table and data
save(fullfile(summary_dir, 'simulation_summary.mat'), 'T');
writetable(T, fullfile(summary_dir, 'simulation_summary.csv'));

fprintf('\nSummary saved to:\n%s\n', summary_dir);