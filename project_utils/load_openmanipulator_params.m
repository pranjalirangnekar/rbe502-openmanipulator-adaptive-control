function [p_true, theta_true, geom_params, g_val] = load_openmanipulator_params()
% Loads the identified OpenManipulator-X model parameters
% p_true     : full parameter vector used by M_fun, C_fun, G_fun
% theta_true : 16 dynamic parameters to be adapted
% geom_params: first 6 geometric parameters
% g_val      : gravity scalar

    R = load(fullfile('Identification', 'identification_result.mat'));
    x_opt_vec = R.x_opt_vec(:);

    p_true = [R.p(1:6); ...
              x_opt_vec(1); x_opt_vec(2); x_opt_vec(3); x_opt_vec(4); ...
              x_opt_vec(5); x_opt_vec(6); x_opt_vec(7); ...
              x_opt_vec(8); x_opt_vec(9); x_opt_vec(10); ...
              x_opt_vec(11); x_opt_vec(12); x_opt_vec(13); ...
              x_opt_vec(14); x_opt_vec(15); x_opt_vec(16); ...
              R.id_info.g];

    geom_params = p_true(1:6);
    theta_true  = p_true(7:22);
    g_val       = p_true(23);
end