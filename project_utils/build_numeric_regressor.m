function Y = build_numeric_regressor(q, qdot, qdot_r, qddot_r, geom_params, g_val)
% Numerically builds the regressor matrix Y for the 16 dynamic parameters
%
% We keep geometry and gravity fixed, and activate one dynamic parameter
% at a time to form each column of Y.

    n_params = 16;
    Y = zeros(4, n_params);

    for j = 1:n_params
        p_basis = zeros(23,1);

        % fixed geometry
        p_basis(1:6) = geom_params;

        % activate one dynamic parameter
        p_basis(6 + j) = 1.0;

        % fixed gravity
        p_basis(23) = g_val;

        % corresponding regressor column
        Y(:,j) = M_fun(q, p_basis) * qddot_r + ...
                 C_fun(q, qdot, p_basis) * qdot_r + ...
                 G_fun(q, p_basis);
    end
end