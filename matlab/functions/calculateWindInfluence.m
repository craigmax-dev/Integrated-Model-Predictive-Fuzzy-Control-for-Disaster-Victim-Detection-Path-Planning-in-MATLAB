function [W_dir, W_dis] = calculateWindInfluence(config, input)
    % Initialize wind matrices
    W_dir = zeros(input.n_x_e, input.n_y_e);
    W_dis = zeros(input.n_x_e, input.n_y_e);

    % Wind parameters
    r_w = config.wind_radius;  % Radius for wind influence
    c_wm_1 = config.c_wm_1;    % Wind coefficient 1
    c_wm_2 = config.c_wm_2;    % Wind coefficient 2

    % Precompute wind influence matrices
    [X, Y] = meshgrid(-r_w:r_w, -r_w:r_w);
    F_d = atan2(Y, X);
    Ang = config.ang_w - F_d;
    W_dir = exp(config.v_w * (c_wm_1 + c_wm_2 * (cos(Ang) - 1)));
    W_dis = c_wm_2.^sqrt(X.^2 + Y.^2);

    % Normalization of wind direction matrix
    W_dir = W_dir / max(W_dir(:));
end
