
% V2.2 - performance improvements
function W = calculateWindSpreadMatrix(r_w, c_wm_1, c_wm_2, c_wm_d, ang_w, v_w)
    % Create grid for wind influence
    [X, Y] = meshgrid(-r_w:r_w, -r_w:r_w);

    % Calculate angle difference between wind direction and cell direction
    F_d = atan2(Y, X);
    ang_diff = ang_w - F_d;

    % Calculate wind direction modifier using vectorized operations
    w_dir = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang_diff) - 1)));

    % Calculate wind distance modifier using vectorized operations
    distances = sqrt(X.^2 + Y.^2);
    w_dis = c_wm_d .^ distances;

    % Combine direction and distance modifiers
    W = w_dir .* w_dis;
end