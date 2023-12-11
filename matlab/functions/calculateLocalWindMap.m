function [W_dir_ws, W_dis_ws] = calculateLocalWindMap(i, j, n_x_e, n_y_e, ang_w, v_w, c_wm_1, c_wm_2)
    [X, Y] = meshgrid(1:n_x_e, 1:n_y_e);
    f_d = atan2(Y - i, X - j);
    ang = ang_w - f_d;
    W_dir_ws = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang) - 1)));
    W_dis_ws = 1 - sqrt((X - i).^2 + (Y - j).^2) / sqrt(n_x_e^2 + n_y_e^2);
    W_dir_ws = mat2gray(W_dir_ws);  % Normalize wind direction influence
end