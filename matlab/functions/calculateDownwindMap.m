function m_dw_e = calculateDownwindMap(m_f, n_x_e, n_y_e, c_wm_1, c_wm_2, ang_w, v_w)

    % Initialize m_dw_e with zeros
    m_dw_e = zeros(n_x_e, n_y_e);
    
    for i = 1:n_x_e
        for j = 1:n_y_e
            if m_f(i, j) == 2 || m_f(i, j) == 3  % Active or burning fire
                [W_dir_ws, W_dis_ws] = calculateLocalWindMap(i, j, n_x_e, n_y_e, ang_w, v_w, c_wm_1, c_wm_2);
                m_dw_e = max(m_dw_e, W_dir_ws .* W_dis_ws);
            end
        end
    end

    m_dw_e = 1 - m_dw_e;  % Invert the downwind effect
end