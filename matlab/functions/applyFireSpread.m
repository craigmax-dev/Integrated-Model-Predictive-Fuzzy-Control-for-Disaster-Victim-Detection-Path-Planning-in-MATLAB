function m_f = applyFireSpread(m_f, F, flag_mpc, k)

    n_x_e = size(m_f, 1);
    n_y_e = size(m_f, 2);

    for i = 1:n_x_e
        for j = 1:n_y_e
            if m_f(i, j) == 1 && rand <= F(i, j)
                m_f(i, j) = 2;  % Ignition occurs
                if ~flag_mpc
                    m_f_hist(i, j) = k;  % Record fire history if not prediction
                end
            end
        end
    end
end
