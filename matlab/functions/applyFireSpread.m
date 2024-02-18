% V2

% CHANGELOG
% Removed m_f_hist

function m_f = applyFireSpread(m_f, F)

    n_x_e = size(m_f, 1);
    n_y_e = size(m_f, 2);

    for i = 1:n_x_e
        for j = 1:n_y_e
            if m_f(i, j) == 1 && rand <= F(i, j)
                m_f(i, j) = 2;  % Ignition occurs
            end
        end
    end
end
