% Function created for environment model refactor

function [m_f, m_bt, F] = updateFireStatesAndProbabilities(m_f, m_bt, F, W, dt_e, t_i, t_b, m_s, m_bo, c_fs_1)
    n_x_e = size(m_f, 1);
    n_y_e = size(m_f, 2);

    for i = 1:n_x_e
        for j = 1:n_y_e
            if m_f(i, j) == 2  % Active fire state
                m_bt(i, j) = m_bt(i, j) + dt_e;
                if m_bt(i, j) >= t_i
                    m_f(i, j) = 3;  % Transition to burning state
                end
            elseif m_f(i, j) == 3  % Burning state
                m_bt(i, j) = m_bt(i, j) + dt_e;
                p = calculateFireSpreadRate(m_bt(i, j), t_i, t_b);
                F(i, j) = c_fs_1 * (m_s(i, j) * m_bo(i, j)) * sum(W(:)) * p;
                if m_bt(i, j) >= t_b
                    m_f(i, j) = 4;  % Transition to burnout state
                end
            end
        end
    end
end
