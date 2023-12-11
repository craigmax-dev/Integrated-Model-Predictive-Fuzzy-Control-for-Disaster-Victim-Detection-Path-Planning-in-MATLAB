% Function created for environment model refactor

% V2.3 performance improvements refactor
function [m_f, m_bt, F] = updateFireStatesAndProbabilities(m_f, m_bt, F, W, dt_e, t_i, t_b, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w)
    % Update burn time for active and burning states
    m_bt(m_f == 2 | m_f == 3) = m_bt(m_f == 2 | m_f == 3) + dt_e;

    % Transition from active to burning state
    m_f(m_f == 2 & m_bt >= t_i) = 3;

    % Update fire spread probability for burning cells
    burning_cells = find(m_f == 3);
    for idx = 1:length(burning_cells)
        [i, j] = ind2sub([n_x_e, n_y_e], burning_cells(idx));
        p = calculateSpreadProbability(m_bt(i, j), t_i, t_b);
        F = updateFireSpreadProbability(F, W, p, i, j, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w);
    end

    % Transition from burning to burnout state
    m_f(m_f == 3 & m_bt >= t_b) = 4;
end

% % V2.2 refactored with c_fs_2
% function [m_f, m_bt, F] = updateFireStatesAndProbabilities(m_f, m_bt, F, W, dt_e, t_i, t_b, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w)
%     % Iterate over each cell in the fire map
%     for i = 1:n_x_e
%         for j = 1:n_y_e
%             % Check and update the fire state based on the model's rules
%             if m_f(i, j) == 2  % Active fire state
%                 m_bt(i, j) = m_bt(i, j) + dt_e;
%                 % Advance ignition to combustion
%                 if m_bt(i, j) >= t_i
%                     m_f(i, j) = 3;
%                 end
%             elseif m_f(i, j) == 3  % Burning state
%                 m_bt(i, j) = m_bt(i, j) + dt_e;
%                 % Calculate fire spread probability 'p'
%                 p = calculateSpreadProbability(m_bt(i, j), t_i, t_b);
%                 % Update fire spread probability around the burning cell
%                 F = updateFireSpreadProbability(F, W, p, i, j, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w);
%                 % Advance combustion to burnout
%                 if m_bt(i, j) >= t_b
%                     m_f(i, j) = 4;
%                 end
%             end
%         end
%     end
% end

% % V2.1
% function [m_f, m_bt, F] = updateFireStatesAndProbabilities(m_f, m_bt, F, W, dt_e, t_i, t_b, m_s, m_bo, c_fs_1)
%     n_x_e = size(m_f, 1);
%     n_y_e = size(m_f, 2);
% 
%     for i = 1:n_x_e
%         for j = 1:n_y_e
%             if m_f(i, j) == 2  % Active fire state
%                 m_bt(i, j) = m_bt(i, j) + dt_e;
%                 if m_bt(i, j) >= t_i
%                     m_f(i, j) = 3;  % Transition to burning state
%                 end
%             elseif m_f(i, j) == 3  % Burning state
%                 m_bt(i, j) = m_bt(i, j) + dt_e;
%                 p = calculateFireSpreadRate(m_bt(i, j), t_i, t_b);
%                 F(i, j) = c_fs_1 * (m_s(i, j) * m_bo(i, j)) * sum(W(:)) * p;
%                 if m_bt(i, j) >= t_b
%                     m_f(i, j) = 4;  % Transition to burnout state
%                 end
%             end
%         end
%     end
% end
