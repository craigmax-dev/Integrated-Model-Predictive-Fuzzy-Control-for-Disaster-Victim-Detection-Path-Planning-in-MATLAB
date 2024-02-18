% Function created for environment model refactor

% CHANGELOG
% Refactor: environment forecast refactor

% % V2.4 refactor for environment prediction refactor
function [m_bt, m_f, F] = updateFireStatesAndProbabilities(environment_model, W, dt_e)
    % Initialize F
    F = zeros(environment_model.n_x_e, environment_model.n_y_e);  % Fire spread probability map
    
    % Update burn time for active and burning states
    m_bt = environment_model.m_bt; % Copy burn time matrix for direct manipulation
    m_bt(environment_model.m_f == 2 | environment_model.m_f == 3) = m_bt(environment_model.m_f == 2 | environment_model.m_f == 3) + dt_e;

    % Copy fire state matrix for direct manipulation
    m_f = environment_model.m_f;

    % Transition from active to burning state
    m_f(m_f == 2 & m_bt >= environment_model.t_i) = 3;

    % Update fire spread probability for burning cells
    burning_cells = find(m_f == 3);
    for idx = 1:length(burning_cells)
        [i, j] = ind2sub([environment_model.n_x_e, environment_model.n_y_e], burning_cells(idx));
        p = calculateSpreadProbability(m_bt(i, j), environment_model.t_i, environment_model.t_b);
        F = updateFireSpreadProbability(F, W, p, i, j, environment_model.c_fs_1, environment_model.c_fs_2, environment_model.m_s, environment_model.m_bo, environment_model.n_x_e, environment_model.n_y_e, environment_model.r_w);
    end

    % Transition from burning to burnout state
    m_f(m_f == 3 & m_bt >= environment_model.t_b) = 4;
end

% % V2.3 performance improvements refactor
% function [environment_model, F] = updateFireStatesAndProbabilities(environment_model, W, dt_e)
% 
%   % Initialize F
%   F = zeros(environment_model.n_x_e, environment_model.n_y_e);  % Fire spread probability map
% 
%   % Update burn time for active and burning states
%   environment_model.m_bt(environment_model.m_f == 2 | environment_model.m_f == 3) = environment_model.m_bt(environment_model.m_f == 2 | environment_model.m_f == 3) + dt_e;
% 
%   % Transition from active to burning state
%   environment_model.m_f(environment_model.m_f == 2 & environment_model.m_bt >= environment_model.t_i) = 3;
% 
%   % Update fire spread probability for burning cells
%   burning_cells = find(environment_model.m_f == 3);
%   for idx = 1:length(burning_cells)
%       [i, j] = ind2sub([environment_model.n_x_e, environment_model.n_y_e], burning_cells(idx));
%       p = calculateSpreadProbability(environment_model.m_bt(i, j), environment_model.t_i, environment_model.t_b);
%       F = updateFireSpreadProbability(F, W, p, i, j, environment_model.c_fs_1, environment_model.c_fs_2, environment_model.m_s, environment_model.m_bo, environment_model.n_x_e, environment_model.n_y_e, environment_model.r_w);
%   end
% 
%   % Transition from burning to burnout state
%   environment_model.m_f(environment_model.m_f == 3 & environment_model.m_bt >= environment_model.t_b) = 4;
% end
