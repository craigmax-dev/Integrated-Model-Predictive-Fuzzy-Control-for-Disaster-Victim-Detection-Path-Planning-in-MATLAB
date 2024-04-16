%% Function environmentModel.m
% Description:  Fire state model for the disaster area. Based on article by
% Ohgai, Gohnai, and Watanabe: "Cellular automata modeling of fire spread in
% built-up areas—A tool to aid community-based
% planning for disaster mitigation"

% Inputs:   environment_model.m_f - fire map - tracks states of fire in each cell. 
%               cell states:
%               0 - unburnable
%               1 - not burning yet - able to burn
%               2 - catching fire   - not able to spread
%               3 - burning         - able to spread
%               4 - extinguished
%           w_s - wind speed
%           w_d - wind direction - 0, pi/2, pi, 3*pi/2
%           environment_model.m_s - map of flammability of map area.
%               cell states:
%               1 - wooden
%               0.6 - fire prevention wooden
%               0 - fireproof
%           m_p - ratio of cell area occupied by building
%           gridSize - length of grid cells (m)
%           a - Coefficient to tune degree of slowdown in fire spread
%           b - Coefficient to tune range and direction of spread
%           c_f - coarsen factor
% Author:   Craig Maxwell
% Date:     28/02/2020

%% Model notes
% Fire in cell burns for fixed time t_burn
% Fire has probability of spreading according to windMap and state of fire.
% Random number generator initialised with string and timestep - therefore
% changes with every timestep
% Structure states - open space, wooden, fire-prevention wooden, fireproof
% m_p - set using indoor temperature standard curve of wooden house on fire
% t_i and t_b are estimated for a grid size of 9m^2 - so set fire map cell
% size to 3m.
% For non-representative use cases we can ignore this assumption.
% Ang - angle between wind direction and the fire propagation

% From another paper - wind model effect - if we have a good model for
% generating wind maps then we may be able to calculate spread time better.

%% Research
% Cellular automa model - https://www.researchgate.net/publication/256662471_A_Cellular_Automata_Model_for_Fire_Spreading_Prediction
% Physics-based fire spread model - http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.157.3638&rep=rep1&type=pdf

% V2
% CHANGELOG
% Refactor: split into smaller functions
% Refactor: performance improvements
% Refactor: will only run environment model with fire states =2 or =3 present
% Refactor: environment_model structures

%% Model of fire spread using cellular automa

% V2.2
function [environment_model] = model_environment(environment_model, k_e, dt_e)
    % Adjust k_e for MATLAB's 1-based indexing
    i = k_e + 1;

    if any(environment_model.m_f(:) == 2) || any(environment_model.m_f(:) == 3)
        W = calculateWindSpreadMatrix(environment_model.r_w, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.c_wm_d, environment_model.ang_w, environment_model.v_w);
        
        % Update fire map states and calculate fire spread probabilities
        [environment_model.m_bt, environment_model.m_f, F] = updateFireStatesAndProbabilities(environment_model, W, dt_e);

        % Determine if fire spread occurs and apply fire spread
        environment_model.m_f = applyFireSpread(environment_model.m_f, F);
        
        % Calculate downwind map based on the new fire map
        environment_model.m_dw_e = calculateDownwindMap(environment_model.m_f, environment_model.n_x_e, environment_model.n_y_e, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.ang_w, environment_model.v_w);

        % Store the updated states in the series at the adjusted MATLAB index
        environment_model.m_f_series(:, :, i) = environment_model.m_f;
        environment_model.m_dw_e_series(:, :, i) = environment_model.m_dw_e;
    else
        % For the first step or if no change, copy the previous state (if not the first step)
        if i > 1
            environment_model.m_f_series(:, :, i) = environment_model.m_f_series(:, :, i - 1);
            environment_model.m_dw_e_series(:, :, i) = environment_model.m_dw_e_series(:, :, i - 1);
        end
    end

end

function [m_bt, m_f, F] = updateFireStatesAndProbabilities(environment_model, W, dt_e)
    
    % Initialize F with the correct dimensions
    F = zeros(environment_model.n_x_e, environment_model.n_y_e);
    
    % Direct manipulation of m_bt and m_f
    m_bt = environment_model.m_bt;
    m_f = environment_model.m_f;
    
    % Logical indexing and update
    active_or_burning = (m_f == 2) | (m_f == 3);
    m_bt(active_or_burning) = m_bt(active_or_burning) + dt_e;  % Correctly increments the burn time for active or burning cells by dt_e

    % Transition from active to burning state based on ignition time
    m_f(m_f == 2 & m_bt >= environment_model.t_i) = 3;

    % Update fire spread probability for burning cells and transition to burnout state
    burning_cells = find(m_f == 3);
    for idx = 1:length(burning_cells)
        [i, j] = ind2sub([environment_model.n_x_e, environment_model.n_y_e], burning_cells(idx));
        p = calculateSpreadProbability(m_bt(i, j), environment_model.t_i, environment_model.t_b);
        F = updateFireSpreadProbability(F, W, p, i, j, environment_model.c_fs_1, environment_model.c_fs_2, environment_model.m_s, environment_model.m_bo, environment_model.n_x_e, environment_model.n_y_e, environment_model.r_w);
    end

    % Transition from burning to burnout state based on burnout time
    m_f(m_f == 3 & m_bt >= environment_model.t_b) = 4;
    
end

function p = calculateSpreadProbability(t_ckl, t_i, t_b)
    % Calculation of fire spread probability based on burn time
    if t_ckl <= (t_b - t_i) / 5 + t_i
        p = 4 / (t_b - t_i) * t_ckl + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
    elseif t_ckl <= t_b
        p = 5 / (4 * (t_b - t_i)) * (-t_ckl + t_b);
    else
        p = 0; % No spread if outside of time range
    end
end

% V2.2 performance improvements refactor
function F = updateFireSpreadProbability(F, W, p, i, j, c_fs_1, c_fs_2, m_s, m_bo, n_x_e, n_y_e, r_w)
    % Determine the range for local update
    row_range = max(1, i - r_w) : min(n_x_e, i + r_w);
    col_range = max(1, j - r_w) : min(n_y_e, j + r_w);

    % Adjust indices in W to align with F's indices
    W_row_range = (row_range - i + r_w + 1);
    W_col_range = (col_range - j + r_w + 1);

    % Update the fire spread probability
    local_W = W(W_row_range, W_col_range);
    F(row_range, col_range) = F(row_range, col_range) + ...
                              c_fs_1 * (m_s(row_range, col_range) .* m_bo(row_range, col_range)) .* ...
                              local_W.^c_fs_2 * p;
end

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
