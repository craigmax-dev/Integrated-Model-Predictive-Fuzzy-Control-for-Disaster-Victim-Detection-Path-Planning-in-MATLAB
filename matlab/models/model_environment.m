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

% TODO 
% Check indexing is done correctly: (:, :, k_e + 1) and (:, :, k_e)

%% Model of fire spread using cellular automa

% V2.2
function environment_model = model_environment(environment_model, k_e, dt_e)
    % Adjust k_e for MATLAB's 1-based indexing
    matlab_index = k_e + 1;

    if any(environment_model.m_f(:) == 2) || any(environment_model.m_f(:) == 3)
        W = calculateWindSpreadMatrix(environment_model.r_w, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.c_wm_d, environment_model.ang_w, environment_model.v_w);
        
        % Update fire map states and calculate fire spread probabilities
        [environment_model.m_bt, environment_model.m_f, F] = updateFireStatesAndProbabilities(environment_model, W, dt_e);

        % Determine if fire spread occurs and apply fire spread
        environment_model.m_f = applyFireSpread(environment_model.m_f, F);
        
        % Calculate downwind map based on the new fire map
        environment_model.m_dw_e = calculateDownwindMap(environment_model.m_f, environment_model.n_x_e, environment_model.n_y_e, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.ang_w, environment_model.v_w);

        % Store the updated states in the series at the adjusted MATLAB index
        environment_model.m_f_series(:, :, matlab_index) = environment_model.m_f;
        environment_model.m_dw_e_series(:, :, matlab_index) = environment_model.m_dw_e;
    else
        % For the first step or if no change, copy the previous state (if not the first step)
        if matlab_index > 1
            environment_model.m_f_series(:, :, matlab_index) = environment_model.m_f_series(:, :, matlab_index - 1);
            environment_model.m_dw_e_series(:, :, matlab_index) = environment_model.m_dw_e_series(:, :, matlab_index - 1);
        end
    end

    % % Update the current state in environment_model for continuity
    % environment_model.m_f = m_f;
    % environment_model.m_dw_e = m_dw_e;
end

% % V2
% function environment_model = model_environment(environment_model, k_e, dt_e)  
% 
%   if any(environment_model.m_f(:) == 2) || any(environment_model.m_f(:) == 3)
% 
%     W = calculateWindSpreadMatrix(environment_model.r_w, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.c_wm_d, environment_model.ang_w, environment_model.v_w);
% 
%     % Update fire map states and calculate fire spread probabilities
%     [environment_model, F] = updateFireStatesAndProbabilities(environment_model, W, dt_e);
% 
%     % Determine if fire spread occurs
%     environment_model.m_f = applyFireSpread(environment_model.m_f, F);
% 
%     % Calculate downwind map
%     environment_model.m_dw_e = calculateDownwindMap(environment_model.m_f, environment_model.n_x_e, environment_model.n_y_e, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.ang_w, environment_model.v_w);
% 
%   else
% 
%     % No change from previous step
%     environment_model.m_dw_e = environment_model.m_dw_e;
% 
%   end
% 
% end