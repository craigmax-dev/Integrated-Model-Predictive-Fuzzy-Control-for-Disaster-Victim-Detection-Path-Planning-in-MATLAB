% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% CHANGELOG
% Rename: to model_fis 
% Bugfix: Coarsen downwind map
% Removed test_fis_sensitivity feature
% Refactor: environment_model and agent_model structures
% Feature: Battery model refactor
% Feature: Local maps

% V2.5 Local Map Refactor
% NOTE: Replaced with model_fis_global and model_fis_local
function agent_model = model_fis(agent_model, ang_w, v_w, config, fisArray)
    % Initialize grid size and agent positions
    gridSize = [agent_model.n_x_s, agent_model.n_y_s];
    agentPositions = agent_model.a_loc; % Assuming a_loc stores the (x, y) positions of agents

    % Determine if 'r_nextagent' input is required by checking the first FIS (assuming all agents have the same FIS inputs)
    inputNames = [fisArray(1).Inputs.Name];  % Concatenate into a single string array

    % Calculate distanceMatrix if required
    if any(inputNames == "r_nextagent")
        distanceMatrix = calc_r_nextagent(gridSize, agentPositions, agent_model.n_x_s, agent_model.n_y_s);
    end

    for q = 2:agent_model.n_q
        % Calculate normalized response time for all agents
        m_t_response = calc_t_response(...
            agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
            agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
            ang_w, v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);

        for a = 1:agent_model.n_a
            fis = fisArray(a);
            fisInputs = struct;

            % Check for local maps flag and generate local maps if needed
            if isfield(config, 'flag_local_maps') && config.flag_local_maps
                local_m_t_response = get_local_map(m_t_response(:,:,a), agent_model.a_loc(a,:), config.r_local_maps);
                local_m_bo_s = get_local_map(agent_model.m_bo_s, agent_model.a_loc(a,:), config.r_local_maps);
                local_m_dw_s = get_local_map(agent_model.m_dw_s, agent_model.a_loc(a,:), config.r_local_maps);
                local_m_scan = get_local_map(agent_model.m_scan, agent_model.a_loc(a,:), config.r_local_maps);
                local_m_f_s = get_local_map(agent_model.m_f_s, agent_model.a_loc(a,:), config.r_local_maps);
                local_m_prior = get_local_map(agent_model.m_prior, agent_model.a_loc(a,:), config.r_local_maps);
            else
                local_m_t_response = [];
                local_m_bo_s = [];
                local_m_dw_s = [];
                local_m_scan = [];
                local_m_f_s = [];
                local_m_prior = [];
            end

            % Loop through FIS inputs and assign values
            for i = 1:numel(fis.Inputs)
                inputName = fis.Inputs(i).Name;

                switch inputName
                    case 't_response'
                        fisInputs.t_response = get_fis_input(config, local_m_t_response, m_t_response(:,:,a), 'flag_local_maps');
                    case 'priority_combined'
                        agent_model.m_prior = calc_prior_combined(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, agent_model.m_victim_s, config.flag_victim_model);
                        fisInputs.priority = get_fis_input(config, local_m_prior, agent_model.m_prior, 'flag_local_maps');
                    case 'priority_first_scan'
                        % NOTE: Not updated for local prediction
                        fisInputs.priority_first_scan = calc_prior_first_scan(agent_model.m_bo_s, agent_model.m_scan);
                    case 'priority_dw'
                        fisInputs.priority_dw = get_fis_input(config, local_m_dw_s, agent_model.m_dw_s, 'flag_local_maps');
                    case 'r_nextagent'
                        fisInputs.r_nextagent = distanceMatrix{a};
                    case 'cell_priority'
                        fisInputs.cell_priority = get_fis_input(config, local_m_bo_s, agent_model.m_bo_s, 'flag_local_maps');
                    case 'cell_scan_certainty'
                        fisInputs.cell_scan_certainty = get_fis_input(config, local_m_scan, agent_model.m_scan, 'flag_local_maps');
                    case 'cell_fire_time_risk'
                        fisInputs.cell_fire_time_risk = get_fis_input(config, calc_fire_time_risk(local_m_f_s, v_w), calc_fire_time_risk(agent_model.m_f_s, v_w), 'flag_local_maps');
                end
            end

            % Generate attraction map using prepared FIS inputs
            m_att = calc_att(fis, fisInputs, agent_model.a_target, config.flag_communication_model);

            % Convert local attraction map to global map if needed
            if isfield(config, 'flag_local_maps') && config.flag_local_maps
                m_att = set_local_map(m_att, agent_model.a_loc(a,:), config.r_local_maps, [agent_model.n_x_s, agent_model.n_y_s]);
            end

            % Task assignment
            agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, agent_model.m_recharge, agent_model.a_loc, agent_model.a_battery_level, agent_model.a_battery_level_i, agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as, config.flag_communication_model);
        end
    end
end

function input_value = get_fis_input(config, local_value, global_value, flag_name)
    % GET_FIS_INPUT Returns the appropriate FIS input value based on the config flag.
    if isfield(config, flag_name) && config.(flag_name)
        input_value = local_value;
    else
        input_value = global_value;
    end
end

function m_prior_rescan = calc_prior_rescan(m_bo, m_scan, t, m_victim, flag_victim_model)
    % Restrict to range 0-1 to prevent too high attraction
    if flag_victim_model
        m_victim = m_victim ./ max(m_victim, [], 'all');
    end

    % Convert matrices to double for calculation
    m_bo = double(m_bo);
    m_scan = double(m_scan);
    m_victim = double(m_victim);

    % Re-scan priority
    % Decide on the basis for re-scan priority based on flag_victim_model
    time_since_scan = max(t - m_scan, 0);  % Ensuring no negative values
    if flag_victim_model
        m_prior_rescan = double(m_scan ~= 0) .* m_victim .* time_since_scan + 1;
    else
        m_prior_rescan = double(m_scan ~= 0) .* m_bo .* time_since_scan + 1;
    end
end

function m_prior_first_scan = calc_prior_first_scan(m_bo, m_scan)
    % Convert matrices to double for calculation
    m_bo = double(m_bo);
    m_scan = double(m_scan);

    % First-time scan priority
    % Using building occupancy as a basis for unscanned cells
    m_prior_first_scan = double(m_scan == 0) .* m_bo;
end

function m_prior_dw = calc_prior_dw(m_dw)
    % Convert matrices to double for calculation
    m_dw = double(m_dw);

    % Incorporate downwind map and travel time
    % Assuming proximity to fire (lower m_dw) increases priority, and longer travel time decreases priority
    m_prior_dw = (1 - m_dw);
end

function m_prior = calc_prior_combined(m_bo, m_dw, m_scan, t, m_victim, flag_victim_model)
    m_P_first_scan = calc_prior_first_scan(m_bo, m_scan);
    m_P_rescan = calc_prior_rescan(m_bo, m_scan, t, m_victim, flag_victim_model);
    m_P_dw = calc_prior_dw(m_dw);

    % Calculate overall priority
    m_prior = m_P_first_scan + m_P_rescan + m_P_dw;
end

function distanceMatrix = calc_r_nextagent(gridSize, agentPositions, n_x_s, n_y_s)
    numRows = gridSize(1);
    numCols = gridSize(2);
    numAgents = size(agentPositions, 1);

    % Calculate the diagonal distance of the search area
    diagonalDistance = sqrt((n_x_s-1)^2 + (n_y_s-1)^2);

    % Initialize the distance matrix
    distanceMatrix = cell(numAgents, 1);

    for a = 1:numAgents
        % Create a matrix for the current agent with initial values as infinity
        currentAgentDistances = inf(numRows, numCols);

        for row = 1:numRows
            for col = 1:numCols
                % Calculate the Euclidean distance from the cell to all other agents
                for otherAgent = 1:numAgents
                    if otherAgent ~= a
                        distToOtherAgent = sqrt((col - agentPositions(otherAgent, 1))^2 + (row - agentPositions(otherAgent, 2))^2);
                        % Update the minimum distance to any other agent
                        currentAgentDistances(row, col) = min(currentAgentDistances(row, col), distToOtherAgent);
                    end
                end
            end
        end

        % Normalize the distances by the diagonal length of the search area
        currentAgentDistances(currentAgentDistances == inf) = 0;  % Handle case where no other agent is close
        distanceMatrix{a} = currentAgentDistances / diagonalDistance;
    end
end

% % V2.4
% function agent_model = model_fis(agent_model, ang_w, v_w, config, fisArray)
%     gridSize = [agent_model.n_x_s, agent_model.n_y_s];
%     agentPositions = agent_model.a_loc; % Assuming a_loc stores the (x, y) positions of agents
% 
%     % Determine if 'r_nextagent' input is required by checking the first FIS
%     % (assuming all agents have the same FIS inputs)
%      inputNames = [fisArray(1).Inputs.Name];  % Concatenate into a single string array
% 
%     % Calculate distanceMatrix if required
%     if any(inputNames == "r_nextagent")
%         distanceMatrix = calc_r_nextagent(gridSize, agentPositions, agent_model.n_x_s, agent_model.n_y_s);
%     end
% 
%     for q = 2:agent_model.n_q
% 
%         % Calculate normalized response time for all agents
%         m_t_response = calc_t_response(...
%             agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%             agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
%             ang_w, v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);
% 
%         for a = 1:agent_model.n_a
%             fis = fisArray(a);
%             fisInputs = struct;
% 
%             for i = 1:numel(fis.Inputs)
%                 inputName = fis.Inputs(i).Name;
% 
%                 switch inputName
%                   case 't_response' % Equivalent to 'Cell time efficiency' in Mirko model
%                       fisInputs.t_response = m_t_response(:,:,a);
%                   case 'priority_combined'
%                       agent_model.m_prior = calc_prior_combined(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, agent_model.m_victim_s, config.flag_victim_model);
%                       fisInputs.priority = agent_model.m_prior;
%                   case 'priority_first_scan'
%                       fisInputs.priority_first_scan = calc_prior_first_scan(m_bo_s, m_scan);
%                   case 'priority_dw'
%                       fisInputs.priority_dw = calc_prior_dw(agent_model.m_dw_s);
%                   case 'r_nextagent'
%                       fisInputs.r_nextagent = distanceMatrix{a};
%                   case 'cell_priority'
%                       fisInputs.cell_priority = agent_model.m_bo_s;
%                   case 'cell_scan_certainty'
%                     fisInputs.cell_scan_certainty = agent_model.m_scan;
%                   case 'cell_fire_time_risk'
%                     fisInputs.cell_fire_time_risk = calc_fire_time_risk(agent_model.m_f_s, v_w);
%                 end
%             end
% 
%             % Generate attraction map using prepared FIS inputs
%             m_att = calc_att(fis, fisInputs, agent_model.a_target, config.flag_communication_model, agent_model.m_recharge);
% 
%             % Task assignment
%             agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, agent_model.m_recharge, agent_model.a_loc, agent_model.a_battery_level, agent_model.a_battery_level_i, agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as, config.flag_communication_model);
%         end
%     end
% end
% 
% function m_prior_rescan = calc_prior_rescan(m_bo, m_scan, t, m_victim, flag_victim_model)
% 
%   % Restrict to range 0-1 to prevent too high attraction
%   if flag_victim_model
%     m_victim = m_victim ./ max(m_victim, [], 'all');
%   end
% 
%   % Convert matrices to double for calculation
%   m_bo = double(m_bo);
%   m_scan = double(m_scan);
%   m_victim = double(m_victim);
% 
%   % Re-scan priority
%   % Decide on the basis for re-scan priority based on flag_victim_model
%   time_since_scan = max(t - m_scan, 0);  % Ensuring no negative values
%   if flag_victim_model
%     % m_P_re_scan = m_victim .* weight.repeat_scan .* time_since_scan;
%     m_prior_rescan = double(m_scan ~= 0) .* m_victim .* time_since_scan + 1;
%   else
%     % m_P_re_scan = m_bo .* weight.repeat_scan .* time_since_scan;
%     m_prior_rescan = double(m_scan ~= 0) .* m_bo .* time_since_scan + 1;
%   end
% 
% end
% 
% % V2.2 - added victim model flag
% function m_prior_first_scan = calc_prior_first_scan(m_bo, m_scan)
% 
%   % Convert matrices to double for calculation
%   m_bo = double(m_bo);
%   m_scan = double(m_scan);
% 
%   % First-time scan priority
%   % Using building occupancy as a basis for unscanned cells
%   m_prior_first_scan = double(m_scan == 0) .* m_bo;
% 
% end
% 
% function m_prior_dw = calc_prior_dw(m_dw)
% 
%   % Convert matrices to double for calculation
%   m_dw = double(m_dw);
% 
%   % Incorporate downwind map and travel time
%   % Assuming proximity to fire (lower m_dw) increases priority, and longer travel time decreases priority
%   m_prior_dw = (1 - m_dw);
% end
% 
% % Single priority with multiple components
% function m_prior = calc_prior_combined(m_bo, m_dw, m_scan, t, m_victim, flag_victim_model)
% 
%   % 
%   m_P_first_scan  =  calc_prior_first_scan(m_bo, m_scan);
%   m_P_rescan  = calc_prior_rescan(m_bo, m_scan, t, m_victim, flag_victim_model);
%   m_P_dw = calc_prior_dw(m_dw);
% 
%   % Calculate overall priority
%   m_prior = m_P_first_scan + m_P_rescan + m_P_dw;
% end
% 
% % Return map of distance of each cell from an agent other than the current agent
% 
% function distanceMatrix = calc_r_nextagent(gridSize, agentPositions, n_x_s, n_y_s)
%     % gridSize is a two-element vector [numRows, numCols]
%     % agentPositions is an n-by-2 matrix, where n is the number of agents,
%     % and each row represents the (x, y) coordinates of an agent.
% 
%     numRows = gridSize(1);
%     numCols = gridSize(2);
%     numAgents = size(agentPositions, 1);
% 
%     % Calculate the diagonal distance of the search area
%     diagonalDistance = sqrt((n_x_s-1)^2 + (n_y_s-1)^2);
% 
%     % Initialize the distance matrix
%     distanceMatrix = cell(numAgents, 1);
% 
%     for a = 1:numAgents
%         % Create a matrix for the current agent with initial values as infinity
%         currentAgentDistances = inf(numRows, numCols);
% 
%         for row = 1:numRows
%             for col = 1:numCols
%                 % Calculate the Euclidean distance from the cell to all other agents
%                 for otherAgent = 1:numAgents
%                     if otherAgent ~= a
%                         distToOtherAgent = sqrt((col - agentPositions(otherAgent, 1))^2 + (row - agentPositions(otherAgent, 2))^2);
%                         % Update the minimum distance to any other agent
%                         currentAgentDistances(row, col) = min(currentAgentDistances(row, col), distToOtherAgent);
%                     end
%                 end
%             end
%         end
% 
%         % Normalize the distances by the diagonal length of the search area
%         currentAgentDistances(currentAgentDistances == inf) = 0;  % Handle case where no other agent is close
%         distanceMatrix{a} = currentAgentDistances / diagonalDistance;
%     end
% end


