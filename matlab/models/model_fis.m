% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% CHANGELOG
% Rename: to model_fis 
% Bugfix: Coarsen downwind map
% Removed test_fis_sensitivity feature
% Refactor: environment_model and agent_model structures

% TODO
% Combined priority should use weights
% Test fire burn risk - should be infinite if/very high if unburnable - could
% also test unburnable neighbourhoods
% Add new inputs to init files (config.sigma, agent_model.sensor_accuracy)
% Check if to do calculations by agent or not - will be some repeated operations
% here

% COMMENTS
% cell_fire_time_risk not directional with wind

% V2.4
function agent_model = model_fis(agent_model, ang_w, v_w, config, fisArray)
    gridSize = [agent_model.n_x_s, agent_model.n_y_s];
    agentPositions = agent_model.a_loc; % Assuming a_loc stores the (x, y) positions of agents
    
    % Determine if 'r_nextagent' input is required by checking the first FIS
    % (assuming all agents have the same FIS inputs)
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

            for i = 1:numel(fis.Inputs)
                inputName = fis.Inputs(i).Name;

                switch inputName
                  case 't_response' % Equivalent to 'Cell time efficiency' in Mirko model
                      fisInputs.t_response = m_t_response(:,:,a);
                  case 'priority_combined'
                      agent_model.m_prior = calc_prior_combined(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, agent_model.m_victim_s, config.flag_victim_model);
                      fisInputs.priority = agent_model.m_prior;
                  case 'priority_first_scan'
                      fisInputs.priority_first_scan = calc_prior_first_scan(m_bo, m_scan);
                  % case 'priority_rescan'
                  %     fisInputs.priority_rescan = calc_prior_rescan(m_bo, m_scan, t, m_victim, flag_victim_model);
                  case 'priority_dw'
                      fisInputs.priority_dw = calc_prior_dw(m_dw);
                  case 'r_nextagent'
                      fisInputs.r_nextagent = distanceMatrix{a};
                  case 'cell_priority'
                      fisInputs.cell_priority = agent_model.m_bo_s;
                  case 'cell_scan_certainty'
                    fisInputs.cell_scan_certainty = agent_model.m_scan;
                  case 'cell_fire_time_risk'
                    fisInputs.cell_fire_time_risk = calc_fire_time_risk(agent_model.m_f_s, v_w);
                end
            end

            % Generate attraction map using prepared FIS inputs
            m_att = calc_att(fis, fisInputs, agent_model.a_target, config.flag_communication_model);

            % Task assignment
            agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, config.flag_communication_model);
        end
    end
end


% function m_scan = calc_scan_certainty(m_scan)
% 
%   % Scan certainty logic implemented in model_agent function, evaluated at each
%   % agent timestep
% 
%   % % Loop through each cell in the matrix
%     % for i = 1:n_x_s
%     %     for j = 1:n_y_s
%     %         % If the cell (i,j) is not scanned at the current time step
%     %         if m_scan_certainty(i,j) == 0
%     %             % Decrease the scan certainty by a fixed amount, sigma
%     %             % but ensure it doesn't go below 0
%     %             m_scan_certainty(i,j) = max(m_scan_certainty(i,j) - sigma, 0);
%     %         else
%     %             % If the cell is scanned, take the maximum of the
%     %             % current certainty minus sigma and the sensor accuracy
%     %             v
%     %         end
%     %     end
%     % end
% 
%     agent_model.m_scan
% end

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
    % m_P_re_scan = m_victim .* weight.repeat_scan .* time_since_scan;
    m_prior_rescan = double(m_scan ~= 0) .* m_victim .* time_since_scan + 1;
  else
    % m_P_re_scan = m_bo .* weight.repeat_scan .* time_since_scan;
    m_prior_rescan = double(m_scan ~= 0) .* m_bo .* time_since_scan + 1;
  end

end

% V2.2 - added victim model flag
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

% Single priority with multiple components
function m_prior = calc_prior_combined(m_bo, m_dw, m_scan, t, m_victim, flag_victim_model)

  % 
  m_P_first_scan  =  calc_prior_first_scan(m_bo, m_scan);
  m_P_rescan  = calc_prior_rescan(m_bo, m_scan, t, m_victim, flag_victim_model);
  m_P_dw = calc_prior_dw(m_dw);

  % Calculate overall priority
  m_prior = m_P_first_scan + m_P_rescan + m_P_dw;
end

% % Cell fire time risk (theory - to be tested)

function t_fire = calc_fire_time_risk(m_f, v_w)
    [n_rows, n_cols] = size(m_f);
    % Initialize with 1000 to signify cells not yet affected by fire.
    % Must be within range of input fuzzy MF
    t_fire = 100 * ones(n_rows, n_cols); 
  
    % Define relative positions for different radii
    radius1 = getRadius(1);
    radius2 = getRadius(2);
    radius3 = getRadius(3);

    for i = 1:n_rows
        for j = 1:n_cols
            % Check neighbors within the respective radii
            for rad = 1:3
                % Get the relative positions based on the radius
                switch rad
                    case 1, rel_pos = radius1;
                    case 2, rel_pos = radius2;
                    case 3, rel_pos = radius3;
                end
                
                % Extract neighbor indices within grid bounds
                neighbors = get_valid_neighbors(i, j, n_rows, n_cols, rel_pos);
                
                % Check conditions based on wind velocity
                t_fire = apply_fire_rules(neighbors, m_f, v_w, rad, t_fire, i, j);
            end
        end
    end
end

function rel_pos = getRadius(radius)
    offset = -radius:radius;
    rel_pos = [offset', zeros(numel(offset), 1); zeros(numel(offset), 1), offset'];
end

function neighbors = get_valid_neighbors(i, j, n_rows, n_cols, rel_pos)
    neighbors = [];
    for k = 1:size(rel_pos, 1)
        ni = i + rel_pos(k, 1);
        nj = j + rel_pos(k, 2);
        if ni >= 1 && ni <= n_rows && nj >= 1 && nj <= n_cols
            neighbors(end+1) = sub2ind([n_rows, n_cols], ni, nj);
        end
    end
end

function t_fire = apply_fire_rules(neighbors, m_f, v_w, radius, t_fire, i, j)
    
    % Ensure that neighbors are within the bounds of m_fire
    neighbors = neighbors(neighbors > 0 & neighbors <= numel(m_f));

    if v_w == 0  % Zero wind velocity
        if radius == 1 && any(m_f(neighbors) == 3)
            t_fire(i, j) = 0;
        elseif radius == 1 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 2);
        elseif radius == 2 && any(m_f(neighbors) == 3)
            t_fire(i, j) = min(t_fire(i, j), 2);
        elseif radius == 2 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 4);
        elseif radius == 3 && any(m_f(neighbors) == 3)
            t_fire(i, j) = min(t_fire(i, j), 4);
        elseif radius == 3 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 6);
        end
    elseif v_w <= 5  % Low wind velocity
        if radius <= 2 && any(m_f(neighbors) == 3)
            t_fire(i, j) = 0;
        elseif radius <= 2 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 2);
        end
    else  % High wind velocity
        if radius <= 3 && any(m_f(neighbors) == 3)
            t_fire(i, j) = 0;
        elseif radius == 1 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 2);
        end
    end
end

% Return map of distance of each cell from an agent other than the current agent

function distanceMatrix = calc_r_nextagent(gridSize, agentPositions, n_x_s, n_y_s)
    % gridSize is a two-element vector [numRows, numCols]
    % agentPositions is an n-by-2 matrix, where n is the number of agents,
    % and each row represents the (x, y) coordinates of an agent.

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


