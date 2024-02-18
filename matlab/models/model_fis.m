% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% CHANGELOG
% Rename: to model_fis 
% Bugfix: Coarsen downwind map
% Removed test_fis_sensitivity feature
% Refactor: environment_model and agent_model structures

% TODO
% Refactor: schedule map removed. Instead agents rely on local task queue
% Refactor: move m_t_response inside agent loop
% Review changes
% Test % V2.3 implementation

% % V2.4 
% % NOTE: in progress, results not good
% function agent_model = model_fis(agent_model, environment_model, config, fisArray)
%   % Calculate distances of all cells from other agents, normalized
%   gridSize = [agent_model.n_x_s, agent_model.n_y_s];
%   agentPositions = reshape(agent_model.a_target(:,1:2,:), [], 2); % Assuming a_target contains [x,y] positions
%   distanceMatrix = calculateAgentDistances(gridSize, agentPositions);
% 
%   % Normalisation
%   maxDist = sqrt(gridSize(1)^2 + gridSize(2)^2);
%   for a = 1:agent_model.n_a
%       distanceMatrix{a} = distanceMatrix{a} / maxDist; % Normalize to [0, 1]
%   end
% 
%   % Iterate over each task queue
%   for q = 2:agent_model.n_q
% 
%     % Calculate normalized response time for all agents
%     m_t_response = calc_t_response(...
%       agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%       agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
%       environment_model.ang_w, environment_model.v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);
% 
%     for a = 1:agent_model.n_a
%       % FIS of agent
%       fis = fisArray(a);
% 
%       % Identify which inputs the current FIS has
%       fisInputs = getFisInputs(fis);
% 
%       % Flatten fisInputs for direct string comparison
%       fisInputsFlat = cellfun(@(c) c{1}, fisInputs, 'UniformOutput', false);
% 
%       % Dynamically build the input matrix based on available FIS inputs
%       fisInputMatrix = [];
%       if ismember('t_response', fisInputsFlat)
%         t_response_input = reshape(m_t_response(:,:,a), [], 1);
%         fisInputMatrix = [fisInputMatrix, t_response_input];
%       end
%       if ismember('priority', fisInputsFlat)
%         priority_input = reshape(agent_model.m_prior, [], 1);
%         fisInputMatrix = [fisInputMatrix, priority_input];
%       end
%       if ismember('r_nextagent', fisInputsFlat)
%         r_nextagent_input = reshape(distanceMatrix{a}, [], 1);
%         fisInputMatrix = [fisInputMatrix, r_nextagent_input];
%       end
% 
%       % Handling of valid cells
%       valid_cells = all(~isinf(fisInputMatrix) & ~isnan(fisInputMatrix), 2); % Identify non-NaN and non-Inf cells
% 
%       % Initialize attraction map
%       m_att = nan(size(m_t_response(:,:,a))); % Use NaN to initialize for clearer handling of invalid cells
% 
%       % Call evalfis only for valid cells
%       if any(valid_cells)
%         % Adjust the indexing for valid_cells to match evalfis input requirement
%         valid_fisInputMatrix = fisInputMatrix(valid_cells, :);
%         valid_outputs = evalfis(valid_fisInputMatrix, fis);
%         m_att(valid_cells) = valid_outputs; % Assign FIS outputs to valid cells in the attraction map
%       end
% 
%       % Task assignment
%       agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, config.flag_communication_model);
%     end
%   end
% end
% 
% function fisInputs = getFisInputs(fis)
%     % Extracts the names of inputs defined in the FIS
%     fisInputs = arrayfun(@(input) input.Name, fis.Inputs, 'UniformOutput', false);
% end


% % V2.3
% function agent_model = model_fis(agent_model, environment_model, config, fisArray)
% 
%     % Calculate distances of all cells from other agents, normalized
%     gridSize = [agent_model.n_x_s, agent_model.n_y_s];
%     agentPositions = reshape(agent_model.a_target(:,1:2,:), [], 2); % Assuming a_target contains [x,y] positions
%     distanceMatrix = calculateAgentDistances(gridSize, agentPositions);
% 
%     % Normalisation
%     % NOTE: move inside calculateAgentDistances function?
%     maxDist = sqrt(gridSize(1)^2 + gridSize(2)^2);
%     for a = 1:agent_model.n_a
%         distanceMatrix{a} = distanceMatrix{a} / maxDist; % Normalize to [0, 1]
%     end
% 
%     % Iterate over each task queue
%     for q = 2:agent_model.n_q
% 
%         % Calculate normalized response time for all agents
%         m_t_response = calc_t_response(...
%             agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%             agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
%             environment_model.ang_w, environment_model.v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);
% 
%         for a = 1:agent_model.n_a
%             % FIS of agent
%             fis = fisArray(a);
% 
%             % Generate attraction map
%             m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, agent_model.a_target, config.flag_communication_model;
%             % m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, agent_model.a_target, config.flag_communication_model, distanceMatrix{a});
% 
% 
%             % Identify which inputs the current FIS has
%             fisInputs = getFisInputs(fis);
% 
%             % Generate inputs for FIS based on what's available
%             fisInputValues = [];
% 
%             % Flatten fisInputs for direct string comparison
%             fisInputsFlat = cellfun(@(c) c{1}, fisInputs, 'UniformOutput', false);
% 
%             if ismember('t_response', fisInputsFlat)
% 
%               fisInputValues = [fisInputValues, m_t_response(:,:,a)]; % Assuming m_t_response calculated elsewhere
% 
%             end
% 
%             if ismember('priority', fisInputsFlat)
% 
%               % Generate priority map
%               agent_model.m_prior = calc_prior(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, config.weight, agent_model.m_victim_s, config.flag_victim_model);
% 
%               fisInputValues = [fisInputValues, agent_model.m_prior]; % Assuming m_prior calculated elsewhere
% 
%             end
% 
%             if ismember('r_nextagent', fisInputsFlat)
% 
%               fisInputValues = [fisInputValues, distanceMatrix{a}]; % Assuming distanceMatrix calculated above
% 
%             end
% 
%             % Task assignment
%             agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, config.flag_communication_model);
%         end
%     end
% end
% 
% function fisInputs = getFisInputs(fis)
%     % Extracts the names of inputs defined in the FIS
%     fisInputs = arrayfun(@(input) input.Name, fis.Inputs, 'UniformOutput', false);
% end


% V2.2
function agent_model = model_fis(agent_model, environment_model, config, fisArray)
    % Iterate over each task queue
    for q = 2:agent_model.n_q
        % Calculate normalized response time for all agents
        m_t_response = calc_t_response(...
            agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
            agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
            environment_model.ang_w, environment_model.v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);

        for a = 1:agent_model.n_a
            % FIS of agent
            fis = fisArray(a);

            % Generate priority map
            agent_model.m_prior = calc_prior(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, config.weight, agent_model.m_victim_s, config.flag_victim_model);

            % Generate attraction map
            m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, agent_model.a_target, config.flag_communication_model);

            % Task assignment
            agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, config.flag_communication_model);

        end
    end
end


% V2.1
% function agent_model = model_fis(agent_model, environment_model, config, fisArray)
%     % Iterate over each task queue
%     for q = 2:agent_model.n_q
%       % Calculate normalized response time for all agents
%       m_t_response = calc_t_response(...
%         agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%         agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
%         environment_model.ang_w, environment_model.v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);
% 
%         for a = 1:agent_model.n_a
%             % FIS of agent
%             fis = fisArray(a);
% 
%             % Generate priority map
%             agent_model.m_prior = calc_prior(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, config.weight, agent_model.m_victim_s);
% 
%             % Generate attraction map
%             m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, agent_model.a_target);
% 
%             % Task assignment
%             agent_model.a_target(a, :, q) = func_taskAssignment(m_att);
% 
%         end
%     end
% end