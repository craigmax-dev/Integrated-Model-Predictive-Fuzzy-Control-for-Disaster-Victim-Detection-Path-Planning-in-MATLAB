% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% CHANGELOG
% Rename: to model_fis 
% Bugfix: Coarsen downwind map
% Removed test_fis_sensitivity feature
% Refactor: environment_model and agent_model structures


% V2.3
function agent_model = model_fis(agent_model, v_w, ang_w, config, fisArray)
    gridSize = [agent_model.n_x_s, agent_model.n_y_s];
    agentPositions = agent_model.a_loc; % Assuming a_loc stores the (x, y) positions of agents

    % Pre-calculate agent distances if required by any FIS
    if any(arrayfun(@(fis) any(strcmp({fis.Inputs.Name}, 'r_nextagent')), fisArray))
        distanceMatrix = calculateAgentDistances(gridSize, agentPositions);
    end

    for q = 2:agent_model.n_q
        % Calculate normalized response time for all agents
        m_t_response = calc_t_response(...
            agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
            agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
            ang_w, v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);

        for a = 1:agent_model.n_a
            % FIS of agent
            fis = fisArray(a);

            % Prepare FIS inputs based on the inputs defined in fisArray.Inputs
            fisInputs = struct;
            for i = 1:numel(fis.Inputs)
                inputName = fis.Inputs(i).Name;
                switch inputName
                    case 't_response'
                        fisInputs.t_response = m_t_response(:,:,a);
                    case 'priority'
                        agent_model.m_prior = calc_prior(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, config.weight, agent_model.m_victim_s, config.flag_victim_model);
                        fisInputs.priority = agent_model.m_prior;
                    case 'r_nextagent'
                        fisInputs.r_nextagent = distanceMatrix{a};
                end
            end

            % Generate attraction map using the prepared FIS inputs
            m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, agent_model.a_target, config.flag_communication_model);

            % Task assignment
            agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, config.flag_communication_model);
        end
    end
end

