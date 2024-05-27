function agent_model = model_fis_local(agent_model, ang_w, v_w, config, fisArray)
    % MODEL_FIS_LOCAL Simulates the agent model using local maps for predictions.
    %
    % This function calculates the normalized response time for all agents,
    % prepares the inputs for the Fuzzy Inference System (FIS), generates the
    % attraction map, and performs task assignment based on the local map data.
    %
    % Inputs:
    %   agent_model - The structure containing the agent model data.
    %   ang_w       - Angular weight used in calculations.
    %   v_w         - Velocity weight used in calculations.
    %   config      - Configuration structure containing various parameters.
    %   fisArray    - Array of Fuzzy Inference System (FIS) structures for each agent.
    %
    % Output:
    %   agent_model - The updated agent model structure after processing.
    %
    % Example usage:
    %   agent_model = model_fis_local(agent_model, ang_w, v_w, config, fisArray);

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

            % Generate local maps
            local_m_t_response = get_local_map(m_t_response(:,:,a), agent_model.a_loc(a,:), config.r_local_maps);
            local_m_bo_s = get_local_map(agent_model.m_bo_s, agent_model.a_loc(a,:), config.r_local_maps);
            local_m_dw_s = get_local_map(agent_model.m_dw_s, agent_model.a_loc(a,:), config.r_local_maps);
            local_m_scan = get_local_map(agent_model.m_scan, agent_model.a_loc(a,:), config.r_local_maps);
            local_m_f_s = get_local_map(agent_model.m_f_s, agent_model.a_loc(a,:), config.r_local_maps);
            local_m_prior = get_local_map(agent_model.m_prior, agent_model.a_loc(a,:), config.r_local_maps);

            % Loop through FIS inputs and assign values
            for i = 1:numel(fis.Inputs)
                inputName = fis.Inputs(i).Name;

                switch inputName
                    case 't_response'
                        fisInputs.t_response = local_m_t_response;
                    case 'priority_combined'
                        agent_model.m_prior = calc_prior_combined(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, agent_model.m_victim_s, config.flag_victim_model);
                        fisInputs.priority = local_m_prior;
                    case 'priority_first_scan'
                        % NOTE: Not updated for local prediction
                        fisInputs.priority_first_scan = calc_prior_first_scan(agent_model.m_bo_s, agent_model.m_scan);
                    case 'priority_dw'
                        fisInputs.priority_dw = local_m_dw_s;
                    case 'r_nextagent'
                        fisInputs.r_nextagent = distanceMatrix{a};
                    case 'cell_priority'
                        fisInputs.cell_priority = local_m_bo_s;
                    case 'cell_scan_certainty'
                        fisInputs.cell_scan_certainty = local_m_scan;
                    case 'cell_fire_time_risk'
                        fisInputs.cell_fire_time_risk = calc_fire_time_risk(local_m_f_s, v_w);
                end
            end

            % Generate attraction map using prepared FIS inputs
            m_att = calc_att(fis, fisInputs, agent_model.a_target, config.flag_communication_model);

            % Convert local attraction map to global map
            m_att = set_local_map(m_att, agent_model.a_loc(a,:), config.r_local_maps, [agent_model.n_x_s, agent_model.n_y_s]);

            % Task assignment
            agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, agent_model.m_recharge, agent_model.a_loc, agent_model.a_battery_level, agent_model.a_battery_level_i, agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as, config.flag_communication_model);
        end
    end
end
