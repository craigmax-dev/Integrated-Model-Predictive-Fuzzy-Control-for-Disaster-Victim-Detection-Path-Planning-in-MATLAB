% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

function agent_model = model_fis_global(agent_model, ang_w, v_w, config, fisArray, m_f)
    % MODEL_FIS_GLOBAL Simulates the agent model using global maps for predictions.
    %
    % This function calculates the normalized response time for all agents,
    % prepares the inputs for the Fuzzy Inference System (FIS), generates the
    % attraction map, and performs task assignment based on the global map data.
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
    %   agent_model = model_fis_global(agent_model, ang_w, v_w, config, fisArray);

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

            % Loop through FIS inputs and assign values
            for i = 1:numel(fis.Inputs)
                inputName = fis.Inputs(i).Name;

                switch inputName
                    case 't_response'
                        fisInputs.t_response = m_t_response(:,:,a);
                    case 'priority_dw'
                        fisInputs.priority_dw = agent_model.m_dw_s;
                    % case 'r_nextagent'
                    %     fisInputs.r_nextagent = distanceMatrix{a};
                    case 'cell_priority'
                        fisInputs.cell_priority = agent_model.m_bo_s;
                    case 'cell_scan_certainty'
                        fisInputs.cell_scan_certainty = agent_model.m_scan;
                    case 'cell_fire_time_risk'
                        % fisInputs.cell_fire_time_risk = calc_fire_time_risk(agent_model.m_f_s, v_w);
                        m_fire_time_risk_env = calc_fire_time_risk(m_f, v_w);
                        m_fire_time_risk_c = func_coarsen(m_fire_time_risk_env, config.c_f_s);
                        fisInputs.cell_fire_time_risk = m_fire_time_risk_c;
                end
            end

            % Generate attraction map using prepared FIS inputs
            m_att = calc_att(fis, fisInputs, agent_model.a_target, config.flag_communication_model);

            % Task assignment
            agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, agent_model.m_recharge, agent_model.a_loc, agent_model.a_battery_level, agent_model.a_battery_level_i, agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as, config.flag_communication_model);
        end
    end
end
