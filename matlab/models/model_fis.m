% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% V2

% CHANGELOG
% Rename: to model_fis 
% Bugfix: Coarsen downwind map
% Removed test_fis_sensitivity feature
% Refactor: environment_model and agent_model structures 

% TODO
% Update the following function with an additional agent task. This is for a case in which the agent is deactivated. This will happen when the agent battery level reaches zero. When in this state the agent is stuck in an 'idle' state and can not perform any tasks and can not be removed from this state.

function [agent_model] = model_fis(agent_model, environment_model, fisArray)
            
    % Coarsen downwind map
    m_dw = func_coarsen(environment_model.m_dw_e, agent_model.c_f_s); 
          
    % Initialize schedule map
    agent_model.m_schedule = zeros(agent_model.n_x_s, agent_model.n_y_s);
    occupied_indices = ~isnan(agent_model.a_target(:, 1, 1));
    agent_model.m_schedule(sub2ind([agent_model.n_x_s, agent_model.n_y_s], agent_model.a_target(occupied_indices, 1, 1), agent_model.a_target(occupied_indices, 2, 1))) = 1;
    
    % Iterate over each task queue
    for q = 2:agent_model.n_q 
        % Calculate normalized response time for all agents
        m_t_response = calc_t_response(...
            agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
            agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
            environment_model.ang_w, environment_model.v_w, agent_model.v_as, agent_model.m_t_scan);
        
        for a = 1:agent_model.n_a
            % FIS of agent
            fis = fisArray(a);

            % Generate attraction map
            m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, m_dw, agent_model.m_scan, agent_model.m_schedule);
            
            % Task assignment
            [agent_model.a_target, agent_model.m_schedule] = func_taskAssignment(a, q, agent_model.a_target, m_att, agent_model.m_schedule);
            
        end
    end
end
