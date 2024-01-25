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
            agent_model.m_prior = calc_prior(agent_model.m_bo_s, agent_model.m_dw_s, agent_model.m_scan, config.t, config.weight, agent_model.m_victim_s);

            % Generate attraction map
            m_att = calc_att(fis, m_t_response(:,:,a), agent_model.m_prior, agent_model.a_target, config.communication_enabled);
            
            % Task assignment
            % agent_model.a_target(a, :, q) = func_taskAssignment(m_att);
            % agent_model.a_target(a, :, q) = func_taskAssignment(q, agent_model.a_target, m_att, config.communication_enabled);

            % Get the target for the current agent and queue position
            target_for_q = func_taskAssignment(q, agent_model.a_target, m_att, config.communication_enabled);
        
            % Update the target for the current agent and queue position
            agent_model.a_target(a, :, q) = target_for_q;

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