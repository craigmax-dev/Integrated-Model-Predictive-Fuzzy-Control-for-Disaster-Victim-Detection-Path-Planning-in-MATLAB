% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% V2

% CHANGELOG
% Rename: to model_fis 
% Bugfix: Coarsen downwind map
% Removed test_fis_sensitivity feature

% TODO
% Update the following function with an additional agent task. This is for a case in which the agent is deactivated. This will happen when the agent battery level reaches zero. When in this state the agent is stuck in an 'idle' state and can not perform any tasks and can not be removed from this state.

% V2 - refactor: modularization and performance improvements
% function [agent_model.a_target, fis_data] = model_fis( ...
%             agent_model.n_a, agent_model.a_target, agent_model.n_q, ...
%             agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%             agent_model.m_scan, agent_model.m_t_scan, m_dw_e, agent_model.m_prior, ...
%             fisArray, ...
%             agent_model.a_t_trav, agent_model.a_t_scan, ...
%             ang_w, agent_model.v_as, v_w, test_fis_sensitivity, fis_data, agent_model.c_f_s)

function [agent_model] = model_fis(agent_model, m_dw_e, fisArray, ang_w, v_w)
            
    % Coarsen downwind map
    m_dw = func_coarsen(m_dw_e, agent_model.c_f_s); 
          
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
            ang_w, v_w, agent_model.v_as, agent_model.m_t_scan);
        
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

% V1
% function  [agent_model.a_target, fis_data] = model_fis( ...
%             agent_model.n_a, agent_model.a_target, agent_model.n_q, ...
%             agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%             agent_model.m_scan, agent_model.m_t_scan, m_dw, agent_model.m_prior, ...
%             fisArray, ...
%             agent_model.a_t_trav, agent_model.a_t_scan, ...
%             ang_w, agent_model.v_as, v_w, test_fis_sensitivity, fis_data)
%   
%   % Initialise maps
%   agent_model.m_schedule   = zeros(agent_model.n_x_s, agent_model.n_y_s);
%   for a=1:agent_model.n_a
%     if ~isnan(agent_model.a_target(a, :, 1))
%       agent_model.m_schedule(agent_model.a_target(a, 1, 1), agent_model.a_target(a, 2, 1)) = 1;
%     end
%   end
%   
%   % Assign tasks
% 	for q = 2:agent_model.n_q 
%     
%     % Calculate normalised response time for all agents
%     [m_t_response] = calc_t_response(...
%       agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
%       agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, q, ...
%       ang_w, v_w, agent_model.v_as, agent_model.m_t_scan);
%     
%     for a=1:agent_model.n_a
%       % FIS of agent
%       fis = fisArray(a);
%       % Generate attraction map
%       m_att = zeros(agent_model.n_x_s, agent_model.n_y_s);
%       
%       for i=1:agent_model.n_x_s
%         for j=1:agent_model.n_y_s
%           m_att(i,j) = calc_att (fis, ...
%             m_t_response(i,j,a), agent_model.m_prior(i,j), m_dw(i,j), ...
%             agent_model.m_scan(i,j), agent_model.m_schedule(i,j));
%           if test_fis_sensitivity
%             % Record fis data
%             fis_data = [fis_data; m_t_response(i,j,a), agent_model.m_prior(i,j), m_dw(i,j), m_att(i,j)];
%           end
%         end
%       end
%       
%       % Task assignment
%     [agent_model.a_target, agent_model.m_schedule] =  func_taskAssignment(...
%       a, q, agent_model.a_target, m_att, agent_model.m_schedule);
%     end
%   end
% end
