% Simulate UAV fuzzy controller
% Date:     01/12/2023
% Author:   Craig Maxwell

% V2

% CHANGELOG
% Rename: to model_fis 

% TODO
% Update the following function with an additional agent task. This is for a case in which the agent is deactivated. This will happen when the agent battery level reaches zero. When in this state the agent is stuck in an 'idle' state and can not perform any tasks and can not be removed from this state.


% V2 - refactor: modularization and performance improvements
function [a_target, fis_data] = model_fis( ...
            n_a, a_target, n_q, ...
            n_x_s, n_y_s, l_x_s, l_y_s, ...
            m_scan, m_t_scan, m_dw, m_prior, ...
            fisArray, ...
            a_t_trav, a_t_scan, ...
            ang_w, v_as, v_w, test_fis_sensitivity, fis_data)
  
    % Initialize schedule map
    m_schedule = zeros(n_x_s, n_y_s);
    occupied_indices = ~isnan(a_target(:, 1, 1));
    m_schedule(sub2ind([n_x_s, n_y_s], a_target(occupied_indices, 1, 1), a_target(occupied_indices, 2, 1))) = 1;
    
    % Iterate over each task queue
    for q = 2:n_q 
        % Calculate normalized response time for all agents
        m_t_response = calc_t_response(...
            n_x_s, n_y_s, l_x_s, l_y_s, ...
            n_a, a_t_scan, a_t_trav, a_target, q, ...
            ang_w, v_w, v_as, m_t_scan);
        
        for a = 1:n_a
            % FIS of agent
            fis = fisArray(a);

            % Generate attraction map
            m_att = calc_att(fis, m_t_response(:,:,a), m_prior, m_dw, m_scan, m_schedule);
            
            % Task assignment
            [a_target, m_schedule] = func_taskAssignment(a, q, a_target, m_att, m_schedule);
            
            % Record FIS data if testing sensitivity
            if test_fis_sensitivity
                fis_data_local = [reshape(m_t_response(:,:,a), [], 1), reshape(m_prior, [], 1), ...
                                  reshape(m_dw, [], 1), reshape(m_att, [], 1)];
                fis_data = [fis_data; fis_data_local];
            end
        end
    end
end

% V1
% function  [a_target, fis_data] = model_fis( ...
%             n_a, a_target, n_q, ...
%             n_x_s, n_y_s, l_x_s, l_y_s, ...
%             m_scan, m_t_scan, m_dw, m_prior, ...
%             fisArray, ...
%             a_t_trav, a_t_scan, ...
%             ang_w, v_as, v_w, test_fis_sensitivity, fis_data)
%   
%   % Initialise maps
%   m_schedule   = zeros(n_x_s, n_y_s);
%   for a=1:n_a
%     if ~isnan(a_target(a, :, 1))
%       m_schedule(a_target(a, 1, 1), a_target(a, 2, 1)) = 1;
%     end
%   end
%   
%   % Assign tasks
% 	for q = 2:n_q 
%     
%     % Calculate normalised response time for all agents
%     [m_t_response] = calc_t_response(...
%       n_x_s, n_y_s, l_x_s, l_y_s, ...
%       n_a, a_t_scan, a_t_trav, a_target, q, ...
%       ang_w, v_w, v_as, m_t_scan);
%     
%     for a=1:n_a
%       % FIS of agent
%       fis = fisArray(a);
%       % Generate attraction map
%       m_att = zeros(n_x_s, n_y_s);
%       
%       for i=1:n_x_s
%         for j=1:n_y_s
%           m_att(i,j) = calc_att (fis, ...
%             m_t_response(i,j,a), m_prior(i,j), m_dw(i,j), ...
%             m_scan(i,j), m_schedule(i,j));
%           if test_fis_sensitivity
%             % Record fis data
%             fis_data = [fis_data; m_t_response(i,j,a), m_prior(i,j), m_dw(i,j), m_att(i,j)];
%           end
%         end
%       end
%       
%       % Task assignment
%     [a_target, m_schedule] =  func_taskAssignment(...
%       a, q, a_target, m_att, m_schedule);
%     end
%   end
% end
