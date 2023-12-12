%% Function objEval
% Evaluate objective function

% V2

% CHANGELOG
% - coarsen all environment maps to agent maps
% - removed inputs n_x_e, n_y_e
% - refactor: replaced flags with config weights
% - refactor: m_scan now contains time at which the cell was scanned
% - refactor: all objective function components fully configurable

% TODO
% - CHECK m_scan is input to FIS! will allow agents to act based off time last
% scanned

% V2 REFACTOR V4
% Fully configurable objective function
function [s_obj, obj] = calc_obj(...
            config, m_f, m_bo, m_scan, m_victim_scan, ...
            dt_s, s_obj, c_f_s, t)

    % Initialize priority maps
    m_fo = (m_f == 3); % Active fires map

    m_P_victim = m_victim_scan * config.weight_victims; 

    % Coarsen all environment maps to agent resolution
    m_bo_s = func_coarsen(m_bo, c_f_s); 
    m_fo_s = func_coarsen(m_fo, c_f_s); 
    
    % Calculate priority maps based on configuration
    m_P_bo = config.weight_bo * m_bo_s;   % Building occupancy map factor
    m_P_fo = config.weight_fo * m_fo_s;   % Active fire factor
    m_P_first_scan = (double(m_scan ~= 0)) * config.weight_first_scan; % Unscanned cells factor
    m_P_repeat_scan = (t - m_scan) * config.weight_repeat_scan;    % Time since cell scanned factor
    
    m_P = m_P_bo + m_P_fo + m_P_victim + m_P_first_scan + m_P_repeat_scan;

    % Ensure the objective function is always positive
    obj = max(sum(m_P, 'all') * dt_s, 0);
    s_obj = s_obj + obj; % Sum of objective over time
end


% % V2 REFACTOR V3
% % Configurable objective function
% 
% function [s_obj, obj] = calc_obj(...
%             config, m_f, m_bo, m_scan, m_victim_scan, ...
%             dt_s, s_obj, n_x_e, n_y_e, c_f_s)
% 
%     % Initialize priority maps
%     m_fo = zeros(n_x_e, n_y_e);
%     m_P_victim = zeros(n_x_e, n_y_e);
% 
%     % Generate active fire map if required
%     if config.use_fire
%         m_fo = (m_f == 3); % Simplified logical indexing
%     end
%     
%     % Generate victim map if required
%     if config.use_victims
%         m_P_victim = m_victim_scan; % Assuming m_victim_scan is already in environment map resolution
%     end
% 
%     % Coarsen all environment maps to agent maps
%     m_bo_s = func_coarsen(m_bo, c_f_s); 
%     m_fo_s = func_coarsen(m_fo, c_f_s); 
%     
%     % Calculate priority maps based on configuration
%     m_P_bo = config.weight_bo * m_bo_s;
%     m_P_fo = config.weight_fo * m_fo_s;
%     m_P = m_P_bo + m_P_fo + config.weight_victims * m_P_victim;
% 
%     
%     % Modify priority based on scan task
%     if strcmp(config.scan_task, 'single')
%         m_scan_inv = ~m_scan;
%         m_P = m_P .* m_scan_inv;
%     elseif strcmp(config.scan_task, 'repeat')
%         if config.use_victims
%             % Give priority to return to scan a cell only if a victim has been detected
%             m_P = -m_scan .* m_P_victim; % Weighted by the presence of victims
%         else
%             % Standard repeat strategy when not using victim data
%             m_P = -m_scan;
%         end
%     else
%         error('Invalid scan task configuration');
%     end
% 
%     % Calculate objective function
%     obj = sum(m_P, 'all') * dt_s;
%     s_obj = s_obj + obj; % Sum of objective over time
% end


% V2 REFACTOR V2
% - refactor objective function for m_scan (for repeat scan case)

% function [s_obj, obj] = calc_obj(...
%             m_f, m_bo, m_scan, ...
%             r_bo, r_fo, dt_s, s_obj, ...
%             n_x_e, n_y_e, c_f_s, flag_scan_task)
% 
%     % Generate active fire map
%     m_fo = (m_f == 3); % Simplified logical indexing
%     
%     % Convert scan map to environment map resolution
%     m_scan_env = zeros(n_x_e, n_y_e);
%     [rows, cols] = find(m_scan == 1); % Find coordinates of scanned cells
%     for k = 1:length(rows)
%         i = rows(k);
%         j = cols(k);
%         x_range = (c_f_s*(i-1) + 1):(c_f_s*i);
%         y_range = (c_f_s*(j-1) + 1):(c_f_s*j);
%         m_scan_env(x_range, y_range) = 1;
%     end
% 
%     % Determine behavior based on flag_scan_task
%     if strcmp(flag_scan_task, 'single')
%         
%       % Inverse scan map - directly using logical NOT
%         m_scan_env_inv = ~m_scan_env;
% 
%         % Priority due to building and fire occupancy
%         m_P_bo = r_bo .* m_bo;
%         m_P_fo = r_fo .* m_fo;    
%         m_P = m_P_bo + m_P_fo;
% 
%         % Ignore already scanned cells
%         m_P = m_P .* m_scan_env_inv;
% 
%         % Objective function for current timestep
%         obj = sum(m_P, 'all') * dt_s;       
%         
%     elseif strcmp(flag_scan_task, 'repeat')
%       
%         % Objective function prioritizes lower sum of normalized m_scan
%         obj = -sum(m_scan, 'all') * dt_s; % Negative sign to prioritize lower sums
%     else
%         error('Invalid value for flag_scan_task');
%     end
%     
%     s_obj = s_obj + obj; % Sum of objective over time
% end

% V2 REFACTOR V1
% function [s_obj, obj] = calc_obj(...
%             m_f, m_bo, m_scan, ...
%             r_bo, r_fo, dt_s, s_obj, ...
%             n_x_e, n_y_e, c_f_s)
% 
%     % Generate active fire map
%     m_fo = (m_f == 3); % Simplified logical indexing
%     
%     % Convert scan map to environment map resolution
%     m_scan_env = zeros(n_x_e, n_y_e);
%     [rows, cols] = find(m_scan == 1); % Find coordinates of scanned cells
%     for k = 1:length(rows)
%         i = rows(k);
%         j = cols(k);
%         x_range = (c_f_s*(i-1) + 1):(c_f_s*i);
%         y_range = (c_f_s*(j-1) + 1):(c_f_s*j);
%         m_scan_env(x_range, y_range) = 1;
%     end
% 
%     % Inverse scan map - directly using logical NOT
%     m_scan_env_inv = ~m_scan_env;
% 
%     % Priority due to building and fire occupancy
%     m_P_bo = r_bo .* m_bo;
%     m_P_fo = r_fo .* m_fo;    
%     m_P = m_P_bo + m_P_fo;
% 
%     % Ignore already scanned cells
%     m_P = m_P .* m_scan_env_inv;
% 
%     % Objective function for current timestep
%     obj = sum(m_P, 'all') * dt_s;       
%     s_obj = s_obj + obj; % Sum of objective over time
% 
% end

% V1 - refactored
% function [s_obj, obj] = calc_obj(...
%             m_f, m_bo, m_scan, ...
%             r_bo, r_fo, dt_s, s_obj, ...
%             n_x_e, n_y_e, n_x_s, n_y_s, c_f_s)
% 
%     % Generate active fire map
%     m_fo          = m_f;
%     m_fo(m_fo==1) = 0;
%     m_fo(m_fo==2) = 0;
%     m_fo(m_fo==3) = 1;
%     m_fo(m_fo==4) = 0;
%     
%     % Convert scan map to environment map resolution
%     m_scan_env = zeros(n_x_e, n_y_e);
%     for i=1:n_x_s
%       for j=1:n_y_s
%         if m_scan(i,j) == 1
%           % Calculate range in m_scan_inv_env
%           x_min = c_f_s*(i-1) + 1;
%           x_max = x_min + c_f_s - 1;
%           y_min = c_f_s*(j-1) + 1;
%           y_max = y_min + c_f_s - 1;
%           % Set range as scanned
%           m_scan_env(x_min:x_max,y_min:y_max) = 1;
%         end
%       end
%     end
%     % Inverse scan map
%     m_scan_env_inv = ones(n_x_e, n_y_e) - m_scan_env;
%     % Priority due to building occupancy
%     m_P_bo = r_bo.*m_bo;
%     % Priority due to fire occupancy
%     m_P_fo = r_fo.*m_fo;    
%     % Priority map
%     m_P   = m_P_bo + m_P_fo;
%     % Ignore already scanned cells
%     m_P   = m_P.*m_scan_env_inv;
%     % Objective function for current timestep
%     obj   = sum(m_P, 'all')*dt_s;       
%     % Sum of objective over time
%     s_obj = s_obj + obj;
% end