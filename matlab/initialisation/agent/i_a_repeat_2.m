%% Function initialise_agent
% Initialise agent model

% V2
function agent_model = i_a_repeat_2(environment_model, config)
  
  % Search map building occupancy
  m_bo_s = func_coarsen(environment_model.m_bo, config.c_f_s); 
  m_dw_s = func_coarsen(environment_model.m_dw_e, config.c_f_s); 
  m_f_s = func_coarsen(environment_model.m_f, config.c_f_s); 
  
  % Calculate victim map
  n_victim_max = 5; % Max number of victims per cell
  m_victim_s = generateVictimsMap(m_bo_s, n_victim_max);  
  
  % Search map dimensions
  n_x_s  = size(m_bo_s, 1);
  n_y_s  = size(m_bo_s, 2);
  
  % Search map cell lengths
  l_x_s     = config.c_f_s(1)*environment_model.l_x_e;
  l_y_s     = config.c_f_s(2)*environment_model.l_y_e;

  % Battery parameters
  m_recharge = zeros(n_x_s, n_y_s); % Recharge stations: NOTE - recharge stations not implemented 
  a_battery_level_i = [4e4; 4e4]; % Fully charged battery level (s)
  a_battery_level = a_battery_level_i; % Current battery level (s)
  t_recharge = 100; % Time to recharge agent: NOTE - recharge stations not implemented 
  
  % Agent parameters
  n_a           = 2;                % Number of UAVs in simulation
  v_as          = 5;               % UAV airspeed (m/s)
  a_t_trav      = zeros(n_a, 1);    % Time left to complete travel
  t_scan_m      = 0.01;             % Scan time per square metre (s)
  a_task        = 2.*ones(n_a, 1);  % Current task for UAVs
  a_loc         = [ 1, 1;   
                    1, 2];          % Start locations of UAVs


  % Search map cell scan time
  t_scan_c    = t_scan_m*l_x_s*l_y_s;       % Scan time per cell
  m_scan      = zeros(n_x_s, n_y_s);        % Scan map
  m_prior      = zeros(n_x_s, n_y_s);        % Priority map
  m_scan_hist = zeros(n_x_s, n_y_s);  
  m_t_scan    = t_scan_c.*ones(n_x_s, n_y_s); % Scan time map (s) - time to scan each cell
  sensor_accuracy = 0.9; % [0 1]

  % Agent targets
  n_q             = 2;
  a_target        = ones(n_a, 2, n_q);
  a_target(:,:,1) = a_loc;
  
  a_t_scan  = zeros(n_a, 1);    % Time left to complete current scanning task
  for a = 1:n_a
      a_t_scan(a) = m_bo_s(a_loc(a, 1), a_loc(a, 2));
  end
  
  % Max response time calculation
  maxResponseTime = calc_maxResponseTime(l_x_s, l_y_s, n_q, n_x_s, n_y_s, t_scan_c, v_as);
  
 
  % Create agent structure with all parameters
  agent_model = struct('n_x_s', n_x_s, 'n_y_s', n_y_s, 'l_x_s', l_x_s, 'l_y_s', l_y_s, ...
                       'n_a', n_a, 'n_q', n_q, 'v_as', v_as, 'a_t_trav', a_t_trav, ...
                       't_scan_m', t_scan_m, 't_scan_c', t_scan_c, 't_recharge', t_recharge, 'a_battery_level_i', a_battery_level_i, 'a_battery_level', a_battery_level, 'a_task', a_task, ... 
                       'a_loc', a_loc, 'a_target', a_target, 'a_t_scan', a_t_scan, ...
                       'm_prior', m_prior, 'm_recharge', m_recharge, 'm_scan', m_scan, 'm_scan_hist', m_scan_hist, 'm_t_scan', m_t_scan, ...
                       'm_bo_s', m_bo_s, 'm_dw_s', m_dw_s, 'm_f_s', m_f_s, 'm_victim_s', m_victim_s, 'maxResponseTime', maxResponseTime, 'sensor_accuracy', sensor_accuracy);
end
