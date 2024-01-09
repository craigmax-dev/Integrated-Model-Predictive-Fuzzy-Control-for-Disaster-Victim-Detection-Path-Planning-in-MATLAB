%% Function initialise_agent
% Initialise agent model

% V2

% CHANGELOG
% m_scan redefined - we can now choose if time since cell last scanned or binary
% cell scanned
% flag_scan_task added
% added m_victim_s
% Refactored function to use agent_model structure
% Added parameters for battery model

% TODO
% Better way to structure a_loc_hist?

function agent_model = initialise_agent_SIM_repeat(m_bo, m_dw_e, l_x_e, l_y_e)
  
  % Search map coarsen factors
  c_f_s  = [5, 5];

  % Search map building occupancy
  m_bo_s = func_coarsen(m_bo, c_f_s); 
  m_dw_s = func_coarsen(m_dw_e, c_f_s); 
  
  % Calculate victim map
  n_victim_max = 5;
  m_victim_s = generateVictimsMap(m_bo_s, n_victim_max);  
  
  % Search map dimensions
  n_x_s  = size(m_bo_s, 1);
  n_y_s  = size(m_bo_s, 2);
  
  % Search map cell lengths
  l_x_s     = c_f_s(1)*l_x_e;
  l_y_s     = c_f_s(2)*l_y_e;
  
  % Agent parameters
  n_a           = 2;                % Number of UAVs in simulation
  n_q           = 2;                % Queue length for UAV tasks
  v_as          = 5;                % UAV airspeed (m/s)
  a_t_trav      = zeros(n_a, 1);    % Time left to complete travel
  t_scan_m      = 0.1;              % Scan time per square metre
  a_task        = 2.*ones(n_a, 1);% Current task for UAVs
  a_loc         = [ 1, 1;
                    1, 2];          % Current locations of UAVs
  a_loc_hist    = [];
  for a = 1:n_a
    a_loc_hist(a,:) = [a_loc(a, 1), a_loc(a, 2), a, 0]; % Note: format is x-axis, y-axis, agent number, timestep number
  end  

  % Agent targets
  a_target        = nan(n_a, 2, n_q);
  a_target(:,:,1) = a_loc;
  
  a_t_scan  = zeros(n_a, 1);    % Time left to complete current scanning task
  for a = 1:n_a
      a_t_scan(a) = m_bo_s(a_loc(a, 1), a_loc(a, 2));
  end

  % Battery parameters
  m_recharge = zeros(n_x_s, n_y_s); % Recharge stations
  a_battery_level_i = 1e6.*ones(n_a, 1); % Fully charged battery level (s)
  a_battery_level = a_battery_level_i; % Current battery level (s)
  
  % Search map cell scan time
  t_scan_c    = t_scan_m*l_x_s*l_y_s;       % Scan time per cell
  m_scan      = zeros(n_x_s, n_y_s);        % Scan map
  m_scan_hist = zeros(n_x_s, n_y_s);  
  m_t_scan    = t_scan_c.*ones(n_x_s, n_y_s); % Scan time map (s) - time to scan each cell
  
  % Define agent objectives
  config = struct();
  config.weight_dw = 0;      % Weight for victims
  config.weight_fire = 0;      % Weight for victims
  config.weight_first_scan = 0;   % Weight for the first-time scan
  config.weight_repeat_scan = 0.1;  % Weight for repeat scans

  % Initialise priority map
  m_prior = calc_prior(m_bo_s, m_dw_s, m_scan, 0, config, m_victim_s);

  % Create agent structure with all parameters
  agent_model = struct('n_x_s', n_x_s, 'n_y_s', n_y_s, 'l_x_s', l_x_s, 'l_y_s', l_y_s, ...
                       'n_a', n_a, 'n_q', n_q, 'v_as', v_as, 'a_t_trav', a_t_trav, ...
                       't_scan_m', t_scan_m, 't_scan_c', t_scan_c, 'a_battery_level_i', a_battery_level_i, 'a_battery_level', a_battery_level, 'a_task', a_task, ... 
                       'a_loc', a_loc, 'a_loc_hist', a_loc_hist, 'a_target', a_target, 'a_t_scan', a_t_scan, ...
                       'm_prior', m_prior, 'm_recharge', m_recharge, 'm_scan', m_scan, 'm_scan_hist', m_scan_hist, 'm_t_scan', m_t_scan, ...
                       'm_bo_s', m_bo_s, 'm_dw_s', m_dw_s, 'm_victim_s', m_victim_s, ...
                       'config', config, 'c_f_s', c_f_s);
end