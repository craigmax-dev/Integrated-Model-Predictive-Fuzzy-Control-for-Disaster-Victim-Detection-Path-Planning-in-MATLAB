%% Function initialise_agent
% Initialise agent model

% V2

% CHANGELOG
% m_scan redefined - we can now choose if time since cell last scanned or binary
% cell scanned
% flag_scan_task added
% added m_victim_scan

% TODO
% replace m_t_scan with single value t_scan (same for all cells)
% check n_x_s etc handled correctly
% c_f_s as function output
% remove m_dw - should be unnecessary

function [n_x_s, n_y_s, l_x_s, l_y_s, n_a, n_q, v_as, a_t_trav, ...
  t_scan_m, t_scan_c, a_task, a_loc, a_target, a_t_scan, ...
  m_scan, m_t_scan, m_bo_s, m_dw, m_victim_scan, config, c_f_s] = initialise_agent_SIM_repeat(m_bo, m_dw_e, l_x_e, l_y_e)
  
  % Search map coarsen factors
  c_f_s  = [5, 5];

  % Search map building occupancy
  m_bo_s = func_coarsen(m_bo, c_f_s); 
  m_dw = func_coarsen(m_dw_e, c_f_s); 
  
  % Calculate victim map
  n_victim_max = 5;
  m_victim_scan = generateVictimsMap(m_bo_s, n_victim_max);  
  
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
  % Agent targets
  a_target        = nan(n_a, 2, n_q);
  a_target(:,:,1) = a_loc;
  
  a_t_scan  = zeros(n_a, 1);    % Time left to complete current scanning task
  for a = 1:n_a
      a_t_scan(a) = m_bo_s(a_loc(a, 1), a_loc(a, 2));
  end
  
  % Search map cell scan time
  t_scan_c    = t_scan_m*l_x_s*l_y_s;       % Scan time per cell
  m_scan      = zeros(n_x_s, n_y_s);        % Scan map
  m_t_scan    = t_scan_c.*ones(n_x_s, n_y_s); % Scan time map (s) - time to scan each cell
  
  % Define agent objectives
  config = struct();
  config.weight_bo = 0;           % Weight for building occupancy map
  config.weight_fo = 0;           % Weight for active fires
  config.weight_victims = 0;      % Weight for victims
  config.weight_first_scan = 0;   % Weight for the first-time scan
  config.weight_repeat_scan = 0.1;  % Weight for repeat scans  

end