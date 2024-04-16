%% Function initialise_environment
% Initialise environment model

% V2
function environment_model = i_env_basic_no_dynamics(config)
  
  % Environment map dimensions
  n_x_e       = 40;
  n_y_e       = 40;

  % Create a matrix of ones to represent an even distribution of buildings
  m_bo = 0.5 .* ones(n_x_e, n_y_e);
  m_s = ones(n_x_e, n_y_e);
  m_p_ref = ones(n_x_e, n_y_e);

  % V2 define cell sizes
  l_x_e = 10;
  l_y_e = 10;
  
  % Wind speed (m/s)
  v_w         = 0;        

  % Wind direction angles (in radians) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
  % North (N): 0 or 2*pi
  % Northeast (NE): pi/4
  % East (E): pi/2
  % Southeast (SE): 3*pi/4
  % South (S): pi or -pi
  % Southwest (SW): -3*pi/4
  % West (W): -pi/2
  % Northwest (NW): -pi/4 or 7*pi/4
  ang_w       = -pi/4;

  % Initialise fire map
  m_f       = m_s;
  
  % Initialise burntime map
  m_bt        = zeros(n_x_e,n_y_e);
  m_dw_e      = zeros(n_x_e,n_y_e);
  
  % Fire model parameters
  c_fs_1        = 0.2;    % Wind constant 1 (for fire model)
  c_fs_2        = 0.2;    % Wind constant 2 (for fire model)

  t_i = 120;  % Ignition time
  t_b = 600;  % Burnout time
  r_w = 3;
  c_wm_1 = 0.1;
  c_wm_2 = 0.1;
  c_wm_d  = 0.4;
  
  % Calculate the total number of steps needed for the pre-computation phase
  % assuming one prediction step
  prediction_duration = config.dk_pred * config.dt_s;
  total_time = config.t_f + prediction_duration;
  total_steps = ceil(total_time / config.dt_e);  
  
  % Initialize m_f_series and m_dw_e_series in the environment_model
  m_f_series = zeros(n_x_e, n_y_e, total_steps);
  m_dw_e_series = zeros(n_x_e, n_y_e, total_steps);
    
  % Create environment structure with all parameters
  environment_model = struct('l_x_e', l_x_e, 'l_y_e', l_y_e, 'n_x_e', n_x_e, 'n_y_e', n_y_e, ...
  'm_bo', m_bo, 'm_s', m_s, 'm_f', m_f, 'm_bt', m_bt, 'm_dw_e', m_dw_e, 'm_p_ref', m_p_ref, ...
  'c_fs_1', c_fs_1, 'c_fs_2', c_fs_2, 'v_w', v_w, 'ang_w', ang_w, 't_i', t_i, 't_b', t_b, 'r_w', r_w, ...
  'c_wm_1', c_wm_1, 'c_wm_2', c_wm_2, 'c_wm_d', c_wm_d, 'm_f_series', m_f_series, 'm_dw_e_series', m_dw_e_series);


  %  Initialise fire maps
  environment_model = model_environment(environment_model, config.k_e, config.dt_e);

end

