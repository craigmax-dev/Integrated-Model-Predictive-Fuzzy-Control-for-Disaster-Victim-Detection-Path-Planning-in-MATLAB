%% Function initialise_environment
% Initialise environment model

% V2
% CHANGELOG
% Refactor: structures

% TODO
% - replace c_f_s with single unit variable
% check difference between m_bo, m_s, m_p_ref

function environment_model = initialise_environment_SIM_basic_no_dynamic()
  
  % Environment map dimensions
  n_x_e       = 40;
  n_y_e       = 40;

  % Create a matrix of ones to represent an even distribution of buildings
  m_bo = ones(n_x_e, n_y_e);
  m_s = ones(n_x_e, n_y_e);
  m_p_ref = ones(n_x_e, n_y_e);

  % V2 define cell sizes
  l_x_e = 10;
  l_y_e = 10;
  
  % Wind model
  v_w         = 0;        % Wind speed (m/s)
  ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
  
  % Initialise fire map
  m_f       = m_s;
  
  % Initialise burntime map
  m_bt        = zeros(n_x_e,n_y_e);
  m_dw_e      = zeros(n_x_e,n_y_e);
  
  % Fire model parameters
  c_fs_1        = 0.2;    % Wind constant 1 (for fire model)
  c_fs_2        = 0.2;    % Wind constant 2 (for fire model)
  
  % Create environment structure with all parameters
  environment_model = struct('l_x_e', l_x_e, 'l_y_e', l_y_e, 'n_x_e', n_x_e, 'n_y_e', n_y_e, ...
  'm_bo', m_bo, 'm_s', m_s, 'm_f', m_f, 'm_bt', m_bt, 'm_dw_e', m_dw_e, 'm_p_ref', m_p_ref, ...
  'c_fs_1', c_fs_1, 'c_fs_2', c_fs_2, 'v_w', v_w, 'ang_w', ang_w);

  %  Initialise fire maps
  environment_model = model_environment(environment_model);          

end

