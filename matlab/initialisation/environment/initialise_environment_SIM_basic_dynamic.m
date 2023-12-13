%% Function initialise_environment
% Initialise environment model

% V2

% TODO
% how to handle c_f_s: used & defined both in initialise_agent and
% initialise_environment
% check difference between m_bo, m_s, m_p_ref
% add seed as input?

function [l_x_e, l_y_e, n_x_e, n_y_e, ...
  m_bo, m_s, m_f, m_bt, m_dw_e, m_p_ref, ...
  c_fs_1, c_fs_2, v_w, ang_w] = initialise_environment_SIM_basic_dynamic(dt_e, k)

  seed = 0;
  
  % Environment map dimensions
  n_x_e       = 100;
  n_y_e       = 100;

  % Create a 100 by 100 matrix of ones to represent an even distribution of buildings
  m_bo = ones(n_x_e, n_y_e);
  m_s = ones(n_x_e, n_y_e);
  m_p_ref = ones(n_x_e, n_y_e);

  % V2 define cell sizes
  l_x_e = 10;
  l_y_e = 10;
  
  % Wind model
  v_w         = 2;        % Wind speed (m/s)
  ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
  
  % Initialise fire map
  m_f       = m_s;
  m_f(1:2,1:2) = 3 * m_s(1:2,1:2);
  
  % Initialise burntime map
  m_bt        = zeros(n_x_e,n_y_e);
  
  % Fire model parameters
  c_fs_1        = 0.2;    % Wind constant 1 (for fire model)
  c_fs_2        = 0.2;    % Wind constant 2 (for fire model)
  
  %  Initialise fire maps
  [m_f, m_bt, m_dw_e] = model_environment(m_f, m_s, m_bo, m_bt, dt_e, k, seed, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2);

end

% V1

% function [l_x_e, l_y_e, n_x_e, n_y_e, ...
%   m_bo, m_s, m_f, m_bt, m_dw, m_p_ref, ...
%   c_fs_1, c_fs_2, v_w, ang_w] = initialise_environment_SIM_basic_dynamic(dt_e, k)
%     
%   % Create a 100 by 100 matrix of ones to represent an even distribution of buildings
%   m_p_in = ones(100, 100);
%   m_p_ref = ones(100, 100);
%   
%   % Environment map cell length (m)
%   l_c_e = 3;
% 
%   % Correct the call to calc_coarsenRatio by passing the size of m_p_in
%   % and the desired cell length l_c_e
%   [c_f_e, l_x_e, l_y_e] = calc_coarsenRatio(size(m_p_in), l_c_e);
% 
%   % Building occupancy map and structure map
%   % Call func_coarsen with the matrix input and coarsening factor
%   [m_bo, m_s] = func_coarsen(m_p_in, c_f_e);
%       
%   % Environment map dimensions
%   n_x_e       = size(m_bo,1);
%   n_y_e       = size(m_bo,2);
%   
%   % Wind model
%   v_w         = 2;        % Wind speed (m/s)
%   ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
%   
%   % Initialise fire map
%   m_f       = m_s;
%   m_f(1:2,1:2) = 3 * m_s(1:2,1:2);
%   
%   
%   % Initialise burntime map
%   m_bt        = zeros(n_x_e,n_y_e);
%   
%   % Fire model parameters
%   c_fs_1        = 0.2;    % Wind constant 1 (for fire model)
%   c_fs_2        = 0.2;    % Wind constant 2 (for fire model)
%   c_f_s         = [5, 5];
%   
%   %  Initialise fire maps
%   [m_f, ~, ~, ~, ...
%   m_bt, m_dw] = model_environment(...
%   m_f, [], [], [], m_s, m_bo, m_bt, ...
%   dt_e, k, 0, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, false);
% end