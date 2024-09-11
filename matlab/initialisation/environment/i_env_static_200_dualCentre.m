%% Function initialise_environment
% Initialise environment model

% V2
function environment_model = i_env_static_200_dualCentre(config)

  % Environment map dimensions
  n_x_e = 200;
  n_y_e = 200;

  % Define PDF
  pdf1 = @(x, y) reshape(mvnpdf([x(:) y(:)], [0.3, 0.3], [0.05, 0.02]), size(x));
  pdf2 = @(x, y) reshape(mvnpdf([x(:) y(:)], [0.7, 0.7], [0.05, 0.02]), size(x));

  % Define the list of PDFs
  pdfs = {pdf1, pdf2};

  % Define the weights for each PDF
  weights = [0.5, 0.5];

  % Define a random seed for reproducibility
  seed = 123;

  % Generate the matrix from PDFs
  pdf_matrix = generateMatrixFromPDFs([n_x_e, n_y_e], pdfs, weights, seed);

  % Generate the Brown noise matrix
  fluctuation = 0.0001; % Define the fluctuation level for Brown noise
  brown_noise_matrix = generateBrownNoiseMatrix(n_x_e, n_y_e, fluctuation, seed);

  % Combine the PDF matrix and Brown noise matrix
  m_bo = pdf_matrix + brown_noise_matrix;

  % Normalize the resulting matrix to have values between 0 and 1
  m_bo = (m_bo - min(m_bo(:))) / (max(m_bo(:)) - min(m_bo(:)));

  % Create a matrix of ones to represent an even distribution of buildings
  m_s = ones(n_x_e, n_y_e);
  m_p_ref = ones(n_x_e, n_y_e);

  % V2 define cell sizes
  l_x_e = 10;
  l_y_e = 10;
  
  % Wind speed (m/s)
  v_w = 1;

  % Wind direction angles (in radians) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
  ang_w = -pi/4;

  % Initialise fire map
  m_f = m_s;
  
  % Initialise burntime map
  m_bt = zeros(n_x_e, n_y_e);
  m_dw_e = zeros(n_x_e, n_y_e);
  
  % Fire model parameters
  c_fs_1 = 0.2;    % Wind constant 1 (for fire model)
  c_fs_2 = 0.2;    % Wind constant 2 (for fire model)

  t_i = 120;  % Ignition time
  t_b = 600;  % Burnout time
  r_w = 3;
  c_wm_1 = 0.1;
  c_wm_2 = 0.1;
  c_wm_d = 0.4;
  
  % Calculate the total number of steps needed for the pre-computation phase
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

  % Initialise fire maps
  environment_model = model_environment(environment_model, config.k_e, config.dt_e);

end