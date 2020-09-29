%% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

%% Simulation Settings
mpc_active            = false;
% Solver options: fmincon, ga, particleswarm, patternsearch
solver                = "patternsearch";
% Optimisation time limit for solver
t_opt                 = 60;
% Tests 
test_fis_sensitivity  = false;
test_obj_sensitivity  = false;
test_solvers          = false;
% Test variables
fis_data = [];
% Data export configuration
data_exp    = true;
fig_exp     = true;
exp_folder  = "BUGFIX01";
exp_dir     = "results";
% Set up folder paths
addpath('functions', 'inputData', 'figures')

%% Time steps and counters
t       = 0;        % Current time
dt_s    = 5;        % Simulation step size
dk_a    = 1;        % Agent step size
dk_c    = 2;       % Control step size
dk_e    = 12;       % Fire step size
dk_mpc  = 240;     % MPC step size
dk_prog = 120;      % Progress report step size
dt_a    = dk_a*dt_s;
dt_c    = dk_c*dt_s;
dt_e    = dk_e*dt_s;
dt_mpc  = dk_mpc*dt_s;
% dt_prog = dk_prog*dt_s;
k       = 0;        % Discrete time step counter
k_a     = 0;        % Agent counter
k_c     = 0;        % Control counter
k_e     = 0;        % Fire counter
k_mpc   = 0;        % MPC counter
k_prog  = 0;        % Prog report counter

% Simulation end condition - "time" or "scan"
endCondition  = "scan"; 
finishFlag    = false;

% Objective function
obj           = 0;
s_obj         = 0;

% Risk model
r_bo  = 0.5;      % Risk weighting due to building occupancy
r_fo  = 0.5;      % Risk weighting due to environmental fire

%% Environment models
% Import building raster
buildingRaster      = 'data\maps\portAuPrince\portAuPrince_campeche.tif';
[m_p_in, m_p_ref]   = geotiffread(buildingRaster);
m_p_in              = m_p_in(50:450, 50:250);
% Environment map cell length (m)
l_c_e   = 3;
% Building occupancy map and structure map
[c_f_e, l_x_e, l_y_e] = coarsenRatio(m_p_ref, l_c_e);
[m_bo, m_s] = coarsen(m_p_in, c_f_e);
% Environment map dimensions
n_x_e       = size(m_bo,1);
n_y_e       = size(m_bo,2);
% Wind model
v_w         = 2;        % Wind speed (m/s)
ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
% Initialise fire map
m_f_i       = zeros(n_x_e, n_y_e);
m_f_i(80,1) = 3;
% Initialise burntime map
m_bt        = zeros(n_x_e,n_y_e);
% Fire model parameters
c_fs_1        = 0.5;    % Wind constant 1 (for fire model)
c_fs_2        = 0.5;    % Wind constant 2 (for fire model)

%% Agent models
% Search map coarsen factors
c_f_s  = [5, 5];
% Search map building occupancy
m_bo_s = coarsen(m_bo, c_f_s); 
% Search map dimensions
n_x_s  = size(m_bo_s, 1);
n_y_s  = size(m_bo_s, 2);
% Search map cell lengths
l_x_s     = c_f_s(1)*l_x_e;
l_y_s     = c_f_s(2)*l_y_e;
% Search map cell scan time
t_scan_c    = t_scan_m*l_x_s*l_y_s;       % Scan time per cell
m_scan      = zeros(n_x_s, n_y_s);        % Scan map
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
% Unsorted
m_t_scan    = t_scan_c.*ones(n_x_s, n_y_s); % Scan time map (s) - time to scan each cell
a_t_scan  = zeros(n_a, 1);    % Time left to complete current scanning task
for a = 1:n_a
    a_t_scan(a) = m_bo_s(a_loc(a, 1), a_loc(a, 2));
end

%% Path planner models
% Priority map 
c_prior_building  = 1;    % Priority constant for building
c_prior_open      = 0.1;  % Priority constant for open space
% Calculate Priority map
m_prior = arrayfun(@(bo_search)(c_prior_building*bo_search + c_prior_open*(1-bo_search)), m_bo_s);
% Generate FIS
[fisArray] = createFIS( n_a );
% Initial parameters for simulat ion
ini_params = [];
for i = 1:n_p
  ini_params = [ini_params, fis_params];
end

%% MPC models
% Prediction horizon
n_p = 1;
% Optimisation variables
fis_params = [];
for a = 1:n_a
  fis_params = [fis_params, fisArray(a).Outputs.MembershipFunctions.Parameters];
end
% Constraints
A       = [];
b       = [];
Aeq     = [];
beq     = [];
lb      = [];
ub      = [];
nonlcon = [];
nvars = size(ini_params, 2);
% Function handle
fun = @(params)mpcModel(params, ...
  fisArray, test_fis_sensitivity, ...
  m_f, [], m_s, m_bo, m_bt, m_scan, m_t_scan, ...
  dk_a, dk_c, dk_e, dk_mpc, dt_s, k, ...
  n_a, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
  a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
  l_x_s, l_y_s, c_f_s, ...
  c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
  r_bo, r_fo);
% Solver options
optTermCond       = 'MaxTime';
fminsearchOptions = optimset('Display','iter','PlotFcns',@optimplotfval);
gaOptions         = optimoptions('ga','Display','iter', 'PlotFcn', @gaplotbestf);
patOptions        = optimoptions('patternsearch','Display','iter', optTermCond, t_opt);
parOptions        = optimoptions('particleswarm','Display','iter', 'PlotFcn',@pswplotbestf);

%% Plotting variables
% Axes may not be entirely accurate as coarsening may remove some
% rows/columns from original map.
% Axes for dynamic environment states
ax_lat_env = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_e);
ax_lon_env = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_e);
% Axes for search map
ax_lat_scan = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_s);
ax_lon_scan = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_s);
% History plots
obj_hist    = [];
s_obj_hist  = [];
t_hist      = [];
m_f_hist    = m_f_i;
m_bt_hist   = m_bt;
a_loc_hist    = [];
for a = 1:n_a
  a_loc_hist(a,:) = [a_loc(a, 1), a_loc(a, 2), a, t];
end
m_scan_hist = zeros(1,2);
m_dw_hist   = zeros(n_x_s, n_y_s);        % Downwind map history
fis_param_hist = fis_params;

%% Simulation variables
% Time estimation
% Number of desired data points
n_prog_data = 100;
% Avg travel time
k_trav_avg = (t_scan_c + l_x_s/v_as)/dt_s;
% Estimated sim time
k_sim_est = k_trav_avg * n_x_s * n_y_s / n_a;
% Save data time
dk_v = k_sim_est / n_prog_data;
ct_v = 0;

%% Test setup

% Objective function sensitivity test setup
if test_obj_sensitivity
  p1_i = fis_params(1);
  p2_i = fis_params(2);
  p3_i = fis_params(3);
  p4_i = fis_params(4);
  obj_hist_eval   = [];
  obj_hist_sens   = [];
  ct_mpc_eval     = 0;
  ct_mpc_sens     = 0; 
  ct_mpc_sens_fin = 2;

  % Text Variables
  % Check these rangeps work properly
  n_sens_1  = 3;
  n_sens_2  = 3;
  n_sens_3  = 3;
  n_sens_4  = 3;
  r_sens    = 1;
  p1        = p1_i*linspace(1-r_sens, 1+r_sens, n_sens_1);
  p2        = p2_i*linspace(1-r_sens, 1+r_sens, n_sens_2);
  p3        = p3_i*linspace(1-r_sens, 1+r_sens, n_sens_3);
  p4        = p4_i*linspace(1-r_sens, 1+r_sens, n_sens_4);
end

%% Error checking
if (v_w >= v_as)
  fprintf("ERROR: UAV airspeed lower than wind speed")
  return
elseif (dk_a >= t_scan_c)
  fprintf("ERROR: UAV airspeed lower than wind speed")
  return
end

% Initialise fire maps
[m_f, m_f_hist, m_bt, m_dw] = environmentModel(...
      m_f_i, m_f_hist, m_s, m_bo, m_bt, dt_e, k, n_x_e, n_y_e, ...
      v_w, ang_w, c_fs_1, c_fs_2, c_f_s);

%% Simulation
while finishFlag == false
  % Start timer
  t_sim = tic;
  %% MPC
  if mpc_active
    t_start = tic;    
    if k_mpc*dk_mpc <= t
      % For reproducibility
      rng default;
      % Optimisation
      if solver == "fminsearch"
        [mpc_params, fval] = fminsearch(fun, ini_params, fminsearchOptions);
      elseif solver == "ga"
        [mpc_params,fval] = ga(fun, nvars, A, b, Aeq, beq, lb, ub, nonlcon, gaOptions);   
      elseif solver == "patternsearch"
        [mpc_params,fval] = patternsearch(fun, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, patOptions);   
      elseif solver == "particleswarm"
        [mpc_params,fval] = particleswarm(fun, nvars, lb, ub, parOptions);   
      end
      % Update FIS Parameters
      for a=1:n_a
        range       = 1 + (a - 1) * 4;
        fis_params  = mpc_params(range:range+3);
        fisArray(a).Outputs.MembershipFunctions.Parameters = fis_params;
      end
      % Update initial guess
      ini_params = mpc_params;
      % Record new parameters
      fis_param_hist = [fis_param_hist; mpc_params(1:n_a*4)];
      % Counter 
      k_mpc = k_mpc + 1;
    end
    t_MPC = toc(t_start);
  end
  
  %% Path planning
  t_start = tic;
  if k_c*dk_c <= k
    % Counter
    k_c = k_c + 1;
    % Path planner
    a_target = pathPlanner(...
      n_a, a_target, n_q, ...
      n_x_s, n_y_s, l_x_s, l_y_s, ...
      m_scan, m_t_scan, m_dw, m_prior, ...
      fisArray, ...
      a_t_trav, a_t_scan, ...
      ang_w, v_as, v_w, test_fis_sensitivity); 
  end
  t_pathPlanning = toc(t_start);

  %% Agent actions
  t_start = tic;
  if k_a*dk_a <= k
    % Counter 
    k_a = k_a + 1;
    % Agent model
    [ m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, ...
      a_t_trav, a_t_scan] ...
        = agentModel( n_a, ...
        m_t_scan, m_scan, m_scan_hist, ...
        a_loc, a_loc_hist, a_task, a_target, ...
        a_t_trav, a_t_scan, ...
        l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, t, false);
  end
  t_agentActions = toc(t_start);
  
  %% Environment model
  t_start = tic;
  if k_e*dk_e <= k
    % Counter 
    k_e = k_e + 1;
    % Environment map
    [m_f, m_f_hist, m_bt, m_dw] = environmentModel(...
      m_f, m_f_hist, m_s, m_bo, m_bt, dt_e, k, n_x_e, n_y_e, ...
      v_w, ang_w, c_fs_1, c_fs_2, c_f_s, false);
  end
  t_environment = toc(t_start);

  %% Store variables
  if ct_v*dk_v <= k
    ct_v = ct_v + 1;
    t_hist(ct_v) = ct_v*dk_v*dt_s;
    s_obj_hist(ct_v)    = s_obj;
    obj_hist(ct_v)      = obj;
  end

  %% Objective function evaluation
  [s_obj, obj]  = objEval(m_f, m_bo, m_scan, r_bo, r_fo, dt_s, s_obj, n_x_e, n_y_e, n_x_s, n_y_s, c_f_s);
  %% Advance timestep
  t = t + dt_s;
  k = k + 1;
    
  %% Progress report
  if k_prog * dk_prog <= t
    progReport(endCondition, t, t_f, m_scan, n_x_s, n_y_s);
    k_prog = k_prog + 1;
  end
  
  %% Check end condition
  [finishFlag] = simEndCondition(endCondition, t, t_f, m_scan, n_x_s, n_y_s);
end

% Simulation time  
t_end = toc(t_sim);

%% Postprocessing

% % Additional maps
% m_prior_hist = (ones(size(m_scan_hist)) - m_scan_hist).*m_prior;

% Generate folder name
dateTime = datestr(now,'yyyy-mm-dd-HH-MM');
folder = strcat(dateTime, '-', exp_folder);

if fig_exp
  % Generate and export figures 
  plotData  = {  
    'm_dw_hist',        m_dw_hist,        false;    
    'm_f_hist',         m_f_hist,         true;
    'm_scan_hist',      m_scan_hist,      false;
    'UAV_loc_hist',     a_loc_hist,       true;
    's_obj_hist',       s_obj_hist,       true;
    'obj_hist',         obj_hist,         true;
    'm_bo',             m_bo,             true;
    'fis',              fisArray,         true;
    'm_prior',          m_prior,          true};

%     'm_prior_hist',     m_prior_hist,     false
  
  if test_obj_sensitivity
    plotData = [plotData; {'obj_hist_sens', obj_hist_sens, true}];  
  end
  
  if mpc_active
    plotData = [plotData; {'fis_param_hist', fis_param_hist, true}]; 
  end

  plotResults( plotData, exp_dir, folder, ...
            ax_lon_env, ax_lat_env, ax_lon_scan, ax_lat_scan, ...
            dk_v, t, n_x_s, n_y_s, n_a, ct_v, fisArray);
end
        
% Export data
if data_exp
  % Save working directory path
  work_dir = pwd;
  % Change to save directory
  cd(exp_dir); 
  % Save workspace  
  mkdir(folder);
  cd(folder);
  save(folder);
  % Go back to working directory
  cd(work_dir);
end
