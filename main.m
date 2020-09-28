%% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

%% Assumptions 
% - Minimise risk to victims until the point at which they are rescued.
% Assumes victims are rescued quickly after detection and little extra risk
% to them during that time. Also doesn't account for additional factors
% which may be important such as equipment / process required to rescue
% victims.
% - Wind is assumed to be in fixed direction and velocity

%% Preprocessing

% MPC configuration
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
sim_name    = "";
exp_dir     = "results";

% Set up folder paths
addpath('functions', 'inputData', 'figures')

% Import building raster
buildingRaster      = 'data\maps\portAuPrince\portAuPrince_campeche.tif';
[m_p_in, m_p_ref]   = geotiffread(buildingRaster);
m_p_in              = m_p_in(50:450, 50:250);

% Time steps and counters
t       = 0;        % Current time (s)
t_f     = 6000;     % Simulation end time (s) - optional end condition
dt_s    = 5;        % Minimum step size (s)
dt_a    = 5;        % Agent step size (s)
dt_c    = 10;       % Control step size (s)
dt_f    = 60;       % Fire step size (s)
dt_mpc  = 1200;     % MPC step size (s)
dt_prog = 600;      % Progress report step size (s)
k       = 0;        % Discrete time step counter
k_a     = 0;        % Agent counter
k_c     = 0;        % Control counter
k_e     = 0;        % Fire counter
k_mpc   = 0;        % MPC counter
k_prog  = 0;        % Prog report counter
n_p     = 1;        % Prediction horizon (= n_p*dt_p)s

% Simulation end condition - "time" or "scan"
endCondition  = "scan"; 
finishFlag    = false;

% Objective function
obj           = 0;
obj_hist      = [];
s_obj         = 0;
s_obj_hist    = [];

% Quadcopter model
n_a           = 2;                % Number of UAVs in simulation
n_q           = 2;                % Queue length for UAV tasks
v_as          = 5;                % UAV airspeed (m/s)
a_t_trav      = zeros(n_a, 1);    % Time left to complete travel
a_t_trav_hist = zeros(1,2);
t_scan_m      = 0.1;              % Scan time per square metre
a_task        = 2.*ones(n_a, 1);% Current task for UAVs
a_loc         = [ 1, 1;
                  1, 2];          % Current locations of UAVs
a_loc_hist = NaN(1,4);
for a = 1:n_a
    a_loc_hist(a,:) = [a_loc(a, 1), a_loc(a, 2), a, t];
end

% UAV targets
a_target        = nan(n_a, 2, n_q);
a_target(:,:,1) = a_loc;

% Risk model
r_bo  = 0.5;      % Risk weighting due to building occupancy
r_fo  = 0.5;      % Risk weighting due to environmental fire

c_fs_1        = 0.2;    % Wind constant 1 (for fire model)
c_fs_2        = 0.2;    % Wind constant 2 (for fire model)

%% Environment models
% Environment map cell length (m)
l_c_e   = 3;
% Building occupancy map
[c_f_e, l_x_e, l_y_e] = coarsenRatio(m_p_ref, l_c_e);
[m_bo, m_r] = coarsen(m_p_in, c_f_e);
% Environment map dimensions
n_x_e       = size(m_bo,1);
n_y_e       = size(m_bo,2);
% Wind model
v_w         = 2;        % Wind speed (m/s)
ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
% Initialise fire map
m_f_i       = zeros(n_x_e, n_y_e);
m_f_i(80,1) = 3;
m_f_hist    = m_f_i;
% Initialise burntime map
m_bt        = zeros(n_x_e,n_y_e);
m_bt_hist   = m_bt;

%% Search models
c_f_search  = [5, 5]; % Search map coarsen factor from environment map
m_bo_search = coarsen(m_bo, c_f_search); 
n_x_s  = size(m_bo_search, 1);            % Scan map size in x direction
n_y_s  = size(m_bo_search, 2);            % Scan map size in y direction
l_x_s     = c_f_search(1)*l_x_e;        %
l_y_s     = c_f_search(2)*l_y_e;        %
t_scan_c    = t_scan_m*l_x_s*l_y_s;       % Scan time per cell
m_scan      = zeros(n_x_s, n_y_s);        % Scan map
% m_scan_hist = zeros(n_x_s, n_y_s);        % Scan map history
m_scan_hist = zeros(1,2);
m_dw_hist   = zeros(n_x_s, n_y_s);        % Downwind map history
m_t_scan    = t_scan_c.*ones(n_x_s, n_y_s); % Scan time map (s) - time to scan each cell
a_t_scan  = zeros(n_a, 1);    % Time left to complete current scanning task
for a = 1:n_a
    a_t_scan(a) = m_bo_search(a_loc(a, 1), a_loc(a, 2));
end

% Axes may not be entirely accurate as coarsening may remove some
% rows/columns from original map.

% Axes for dynamic environment states
ax_lat_env = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_e);
ax_lon_env = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_e);

% Axes for search map
ax_lat_scan = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_s);
ax_lon_scan = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_s);

% Time estimation
% Number of desired data points
n_prog_data = 100;
% Avg travel time
t_trav_avg = t_scan_c + l_x_s/v_as;
% Estimated sim time
t_sim_est = t_trav_avg * n_x_s * n_y_s / n_a;
% Save data time
dt_v = t_sim_est / n_prog_data;
ct_v = 0;

% Priority map 
negAtt            = NaN;  % Negative attraction for cells which don't need scanned
c_prior_building  = 1;    % Priority constant for building
c_prior_open      = 0.1;  % Priority constant for open space

% Calculate Priority map
m_prior = arrayfun(@(bo_search)(c_prior_building*bo_search + c_prior_open*(1-bo_search)), m_bo_search);

% % Fire outbreak - n fire outbreaks in random buildings.
% rng(1) % Seed to ensure consistency of initial firemap generation
% for i = 1:numel(m_bo)
%   if (m_bo(i) ~= 0)
%     m_f_i(i) = 1;
%   end
% end
% 
% while n_f_i > 0
%   coords = [randi([1 n_x_e]), randi([1 n_y_e])];
%   if m_bo(coords(1), coords(2)) > 0
%     m_f_i(coords(1), coords(2)) = 2;
%     n_f_i = n_f_i - 1;
%   end
% end

% Alternative fire map
m_f_i(numel(m_bo)) = 1;

% Initialise fire maps
[m_f, m_f_hist, m_bt, m_dw] = environmentModel(...
      m_f_i, m_f_hist, m_r, m_bo, m_bt, dt_f, k, n_x_e, n_y_e, ...
      v_w, ang_w, c_fs_1, c_fs_2, c_f_search);

% Generate FIS
[fisArray] = createFIS( n_a );

% MPC variables
fis_params = [];
for a = 1:n_a
  fis_params = [fis_params, fisArray(a).Outputs.MembershipFunctions.Parameters];
end
% FIS parameters history
fis_param_hist = fis_params;
% Initial parameters for simulat ion
ini_params = [];
for i = 1:n_p
  ini_params = [ini_params, fis_params];
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
  m_f, [], m_r, m_bo, m_bt, m_scan, m_t_scan, ...
  dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
  n_a, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
  a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
  k, negAtt, ...
  l_x_s, l_y_s, c_f_search, ...
  c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
  r_bo, r_fo);

% Optimisation options
optTermCond = 'MaxTime';
fminsearchOptions = optimset('Display','iter','PlotFcns',@optimplotfval);
gaOptions         = optimoptions('ga','Display','iter', 'PlotFcn', @gaplotbestf);
patOptions        = optimoptions('patternsearch','Display','iter', optTermCond, t_opt);
parOptions        = optimoptions('particleswarm','Display','iter', 'PlotFcn',@pswplotbestf);

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
elseif (dt_a >= t_scan_c)
  fprintf("ERROR: UAV airspeed lower than wind speed")
  return
end

%% Simulation
while finishFlag == false
  % Start timer
  t_sim = tic;
  %% MPC
  if mpc_active
    t_start = tic;    
    if k_mpc*dt_mpc <= t
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
  if k_c*dt_c <= t
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
  if k_a*dt_a <= t
    % Counter 
    k_a = k_a + 1;
    % Agent model
    [ m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, ...
      a_t_trav, a_t_trav_hist, a_t_scan] ...
        = agentModel( n_a, ...
        m_t_scan, m_scan, m_scan_hist, ...
        a_loc, a_loc_hist, a_task, a_target, ...
        a_t_trav, a_t_trav_hist, a_t_scan, ...
        l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, t, false);
  end
  t_agentActions = toc(t_start);
  
  %% Environment model
  t_start = tic;
  if k_e*dt_f <= t
    % Counter 
    k_e = k_e + 1;
    % Environment map
    [m_f, m_f_hist, m_bt, m_dw] = environmentModel(...
      m_f, m_f_hist, m_r, m_bo, m_bt, dt_f, k, n_x_e, n_y_e, ...
      v_w, ang_w, c_fs_1, c_fs_2, c_f_search, false);
  end 
  t_environment = toc(t_start);

  %% Store variables
  if ct_v*dt_v <= t
    ct_v = ct_v + 1;
    s_obj_hist(ct_v)    = s_obj;
    obj_hist(ct_v)      = obj;
  end

  %% Objective function evaluation
  [s_obj, obj]  = objEval(m_f, m_bo, m_scan, r_bo, r_fo, dt_s, s_obj, n_x_e, n_y_e, n_x_s, n_y_s, c_f_search);
  %% Advance timestep
  t = t + dt_s;
  k = k + 1;
    
  %% Progress report
  if k_prog * dt_prog <= t
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
if sim_name ~= ""
  folder = strcat(folder, '-', sim_name);
end

if fig_exp
  % Generate and export figures 
  plotData  = {  
    'm_dw_hist',        m_dw_hist,        false;    
    'm_f_hist',         m_f_hist,         false;
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
            dt_v, t, n_x_s, n_y_s, n_a, ct_v, fisArray);
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


%% Errata
%   %% MPC obj eval
%   if testObj
%     t_start = tic;
%     if ct_mpc_eval*dt_mpc <= t
%     % Counter 
%     ct_mpc_eval = ct_mpc_eval + 1;
%     % Generate initial parameters
%     obj_eval = mpcModel(ini_params, ...
%       fisArray, ...
%       m_f, m_r, m_bo, m_bt, m_scan, m_t_scan, ...
%       dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
%       n_a, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
%       a_loc, a_target, a_task, t_trav, a_t_scan, ...
%       k, negAtt, ...
%       l_x_s, l_y_s, c_f_search, ...
%       c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
%       r_bo, r_fo);
%     % Save objective
%     obj_hist_eval(ct_mpc_eval) = obj_eval;
%     end
%     t_testObj = toc(t_start);
%   end  

%   %% Record module execution times
%   if recordTime
%     fprintf(strcat("Execution time - MPC Module - ", num2str(t_MPC), "\n"));
%     fprintf(strcat("Execution time - testObj Module - ", num2str(t_testObj), "\n"));
%     fprintf(strcat("Execution time - testSensitivity Module - ", num2str(t_testSensitivity), "\n"));    
%     fprintf(strcat("Execution time - Path Planning Module - ", num2str(t_pathPlanning), "\n"));  
%     fprintf(strcat("Execution time - Agent Actions Module - ", num2str(t_agentActions), "\n"));  
%     fprintf(strcat("Execution time - Environment Module - ", num2str(t_environment), "\n"));
%   end

%   if test_obj_sensitivity
%     [s_obj, obj]  = objEval(m_f, m_bo, m_scan, r_bo, r_fo, dt_s, s_obj, n_x_e, n_y_e, n_x_s, n_y_s, c_f_search, obj_fun_scaling);        
%   else
%     [s_obj, obj]  = objEval(m_f, m_bo, m_scan, r_bo, r_fo, dt_s, s_obj, n_x_e, n_y_e, n_x_s, n_y_s, c_f_search);    
%   end

    % Changing way this is calculated
%     m_f_hist(:,:,ct_v)    = m_f;
%     m_scan_hist(:,:,ct_v) = m_scan;  
%     m_dw_hist(:,:,ct_v)   = m_dw;
%     m_bt_hist(:,:,ct_v)   = m_bt;

%   %% Sensitivity test
%   if test_obj_sensitivity
%     t_start = tic;
%     if ct_mpc_sens*dt_mpc <= t && ct_mpc_sens < ct_mpc_sens_fin
%       % Counter 
%       ct_mpc_sens = ct_mpc_sens + 1;
%       % Eval obj function over prediction horizon for range of MF parameters
%       for i = 1:n_sens_1
%         for j = 1:n_sens_2
%           for k = 1:n_sens_3
%             for l = 1:n_sens_4
%               % Generate parameter array
%               fis_params = [];
%               for a = 1:n_a
%                 fis_params = [fis_params, [p1(i), p2(j), p3(k), p4(l)]];
%               end 
%               sens_params = [];
%               for p = 1:n_p
%                 sens_params = [sens_params, fis_params];
%               end
%               % Evaluate objective function
%               obj = mpcModel( sens_params, ...
%               fisArray, ...
%               m_f, m_r, m_bo, m_bt, m_scan, m_t_scan, ...
%               dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
%               n_a, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
%               a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
%               k, negAtt, ...
%               l_x_s, l_y_s, c_f_search, ...
%               c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
%               r_bo, r_fo);
%               % Save objective
%               obj_hist_sens(i,j,k,l,ct_mpc_sens) = obj;
%             end
%           end
%         end
%       end
%     end
%     t_testSensitivity = toc(t_start);
%   end

    % Path planner
%     if test_fis_sensitivity
%       [a_target, fis_data] = pathPlanner(...
%         n_a, a_target, n_q, ...
%         n_x_s, n_y_s, l_x_s, l_y_s, ...
%         m_scan, m_t_scan, m_dw, m_prior, ...
%         fisArray, ...
%         a_t_trav, a_t_scan, ...
%         ang_w, v_as, v_w, ...
%         negAtt, test_fis_sensitivity, fis_data);
%     else
%       a_target = pathPlanner(...
%         n_a, a_target, n_q, ...
%         n_x_s, n_y_s, l_x_s, l_y_  s, ...
%         m_scan, m_t_scan, m_dw, m_prior, ...
%         fisArray, ...
%         a_t_trav, a_t_scan, ...
%         ang_w, v_as, v_w, ...
%         negAtt, test_fis_sensitivity);    
%     end
