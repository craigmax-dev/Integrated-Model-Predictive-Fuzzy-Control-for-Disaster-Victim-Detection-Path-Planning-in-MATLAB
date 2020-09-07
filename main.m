%% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

%% To do
% - Selection criteria for path-planning algorithm (e.g. 100 nearest cells)
% - Code management with git - https://nl.mathworks.com/help/simulink/ug/push-and-fetch-files-with-git.html
% - Conversion to python
% - Full visual analysis of all matrices calculated in simulation
% - Test results for different combinations of FIS parameters
% - clean up path plot
% - test different options for FIS fire input
% - plot / compare optimal FIS parameters over simulation
% - plot / compare optimal function value from solver 

%% Bugs
% - add testScaling functionality - currently not working

%% Syntax notes
% - Change terminology - fire model to envionment model
% - n_UAV to n_a, UAV to agent, across project

%% Assumptions 
% - Minimise risk to victims until the point at which they are rescued.
% Assumes victims are rescued quickly after detection and little extra risk
% to them during that time. Also doesn't account for additional factors
% which may be important such as equipment / process required to rescue
% victims.
% - Wind is assumed to be in fixed direction and velocity

%% Recommended Future work
% Add earthquake data to model - https://www.gislounge.com/haiti-maps-and-gis-data-resources
% Implement error checking / code termination condition script.

%% Change log
% Changed FIS output surface to decrease with time
% Implemented modular code design
% Implemented new code styling standard across project
% Added fire component to victim risk calculation
% - constant weighting added while fire is active
% Designed input for FIS to characterise fire spread
% - downwindfire proximity - linear function of distance and windspeed related to nearest active fire
% Implemented fire spread probability matrix based on wind speed.
% Fixed bug where fuzzy rules were not being fired.
% Plot changes:
% - animated plots
% - added lat/lon to axes
% - multiplot
% Implemented separate timesteps for all components of the simulation - MPC,
% FIS, Control, Dynamics.
% Implemented UAV actions independent of others.
% Implemented model for scan cell priority - constant function of building area
% coverage
% Implemented function to save figures and simulation data to specified folder.
% Implemented simulation progress report
% 30/06/2020 - Get simulation working for UAV's without MPC part
% 30/06/2020 - Fixed objective function evaluation
% 30/06/2020 - UAV_target initialised at start of simulation
% 01/07/2020 - Changed order of script execution in main
% 01/07/2020 - Reorganised counters
% 02/07/2020 - Added MPC model to simulation
% 03/07/2020 - Changed fmincon to fminsearch
% 04/07/2020 - Added plotting of optimisation
% 17/07/2020 - Scaling problem investigation
% - Objective function parts
% - Fuzzy inputs and associated output
% 17/07/2020 - Changed exportData function - save all workspace variables
% 20/07/2020 - Implemented test logic
% 30/07/2020 - Removed m_att to simplify code
% 30/07/2020 - add time measurement to all modules of simulation
% 30/07/2020 - created time measurement function
% 30/07/2020 - replaced nested sum with sum(a,'all') across all functions
% 01/08/2020 - bugfix - m_dw not coarsened to search map size
% 04/08/2020 - added error checking script in preamble
% 04/08/2020 - added simulation time estimation
% 04/08/2020 - added choice of number of datapoints over simulation
% 08/08/2020 - bugfix - fixed error with wrong dimention m_att being passed to
% taskAssignment function
% 10/08/2020 - fixed m_dw calculation
% 10/08/2020 - separated coarsen and coarsenFactor calculation
% 12/08/2020 - changed recording of m_scan and m_scan_hist
% 14/08/2020 - fixed saving simulation results
% 14/08/2020 - added plotting of testSensitivity results
% 15/08/2020 - added patternsearch and particleswarm options for MPC solver
% 17/08/2020 - cleaned up export folder generation and add sim name
% 18/08/2020 - created script for comparison of results from multiple
% simulations


%% Preprocessing

% Simulation logic
runMPC          = false;
testSensitivity = false;
testScaling     = false;  
testObj         = false;
recordTime      = false;
optFunction     = "particleswarm";

% Data export settings
data_exp    = true;
fig_exp     = true;
exp_folder  = "FISPARAM";
sim_name    = "";
exp_saveDir = "test";

% Set up folder paths
addpath('functions', 'inputData', 'outputData', 'figures')

% Import building raster
buildingRaster      = 'inputData\maps\portAuPrince\portAuPrince_campeche.tif';
[m_p_in, m_p_ref]   = geotiffread(buildingRaster);
m_p_in              = m_p_in(50:450, 50:250);

% Time steps and counters
t       = 0;        % Current time (s)
t_f     = 6000;     % Simulation end time (s) - optional end condition
k       = 1;        % Discrete time step counter
dt_s    = 5;        % Minimum step size (s)
dt_a    = 5;        % Agent step size (s)
dt_c    = 10;       % Control step size (s)
dt_f    = 60;       % Fire step size (s)
dt_mpc  = 1200;     % MPC step size (s)
dt_prog = 600;      % Progress report step size (s)
ct_a    = 0;        % Agent counter
ct_c    = 0;        % Control counter
ct_f    = 0;        % Fire counter
ct_mpc  = 0;        % MPC counter
ct_p    = 0;        % Prediction counter
ct_prog = 0;        % Prog report counter

n_p     = 1;        % Prediction horizon (= n_p*dt_p)s

% Simulation end condition - "time" or "scan"
endCondition  = "scan"; 
finishFlag    = false;

% Objective function
obj           = 0;
obj_hist      = obj;
s_obj         = 0;
s_obj_hist    = s_obj;

% Quadcopter model
n_UAV         = 2;                % Number of UAVs in simulation
l_queue       = 2;                % Queue length for UAV tasks
v_as_UAV      = 5;                % UAV airspeed (m/s)
t_travel_UAV  = zeros(n_UAV, 1);  % Time left to complete travel
t_scan_m      = 0.1;              % Scan time per square metre
UAV_task      = 2.*ones(n_UAV, 1);% Current task for UAVs
UAV_loc       = [ 1, 1;
                  1, 2];          % Current locations of UAVs
UAV_loc_hist = NaN(1,4);
for UAV= 1:n_UAV
    UAV_loc_hist(UAV,:) = [UAV_loc(UAV, 1), UAV_loc(UAV, 2), UAV, t];
end

% UAV targets
UAV_target    = nan(n_UAV, 2, l_queue);
UAV_target(:,:,1) = UAV_loc;

% Risk model
r_bo        = 0.5;      % Risk weighting due to building occupancy
r_f         = 0.5;      % Risk weighting due to environmental fire

% Wind variables
v_w         = 1;        % Wind speed (m/s)
ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
c_w1        = 0.2;      % Wind constant 1 (for fire model)
c_w2        = 0.2;      % Wind constant 2 (for fire model)

% State maps
l_c_state   = 3;                        % dynamic state cell size (m)

%% Under construction
[c_f_env, l_c_f_x, l_c_f_y] = coarsenRatio(m_p_ref, l_c_state);
[m_bo, m_r] = coarsen(m_p_in, c_f_env); % Coarsen grid
n_x_f       = size(m_bo,1);             %
n_y_f       = size(m_bo,2);             %
n_f_i       = 1;                        % Number of initial fire outbreaks
m_f_i       = zeros(n_x_f, n_y_f);      % Initial fire map
m_bt        = zeros(n_x_f,n_y_f);       % Burntime map
m_bt_hist   = zeros(n_x_f, n_y_f);      % Burntime map history
m_f_hist    = zeros(n_x_f, n_y_f);      % Fire map history

% Search maps
c_f_search  = [5, 5]; % Search map coarsen factor from environment map
m_bo_search = coarsen(m_bo, c_f_search); 
n_x_search  = size(m_bo_search, 1);                 % Scan map size in x direction
n_y_search  = size(m_bo_search, 2);                 % Scan map size in y direction
l_c_s_x     = c_f_search(1)*l_c_f_x;                %
l_c_s_y     = c_f_search(2)*l_c_f_y;                %
t_scan_c    = t_scan_m*l_c_s_x*l_c_s_y;             % Scan time per cell
m_s         = zeros(n_x_search, n_y_search);        % Scan map
m_s_hist    = zeros(n_x_search, n_y_search);        % Scan map history
m_dw_hist   = zeros(n_x_search, n_y_search);        % Downwind map history
m_t_scan    = t_scan_c.*ones(n_x_search, n_y_search); % Scan time map (s) - time to scan each cell
t_scan_UAV  = zeros(n_UAV, 1);    % Time left to complete current scanning task
for UAV = 1:n_UAV
    t_scan_UAV(UAV) = m_bo_search(UAV_loc(UAV, 1), UAV_loc(UAV, 2));
end

% Axes may not be entirely accurate as coarsening may remove some
% rows/columns from original map.

% Axes for dynamic environment states
ax_lat_env = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_f);
ax_lon_env = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_f);

% Axes for search map
ax_lat_scan = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_search);
ax_lon_scan = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_search);

% Time estimation
% Number of desired data points
n_prog_data = 100;
% Avg travel time
t_trav_avg = t_scan_c + l_c_s_x/v_as_UAV;
% Estimated sim time
t_sim_est = t_trav_avg * n_x_search * n_y_search / n_UAV;
% Save data time
dt_v = t_sim_est / n_prog_data;
ct_v = 0;

% Priority map
negAtt            = NaN;  % Negative attraction for cells which don't need scanned
c_prior_building  = 1;    % Priority constant for building
c_prior_open      = 0.1;  % Priority constant for open space

% Calculate Priority map
m_prior = arrayfun(@(bo_search)(c_prior_building*bo_search + c_prior_open*(1-bo_search)), m_bo_search);

% Fire outbreak - n fire outbreaks in random buildings.
rng(1) % Seed to ensure consistency of initial firemap generation
for i = 1:numel(m_bo)
  if (m_bo(i) ~= 0)
    m_f_i(i) = 1;
  end
end

while n_f_i > 0
  coords = [randi([1 n_x_f]), randi([1 n_y_f])];
  if m_bo(coords(1), coords(2)) > 0
    m_f_i(coords(1), coords(2)) = 2;
    n_f_i = n_f_i - 1;
  end
end

% Alternative fire map
m_f_i(numel(m_bo)) = 1;

% Initialise fire maps
[m_f, m_bt, m_dw] = fireModel(...
      m_f_i, m_r, m_bo, m_bt, dt_f, k, n_x_f, n_y_f, ...
      v_w, ang_w, c_w1, c_w2, c_f_search);


% Generate FIS
[fisArray] = genFis_01( n_UAV );

% MPC variables
fis_params = [];
for UAV = 1:n_UAV
  fis_params = [fis_params, fisArray(UAV).Outputs.MembershipFunctions.Parameters];
end
% FIS parameters history
fis_param_hist = fis_params;
% Initial parameters for simulation
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
  fisArray, ...
  m_f, m_r, m_bo, m_bt, m_s, m_t_scan, ...
  dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
  n_UAV, n_p, n_x_search, n_y_search, n_x_f, n_y_f, l_queue, ...
  UAV_loc, UAV_target, UAV_task, t_travel_UAV, t_scan_UAV, ...
  k, negAtt, ...
  l_c_s_x, l_c_s_y, c_f_search, ...
  c_w1, c_w2, v_as_UAV, v_w, ang_w, ...
  r_bo, r_f);

% Optimisation options
t_max_opt = 60;
optTermCond = 'MaxTime';

fminsearchOptions = optimset('Display','iter','PlotFcns',@optimplotfval);
gaOptions         = optimoptions('ga','Display','iter', 'PlotFcn', @gaplotbestf);
patOptions        = optimoptions('patternsearch','Display','iter', 'PlotFcn',@psplotbestf, optTermCond, t_max_opt);
parOptions        = optimoptions('particleswarm','Display','iter', 'PlotFcn',@pswplotbestf);
% Termination condition

% Notes
% can try different mutation functions
% can set max number of generations
% Can set tolerances
% For patternsearch only or paretosearch with a single objective: 'psplotbestf' | 'psplotmeshsize' | 'psplotbestx' 

%% Test setup

% Objective function sensitivity test setup
if testSensitivity
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
  % Check these ranges work properly
  n_sens_1  = 1;
  n_sens_2  = 1;
  n_sens_3  = 1;
  n_sens_4  = 1;
  r_sens    = 1;
  p1        = p1_i*linspace(1-r_sens, 1+r_sens, n_sens_1);
  p2        = p2_i*linspace(1-r_sens, 1+r_sens, n_sens_2);
  p3        = p3_i*linspace(1-r_sens, 1+r_sens, n_sens_3);
  p4        = p4_i*linspace(1-r_sens, 1+r_sens, n_sens_4);
end

% Scaling test
if testScaling
  obj_fun_scaling = [];
  fis_var_scaling = [];
end

%% Error checking
if (v_w >= v_as_UAV)
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
  if runMPC
    t_start = tic;    
    if ct_mpc*dt_mpc <= t
      % For reproducibility
      rng default;
%       % Initialise figure
%       fprintf(strcat("MPC step ", num2str(ct_mpc)));
%       nameStr = strcat("optResults_MPC_step_", num2str(ct_mpc));
%       h = figure('name',nameStr);
      % Optimisation
      if optFunction == "fminsearch"
        [mpc_params, fval] = fminsearch(fun, ini_params, fminsearchOptions);
      elseif optFunction == "ga"
        [mpc_params,fval] = ga(fun, nvars, A, b, Aeq, beq, lb, ub, nonlcon, gaOptions);   
      elseif optFunction == "patternsearch"
        [mpc_params,fval] = patternsearch(fun, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, patOptions);   
      elseif optFunction == "particleswarm"
        [mpc_params,fval] = particleswarm(fun, nvars, lb, ub, parOptions);   
      end
      % Update FIS Parameters
      for UAV=1:n_UAV
        range       = 1 + (UAV - 1) * 4;
        fis_params  = mpc_params(range:range+3);
        fisArray(UAV).Outputs.MembershipFunctions.Parameters = fis_params;
      end
      % Update initial guess
      ini_params = mpc_params;
      % Record new parameters
      fis_param_hist = [fis_param_hist; mpc_params(1:n_UAV*4)];
      % Counter 
      ct_mpc = ct_mpc + 1;
    end
    t_MPC = toc(t_start);
  end

  %% MPC obj eval
  if testObj
    t_start = tic;
    if ct_mpc_eval*dt_mpc <= t
    % Counter 
    ct_mpc_eval = ct_mpc_eval + 1;
    % Generate initial parameters
    obj_eval = mpcModel(ini_params, ...
      fisArray, ...
      m_f, m_r, m_bo, m_bt, m_s, m_t_scan, ...
      dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
      n_UAV, n_p, n_x_search, n_y_search, n_x_f, n_y_f, l_queue, ...
      UAV_loc, UAV_target, UAV_task, t_travel_UAV, t_scan_UAV, ...
      k, negAtt, ...
      l_c_s_x, l_c_s_y, c_f_search, ...
      c_w1, c_w2, v_as_UAV, v_w, ang_w, ...
      r_bo, r_f);
    % Save objective
    obj_hist_eval(ct_mpc_eval) = obj_eval;
    end
    t_testObj = toc(t_start);
  end  
  %% Sensitivity test
  if testSensitivity
    t_start = tic;
    if ct_mpc_sens*dt_mpc <= t && ct_mpc_sens < ct_mpc_sens_fin
      % Counter 
      ct_mpc_sens = ct_mpc_sens + 1;
      % Eval obj function over prediction horizon for range of MF parameters
      for i = 1:n_sens_1
        for j = 1:n_sens_2
          for k = 1:n_sens_3
            for l = 1:n_sens_4
              % Generate parameter array
              fis_params = [];
              for UAV = 1:n_UAV
                fis_params = [fis_params, [p1(i), p2(j), p3(k), p4(l)]];
              end 
              sens_params = [];
              for p = 1:n_p
                sens_params = [sens_params, fis_params];
              end
              % Evaluate objective function
              obj = mpcModel( sens_params, ...
              fisArray, ...
              m_f, m_r, m_bo, m_bt, m_s, m_t_scan, ...
              dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
              n_UAV, n_p, n_x_search, n_y_search, n_x_f, n_y_f, l_queue, ...
              UAV_loc, UAV_target, UAV_task, t_travel_UAV, t_scan_UAV, ...
              k, negAtt, ...
              l_c_s_x, l_c_s_y, c_f_search, ...
              c_w1, c_w2, v_as_UAV, v_w, ang_w, ...
              r_bo, r_f);
              % Save objective
              obj_hist_sens(i,j,k,l,ct_mpc_sens) = obj;
            end
          end
        end
      end
    end
    t_testSensitivity = toc(t_start);
  end
  
  %% Path planning
  t_start = tic;
  if ct_c*dt_c <= t
    % Counter
    ct_c = ct_c + 1;
    % Path planner
    if testScaling
      [UAV_target, fis_var_scaling] = pathPlanner(...
        n_UAV, UAV_target, l_queue, ...
        n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
        m_s, m_t_scan, m_dw, m_prior, ...
        fisArray, ...
        t_travel_UAV, t_scan_UAV, ...
        ang_w, v_as_UAV, v_w, ...
        negAtt, fis_var_scaling);
    else
      UAV_target = pathPlanner(...
        n_UAV, UAV_target, l_queue, ...
        n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
        m_s, m_t_scan, m_dw, m_prior, ...
        fisArray, ...
        t_travel_UAV, t_scan_UAV, ...
        ang_w, v_as_UAV, v_w, ...
        negAtt);    
    end
  end
  t_pathPlanning = toc(t_start);

  %% Agent actions
  t_start = tic;
  if ct_a*dt_a <= t
    % Counter 
    ct_a = ct_a + 1;
    % Agent model
    [ m_s, UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
      t_travel_UAV, t_scan_UAV] ...
        = uavModel( n_UAV, ...
        m_t_scan, m_s, ...
        UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
        t_travel_UAV, t_scan_UAV, ...
        l_c_s_x, l_c_s_y, v_as_UAV, v_w, ang_w, dt_a, t);
  end
  t_agentActions = toc(t_start);
  
  %% Environment model
  t_start = tic;
  if ct_f*dt_f <= t
    % Counter 
    ct_f = ct_f + 1;
    % Environment map
    [m_f, m_bt, m_dw] = fireModel(...
      m_f, m_r, m_bo, m_bt, dt_f, k, n_x_f, n_y_f, ...
      v_w, ang_w, c_w1, c_w2, c_f_search);
  end 
  t_environment = toc(t_start);

  %% Store variables
  if ct_v*dt_v <= t
    ct_v = ct_v + 1;
    m_dw_hist(:,:,ct_v) = m_dw;
    m_bt_hist(:,:,ct_v) = m_bt;
    m_f_hist(:,:,ct_v)  = m_f;
    m_s_hist(:,:,ct_v) = m_s;
    s_obj_hist(ct_v)    = s_obj;
    obj_hist(ct_v)      = obj;
  end

  %% Objective function evaluation
  if testScaling
    [s_obj, obj]  = objEval(m_f, m_bo, m_s, r_bo, r_f, dt_s, s_obj, n_x_f, n_y_f, n_x_search, n_y_search, c_f_search, obj_fun_scaling);        
  else
    [s_obj, obj]  = objEval(m_f, m_bo, m_s, r_bo, r_f, dt_s, s_obj, n_x_f, n_y_f, n_x_search, n_y_search, c_f_search);    
  end
  
  %% Advance timestep
  t = t + dt_s;
  k = k + 1;

  %% Record module execution times
  if recordTime
    fprintf(strcat("Execution time - MPC Module - ", num2str(t_MPC), "\n"));
    fprintf(strcat("Execution time - testObj Module - ", num2str(t_testObj), "\n"));
    fprintf(strcat("Execution time - testSensitivity Module - ", num2str(t_testSensitivity), "\n"));    
    fprintf(strcat("Execution time - Path Planning Module - ", num2str(t_pathPlanning), "\n"));  
    fprintf(strcat("Execution time - Agent Actions Module - ", num2str(t_agentActions), "\n"));  
    fprintf(strcat("Execution time - Environment Module - ", num2str(t_environment), "\n"));
  end
    
  %% Progress report
  if ct_prog * dt_prog <= t
    progReport(endCondition, t, t_f, m_s, n_x_search, n_y_search);
    ct_prog = ct_prog + 1;
  end
  
  %% Check end condition
  [finishFlag] = simEndCondition(endCondition, t, t_f, m_s, n_x_search, n_y_search);
end

% Simulation time  
t_end = toc(t_sim);

%% Postprocessing

% Additional maps
m_prior_hist = (ones(size(m_s_hist)) - m_s_hist).*m_prior;

% Generate folder name
dateTime = datestr(now,'yyyy-mm-dd-HH-MM');
folder = strcat(dateTime, '-', exp_folder);
if sim_name ~= ""
  folder = strcat(folder, '-', sim_name);
end

if fig_exp
  % Generate and export figures 
  plotData  = {  
    'm_dw_hist',        m_dw_hist,        true;    
    'm_f_hist',         m_f_hist,         true;
    'm_scan_hist',      m_s_hist,         true;
    'UAV_loc_hist',     UAV_loc_hist,     true;
    's_obj_hist',       s_obj_hist,       true;
    'obj_hist',         obj_hist,         true;
    'm_bo',             m_bo,             true;
    'fis',              fisArray,         true;
    'm_prior',          m_prior,          true;
    'm_prior_hist',     m_prior_hist,     true};

  if testSensitivity
    plotData = [plotData; {'obj_hist_sens', obj_hist_sens, true}];  
  end
  
  if runMPC
    plotData = [plotData; {'fis_param_hist', fis_param_hist, true}]; 
  end

  plotResults( plotData, exp_saveDir, folder, ...
            ax_lon_env, ax_lat_env, ax_lon_scan, ax_lat_scan, ...
            dt_v, t, n_x_search, n_y_search, n_UAV, ct_v, fisArray);
end
        
% Export data
if data_exp
  % Save working directory path
  workingDir = pwd;
  % Change to save directory
  cd(saveDir); 
  % Save workspace  
  mkdir(folder);
  cd(folder);
  save(folder);
  % Go back to working directory
  cd(workingDir);
end
