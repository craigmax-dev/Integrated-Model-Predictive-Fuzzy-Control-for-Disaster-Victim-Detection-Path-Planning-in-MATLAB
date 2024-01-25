 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

% CHANGELOG
% - add flag_scan_task to initialization
% - flag_scan_task needs to be increased at each time step
% - time since last scanned
% - function: generateVictimsMap
% - function: generateBrownNoiseMatrix
% - function: generateMatrixFromPDFs
% - function: compareFunctionOutputs
% - function: runSimulationsAndComputeCI
% - m_victim_s initialisation changed from environment to agent
% - m_victim_s added to function inputs: MPC, obj
% - replaced m_t_dw with m_dw
% calc_obj - efficiency improvements
% - feature: fixed implementation of environment and agent maps. m_dw_e and m_dw
% now handled correctly
% - performance: removed environment history parameters for animation
% - update handling of maps: environment defined in environment, agent defined
% in agent. All parameters use respective size maps
% - feature: functions for statistical analysis and plotting confidence
% intervals
% - removed config.test_fis_sensitivity, etc flags from input file
% - removed fis_param_hist
% - Restructure: move files in main folder to functions folder (using githmpc_model.ub)
 
% TODO 
% - Feature: probability-based predictions in MPC
% - Feature/performance: add localised predictions for given radius around an agent
% - Feature: add battery model and loss of agents
% - Feature: Mirko model implementation: FIS, MPC, MPC steps, 
% - Refactor: initialise plotting data can be removed
% - Feature: Write a function to convert raster data to a matrix, and save any other relevant variables as a separate variable. This will allow us to convert raster data first and then use the matrix as an input to the  calc_coarsenRatio function.
% - Performance: write data to file after each simulation
% - Feature: remove concept of task queue for agents
% - Performance: Single prediction of environment states model before MPC optimization
% - Restructure: clean up unused files
% - Bugfix: implement proper use of m_prior - should this be an agent or mpc parameter?
% - Restructure: Move to config file: mpc_model.flag_mpc, simulation config
% - Rename: config under agent_model (to avoid confusion)
% - Data points to record: battery level, ...
% - Agents seem to take too long to scan cells currently - on scale of days to
% come to stable solution with 8x8 search environment
% Troubleshooting: 
% % check correct/logical cells scanned by agents (validated by reaching stable state)
% % check timestep functionality - what happens when agent completes task early
% etc?
% Remove constraint from multiple agents entering same cell
% Fix bug: agents stop scanning
% Data mapping: clear separation of agent and mpc data. E.g. agents should not
% have knowledge of data states of other agents. e.g. m_prior
% Add recalculation of m_prior
% What other variables can be implemented in model? Variable scan times? 
% Test version with agents communicating cell locations.
% NOTE: normalization using maxTravelTime >1 in some cases
% Fix agents not acting at beginning of simulation: could be tuning issue.
% Fix issue with agent actions calculation.
% Track average max priority - can use this to normalize
% Add agent check: maximum control timestep etc
% Comms-coordination modelling: double-check during assignment (for same spot in
% queue), etc...

% RESEARCH
% Resarch normalization functions for FIS inputs
% Solutions to synchronised agents:
% - constraint on proximity to nearest agent
% - communicate planned cells to scan
% - 

% Plots
% Histogram of cells scanned over time
% 


% Clear workspace 
clear all  
% close all
 
% Set up folder paths
addpath('data', ...
  'matlab', ...
  'matlab/functions', ...
  'matlab/models', ...
  'matlab/initialisation/simulation', ...
  'matlab/initialisation/plotting', ...
  'matlab/initialisation/pathPlanning', ...
  'matlab/initialisation/mpc', ...
  'matlab/initialisation/environment', ...
  'matlab/initialisation/agent');
  
%% Define function handles 
% Function handles are used to refer to the diffe rent scripts used in the
% simulations

% Simulation variables
h_s_comms_disabled = @()i_sim_comms_disabled();
h_s_comms_enabled = @()i_sim_comms_enabled();

% Environment
h_e_static = @()initialise_environment_SIM_basic_no_dynamic();
h_e_dynamic = @()initialise_environment_SIM_basic_dynamic();

% Agent
h_a_repeat_1 = @(environment_model)i_a_repeat_1(environment_model);
h_a_repeat_2 = @(environment_model)i_a_repeat_2(environment_model);
h_a_repeat_2_battery_loss = @(environment_model)i_a_repeat_2_battery_loss(environment_model);



% FIS
h_init_fis_1 = @(n_a)initialise_fis_SIM_1(n_a);

% MPC
h_mpc_disabled = @(fisArray, n_a)i_mpc_disabled(fisArray, n_a);
h_mpc_enabled = @(fisArray, n_a)i_mpc_enabled(fisArray, n_a);

% Handles for solver test
h_init_MPC_SOLV_fminsearch = @(fisArray, n_a)initialise_MPC_ST01_fminsearch(fisArray, n_a);
h_init_MPC_SOLV_ga = @(fisArray, n_a)initialise_MPC_ST01_ga(fisArray, n_a);
h_init_MPC_SOLV_particleswarm = @(fisArray, n_a)initialise_MPC_ST01_particleswarm(fisArray, n_a);
h_init_MPC_SOLV_patternsearch = @(fisArray, n_a)initialise_MPC_ST01_patternsearch(fisArray, n_a);

% simulationSetups = {
%   "Comms_Disabled", h_s_comms_disabled, h_e_static, h_a_repeat_2, h_init_fis_1, h_mpc_disabled;
%   "Comms_Enabled", h_s_comms_enabled, h_e_static, h_a_repeat_2, h_init_fis_1, h_mpc_disabled;
%   };

% comparison n_a
% simulationSetups = {
%   "sim_na_1", h_s_comms_disabled, h_e_static, h_a_repeat_1, h_init_fis_1, h_mpc_disabled;
%   "sim_na_2", h_s_comms_enabled, h_e_static, h_a_repeat_2, h_init_fis_1, h_mpc_disabled;
%   };

% % comparison n_a
% simulationSetups = {
%   "sim_no_loss", h_s_comms_enabled, h_e_static, h_a_repeat_2, h_init_fis_1, h_mpc_disabled;
%   "sim_loss", h_s_comms_enabled, h_e_static, h_a_repeat_2_battery_loss, h_init_fis_1, h_mpc_disabled;
%   };

% MPC basic
simulationSetups = {
  "sim_no_mpc", h_s_comms_enabled, h_e_static, h_a_repeat_2, h_init_fis_1, h_mpc_disabled;
  "sim_mpc", h_s_comms_enabled, h_e_static, h_a_repeat_2, h_init_fis_1, h_mpc_enabled;
  };


% Define the number of iterations for each simulation setup
numIterations = 1; 

% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);

% Initialize a structure to store results from all setups
allResults = struct();

% Iterate over each simulation setup
for simSetup = 1:size(simulationSetups, 1)
    
  % TODO REPLACE WITH ORIGINAL STRUCTURE
  simulationName = simulationSetups{simSetup, 1};
  f_init_sim = simulationSetups{simSetup, 2};
  f_init_env = simulationSetups{simSetup, 3};
  f_init_agent = simulationSetups{simSetup, 4};
  f_init_fis = simulationSetups{simSetup, 5};
  f_init_mpc = simulationSetups{simSetup, 6};

  % Initialize an array to store results for this simulation setup
  results = struct('t_hist', [], 's_obj_hist', [], 'obj_hist', []);

  for iteration = 1:numIterations

      %% Initialise models, plotting data, and timestep for saving variables

      % Generate and record a unique seed for each iteration
      seeds(iteration) = randi(10000); % or another method to generate a random seed
      rng(seeds(iteration)); % Set the seed for RNG

      % Simulation parameters
      config = f_init_sim();
      
      % Initialise Environment 
      environment_model = f_init_env();
      
      % Initialise Agent
      agent_model = f_init_agent(environment_model);

      % Initialise FIS
      [fisArray] = f_init_fis(agent_model.n_a);

      % Initialise MPC
      mpc_model = f_init_mpc(fisArray, agent_model.n_a);
       
      % Initialise Plots
      [axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
      m_f_hist, m_f_hist_animate, m_bt_hist_animate, m_dw_hist_animate, ...
      t_hist, obj_hist, s_obj_hist] ... 
      = initialise_plotting(environment_model.n_x_e, environment_model.n_y_e, agent_model.n_x_s, agent_model.n_y_s, environment_model.m_f, environment_model.m_bt, mpc_model.fis_params);
      
      %% Define timestep for saving variables
      % Number of desired data points
      n_prog_data = 500;
      % Estimated avg travel time
      k_trav_avg = (agent_model.t_scan_c + agent_model.l_x_s/agent_model.v_as)/config.dt_s;
      % Estimated sim time
      k_sim_est = k_trav_avg * agent_model.n_x_s * agent_model.n_y_s / agent_model.n_a;
      % Save data time
      dk_v = k_sim_est / n_prog_data;
      ct_v = 0; 
  
      %% Simulation Loop
      while config.flag_finish == false

        % Start timer 
        t_sim = tic;
  
        %% MPC
        % TODO: update for environment_model, config, mpc, agent_model structures
        if mpc_model.flag_mpc 
          if config.k_mpc*config.dk_mpc <= config.k
            [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model, seeds(iteration)); 
            config.k_mpc = config.k_mpc + 1;
          end 
        end  
 
        %% Path planning
        if config.k_c*config.dk_c <= config.k
          [agent_model] = model_fis(agent_model, environment_model, config, fisArray);
          config.k_c = config.k_c + 1;
        end     

        %% Agent model
        if config.k_a*config.dk_a <= config.k 
          agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
          config.k_a = config.k_a + 1; 
        end    
   
        %% Environment model
        if config.k_e*config.dk_e <= config.k
          environment_model = model_environment(environment_model);          
          config.k_e = config.k_e + 1; 
        end 

        %% Objective function evaluation
        [config.s_obj, config.obj] = calc_obj(...
          config.weight, environment_model.m_f, agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
          config.dt_s, config.s_obj, agent_model.c_f_s, config.t);
        
        %% Store variables 
        if ct_v*dk_v <= config.k
          ct_v = ct_v + 1; 
          t_hist(ct_v) = ct_v*dk_v*config.dt_s;
          s_obj_hist(ct_v)    = config.s_obj;
          obj_hist(ct_v)      = config.obj; 
        end 

        %% Advance timestep
        config.t = config.t + config.dt_s;
        config.k = config.k + 1;

        %% Progress report
        if config.k_prog * config.dk_prog <= config.t
          report_progress(config.endCondition, config.t, config.t_f, agent_model.m_scan, agent_model.n_x_s, agent_model.n_y_s);
          config.k_prog = config.k_prog + 1;
        end

        %% Check end condition
        [config.flag_finish] = func_endCondition(config.endCondition, config.t, config.t_f, agent_model.m_scan, agent_model.n_x_s, agent_model.n_y_s);

      end

      % Store results for each iteration
      results(iteration).t_hist = t_hist;
      results(iteration).s_obj_hist = s_obj_hist;
      results(iteration).obj_hist = obj_hist;

  end

    % Store results from this setup for later comparison
    allResults.(simulationName) = results;

end



%% Statistical analysis

% Confidence interval level
alpha = 0.05;

% Calculate stats for s_obj_hist
[means_s_obj, ci_lower_s_obj, ci_upper_s_obj, time_vector_s_obj] = calculateStats(allResults, simulationSetups, 's_obj_hist', config.dt_s, alpha);

% Calculate stats for obj_hist
[means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj] = calculateStats(allResults, simulationSetups, 'obj_hist', config.dt_s, alpha);


%% Plotting

% Let's say we want to plot vertical lines when battery level falls below a threshold
% Assuming you have a function or a way to find the time when battery level falls below a threshold
% battery_threshold_events = findBatteryLevelEvents(agent_model.a_battery_level_i, threshold);


% Plot stats for s_obj_hist
% plotStats(means_s_obj, ci_lower_s_obj, ci_upper_s_obj, time_vector_s_obj, simulationSetups, 'Sum of Objective History Across Simulation Setups', 'Sum of Objective Value');

% Plot stats for obj_hist
plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetups, 'Objective Values and Confidence Intervals Across Simulation Setups', 'Objective Value');
% plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetups, 'Objective Values and Confidence Intervals Across Simulation Setups', 'Objective Value', agent_model.a_battery_level_i);

% plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, battery_threshold_events);




