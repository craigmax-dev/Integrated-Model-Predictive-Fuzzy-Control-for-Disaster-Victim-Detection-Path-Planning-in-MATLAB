 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

% CHANGELOG
% - Refactor: initialise plotting data can be removed
% - Bugfix: Improve strings used in figures
% - Simulations: Re-run with weights for dynamic environment variables
% - Feature: FIS input range - compress inputs to predefined ranges. Add warning
% message.
% - Feature: MPC prediction modes
% - Performance: Single prediction of environment states model before MPC
% optimization

% TODO   
% - Feature: Implement tuning of output MF parameters for MPFC
% - Feature: Implement MPC controller option
% - Correction: Rename MPC to MPFC
% - Performance: write data to file after each simulation. Then read and
% post-processing.
% - Restructure: clean up unused files
% - Feature: add battery recharge model and constraints to agent actions & fis
% - Feature/performance: add localised predictions for given radius around an agent
% - Feature: Implement ability to initiate fire spread during simulation.
% - Feature: Plotting of all simulations using light line and mean simulation
% using solid line 
% - Plotting: How to visualize agent behaviour over simulation. Think of other
% plotting options.
% - Feature: DelftBlue for simulations
% - Analysis: add analysis of computation times#
% - Feature: Implement prediction modes
% - Performance: better handling of environment history parameters
% - Tidying: .gitignore all input files except templates
% - Feature: Deterministic Threshold prediction mode
% - Feature: MPC arcitecture selection
% - Feature: Implement calculateAgentDistances in agent controller (if active - 3rd input)
% - Feature: Implement new controller architecture - MPC directly tuning agent
% locations
% Check: Fire propagation is functioning correctly
% Set fire weight parameters corectly for each simulation

%% SIMULATION CASES
% - 1. Victim locations modeled
% - 2. Dynamic environment variables
% - 3. Probability based predictions 
% - 4. Using m_bo distributions: 
% - Comparison stable behaviour vs MPC for loss of agent
% - TODO: longer prediction horizon, longer prediction timestep?

%% CONTROLLER ARCHITECTURES
% PRE-TUNED FIS (HAND)
% PRE-TUNED FIS (MPFC)
% MPFC
% MPC

% Clear workspace 
clear all  
close all
 
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
h_s_comms_disabled_victim_model = @()i_sim_comms_disabled_victim_model();
h_s_comms_enabled_victim_model = @()i_sim_comms_enabled_victim_model();
h_s_comms_disabled_victim_model_long = @()i_sim_comms_disabled_victim_model_long();

% Environment
h_e_static = @(dt_e)i_env_basic_no_dynamics(dt_e);
h_e_dynamic = @(dt_e)i_env_basic_dynamics(dt_e);

% Agent
h_a_repeat_1 = @(environment_model)i_a_repeat_1(environment_model);
h_a_repeat_2 = @(environment_model)i_a_repeat_2(environment_model);
h_a_repeat_2_battery_loss = @(environment_model)i_a_repeat_2_battery_loss(environment_model);


% FIS
h_init_fis_2 = @(n_a)initialise_fis_t_response_priority(n_a);
h_init_fis_3 = @(n_a)initialise_fis_t_response_priority_r_nextagent(n_a);


% MPC
h_mpc_disabled = @(fisArray, n_a)i_mpc_disabled(fisArray, n_a);
h_mpc_enabled = @(fisArray, n_a)i_mpc_enabled(fisArray, n_a);
h_mpc_enabled_500 = @(fisArray, n_a)i_mpc_enabled_500(fisArray, n_a);
h_mpc_enabled_1000 = @(fisArray, n_a)i_mpc_enabled_1000(fisArray, n_a);
h_mpc_enabled_longFirstEval = @(fisArray, n_a)i_mpc_enabled_longFirstEval(fisArray, n_a);
h_mpc_enabled_deterministic_prediction = @(fisArray, n_a)i_mpc_enabled_deterministic_prediction(fisArray, n_a);


% % comparison comms model
% simulationSetup = {
%   "Comms_Disabled", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Comms Disabled";
%   "Comms_Enabled", h_s_comms_enabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Comms Enabled";
%   };

% % comparison n_a
% simulationSetup = {
%   "sim_na_1", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_1, h_init_fis_2, h_mpc_disabled, "Single Agent";
%   "sim_na_2", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Two Agents";
%   }; 

% Loss of agent
simulationSetup = {
  "sim_loss_mpc_disabled", h_s_comms_disabled_victim_model_long, h_e_static, h_a_repeat_2_battery_loss, h_init_fis_2, h_mpc_disabled, "Pre-tuned FIS";
  "sim_loss_mpc_enabled", h_s_comms_disabled_victim_model_long, h_e_static, h_a_repeat_2_battery_loss, h_init_fis_2, h_mpc_enabled, "MPC Deterministic Exact";
  };

% % Dynamic environment test
% simulationSetup = {
%   "sim_dynamics", h_s_comms_enabled, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Dynamic Environment";
%   "sim_dynamics_mpc_longFirstEval", h_s_comms_enabled, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_longFirstEval, "Dynamic Environment with MPC";
%   };

% % Victim location modelling
% simulationSetup = { 
%   "sim_no_victim_model", h_s_comms_disabled, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No Victim Model";
%   "sim_victim_model", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Victim Model";  
%   };

% % FIS input parameter comparison
% simulationSetup = {
%   "sim_fis_2_inputs", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "sim_fis_2_inputs";
%   "sim_fis_3_inputs", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_3, h_mpc_disabled, "sim_fis_3_inputs";
%   };

% % MPC setup
% simulationSetup = {
%   "sim_no_mpc", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "PRE-TUNED FIS";
%   "sim_mpc_short", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled, "MPC 200 EVAL";
%   "sim_mpc_med", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_500, "MPC 500 EVAL";
%   "sim_mpc_long", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_1000, "MPC 1000 EVAL";
%   };

% % Prediction Modes Comparison for Static Environment
% simulationSetup = {
%   "sim_no_mpc", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No MPC";
%   "sim_mpc_enabled", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_enabled, "MPC Deterministic Exact";
%   "sim_mpc_enabled_deterministic_prediction", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_deterministic_prediction, "MPC Deterministic Prediction";
%   };

% % Prediction Modes Comparison for Dynamic Environment
% simulationSetup = {
%   "sim_no_mpc", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No MPC";
%   "sim_mpc_enabled", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled, "MPC Deterministic Exact";
%   "sim_mpc_enabled_deterministic_prediction", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_deterministic_prediction, "MPC Deterministic Prediction";
%   };

% Define the number of iterations for each simulation setup
numIterations = 2; 

% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);

% Initialize a structure to store results from all setups
allResults = struct();

% Iterate over each simulation setup
for simSetup = 1:size(simulationSetup, 1)
    
  % TODO REPLACE WITH ORIGINAL STRUCTURE
  simulationName = simulationSetup{simSetup, 1};
  f_init_sim = simulationSetup{simSetup, 2};
  f_init_env = simulationSetup{simSetup, 3};
  f_init_agent = simulationSetup{simSetup, 4};
  f_init_fis = simulationSetup{simSetup, 5};
  f_init_mpc = simulationSetup{simSetup, 6};

  % V2
  % Initialize a struct array to store results for this simulation setup
  % Note: Preallocate the struct array for efficiency
  results = struct('t_hist', cell(1, numIterations), 's_obj_hist', cell(1, numIterations), 'obj_hist', cell(1, numIterations));
  
  for iteration = 1:numIterations

      %% Initialise models, plotting data, and timestep for saving variables

      % Generate and record a unique seed for each iteration
      seeds(iteration) = randi(10000); % or another method to generate a random seed

      % Simulation parameters
      config = f_init_sim();
      
      % Initialise Environment 
      environment_model = f_init_env(config);
      
      % Initialise Agent
      agent_model = f_init_agent(environment_model);

      % Initialise FIS
      [fisArray] = f_init_fis(agent_model.n_a);

      % Initialise MPC
      mpc_model = f_init_mpc(fisArray, agent_model.n_a);
       
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

      %% Precompute dynamic environment states
      % The precompute is performed beyond the end of the simulation for the
      % case where the MPC prediction horizon extends beyond the simulation
      % time.

      % Set the seed for RNG once, unless reseeding is explicitly required for each iteration
      rng(seeds(iteration));
      
      % Calculate the total number of steps needed for the pre-computation phase
      total_steps = round((config.t_f + mpc_model.n_p * config.dt_mpc) / config.dt_e);
      
      for k_precompute = 0:(total_steps - 1)
          % Update the environment for the current step
          % Consider optimizing model_environment for batch updates if applicable
          environment_model = model_environment(environment_model, k_precompute, config.dt_e);
      end

      %% Simulation Loop
      while config.flag_finish == false

        % Start timer 
        t_sim = tic;
  
        %% MPC
        % TODO: update for environment_model, config, mpc, agent_model structures
        if mpc_model.flag_mpc 
          if config.k_mpc*config.dk_mpc <= config.k
            [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model); 
            config.k_mpc = config.k_mpc + 1;
          end 
        end  
 
        %% Path planning
        if config.k_c*config.dk_c <= config.k
          [agent_model] = model_fis(agent_model, environment_model.v_w, environment_model.ang_w, config, fisArray);
          config.k_c = config.k_c + 1;
        end     

        %% Agent model
        if config.k_a*config.dk_a <= config.k 
          agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
          config.k_a = config.k_a + 1; 
        end    
   
        %% Environment model
        if config.k_e*config.dk_e <= config.k
          config.k_e = config.k_e + 1; 
        end

        %% Objective function evaluation
        [config.s_obj, config.obj] = calc_obj(...
          config.weight, environment_model.m_f_series(:, :, config.k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
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
        if config.k_prog * config.dk_prog <= config.k
          report_progress(config.endCondition, config.t, config.t_f, agent_model.m_scan, agent_model.n_x_s, agent_model.n_y_s);
          config.k_prog = config.k_prog + 1;
        end

        %% Check end condition
        [config.flag_finish] = func_endCondition(config.endCondition, config.t, config.t_f, agent_model.m_scan, agent_model.n_x_s, agent_model.n_y_s);

      end

      % Correctly store results for each iteration
      results(iteration).t_hist = t_hist;        % Assuming t_hist is your time history for this iteration
      results(iteration).s_obj_hist = s_obj_hist; % Assuming s_obj_hist is your objective history for this iteration
      results(iteration).obj_hist = obj_hist;    % Assuming obj_hist is additional objective metrics for this iteration
      
  end

    % Store results from this setup for later comparison
    allResults.(simulationName) = results;

end

%% Statistical analysis

% Confidence interval level
alpha = 0.05;

% Calculate stats for obj_hist
[means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj] = calculateStats(allResults, simulationSetup, 'obj_hist', config.dt_s, alpha);

row_sums = cellfun(@sum, means_obj);

% fprintf(row_sums)

%% Plotting

% Plot stats for obj_hist
plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetup, 'Mean Objective Value', 'Objective Value');

% NOTE: Concept to plot events
% battery_threshold_events = findBatteryLevelEvents(agent_model.a_battery_level_i, threshold);

% plotStats(means, ci_lower, ci_upper, time_vector, simulationSetup, titleStr, yLabel, battery_threshold_events);

%% Plot Geographical

% Parameters List
agent_parameter_list = {'m_f', 'm_bo'};
search_parameter_list = {'m_bo_s', 'm_victim_s'};

% Items and their locations
items = {'UAV'}; 
item_locations = {agent_model.a_loc};
markerSizes = [10]; % List of marker sizes for items

% plotGeographical(agent_model, environment_model, agent_parameter_list, search_parameter_list, items, item_locations, markerSizes);

%% Save and export simulation results

% Call the function to save results and figures
saveSimulationResults(config.flag_save, allResults, config);
