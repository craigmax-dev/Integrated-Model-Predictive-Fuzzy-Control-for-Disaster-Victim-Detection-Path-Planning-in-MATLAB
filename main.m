 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology
 
% CHANGELOG
% - Refactor: initialise plotting data can be removed
% - Bugfix: Improve strings used in figures
% - Simulations: Re-run with weights for dynamic environment variables

% CHANGELOG
% - Feature: FIS input range - compress inputs to predefined ranges. Add warning
% message.
% - Feature: probability-based predictions in MPC
% - 1: separate fully-defined fire spread probability likelihood
% - 2: base probability on simple parameter e.g. downwind map / probability
% calculation without propagating fire spread.
% - 3: Monte-carlo simulation (not feasible embedded within an MPC optimization)

% IN PROGRESS
% - Feature: MPC arcitecture selection
% - Feature: Implement calculateAgentDistances in agent controller (if active - 3rd input)

% TODO   
% - Performance: Single prediction of environment states model before MPC
% optimization
% - Performance: write data to file after each simulation. Then read and
% post-processing.
% - Restructure: clean up unused files
% - Feature: add battery model and loss of agents
% - Feature/performance: add localised predictions for given radius around an agent
% - Feature: Implement ability to initiate fire spread during simulation.
% - Feature: Plotting of all simulations using light line and mean simulation
% using solid line 
% - Plotting: How to visualize agent behaviour over simulation. Think of other
% plotting options.
% - Feature: DelftBlue for simulations

% Questions: 
% - how to choose suitable range / MF parameters for FIS inputs?
% - how to setup optimization? go with current setup or attempt different?

%% SIMULATION CASES



%% NEXT SIMULATIONS
% - 1. Victim locations modeled
% - 2. Dynamic environment variables
% - 3. Probability based predictions 
% - 4. Using m_bo distributions: 

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

% % Loss of agent
% simulationSetup = {
%   "sim_no_loss", h_s_comms_enabled, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No loss of agent";
%   "sim_loss", h_s_comms_enabled, h_e_static, h_a_repeat_2_battery_loss, h_init_fis_2, h_mpc_disabled, "Loss of agent";
%   };
 
% % Dynamic environment test
% simulationSetup = {
%   "sim_dynamics", h_s_comms_enabled, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Dynamic Environment";
%   "sim_dynamics_mpc_longFirstEval", h_s_comms_enabled, h_e_dynamic, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_longFirstEval, "Dynamic Environment with MPC";
%   };

% Victim location modelling
simulationSetup = { 
  "sim_no_victim_model", h_s_comms_disabled, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No Victim Model";
  "sim_victim_model", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "Victim Model";  
  };

% % MPC basic 
% simulationSetup = {
%   "sim_no_mpc", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No MPC";
%   "sim_mpc", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_enabled, "MPC";
%   "sim_mpc_longFirstEval", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_longFirstEval;
%   };

% % FIS Config comparison
% simulationSetup = {
%   "sim_no_mpc", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_disabled, "No MPC";
%   "sim_mpc", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_enabled, "MPC";
%   "sim_mpc_longFirstEval", h_s_comms_disabled_victim_model, h_e_static, h_a_repeat_2, h_init_fis_2, h_mpc_enabled_longFirstEval;
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

% Define the number of iterations for each simulation setup
numIterations = 4 ; 

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
      environment_model = f_init_env(config.dt_e);
      
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
          environment_model = model_environment(environment_model, config.dt_e);          
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
        if config.k_prog * config.dk_prog <= config.k
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
[means_s_obj, ci_lower_s_obj, ci_upper_s_obj, time_vector_s_obj] = calculateStats(allResults, simulationSetup, 's_obj_hist', config.dt_s, alpha);

% Calculate stats for obj_hist
[means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj] = calculateStats(allResults, simulationSetup, 'obj_hist', config.dt_s, alpha);


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
