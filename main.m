 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology 

% FEATURE BACKLOG
% - Feature: Add battery recharge model to FIS
% - Feature: MPFC/MPC prediction with clustering
% - Feature: Implement slow dynamic environment variable
% - Feature: Deterministic threshold prediction mode

% TASKS
% - Simulations
% - Code base cleanup and documentation
% - Graduation roadmap
% - Methodology and results writeup - diagrams, etc etc
% - Cleanup of simulation files
% - Pre-GL Presentation
% - Publish Github * merge to main branch

% Clear workspace 
clear all  
close all
 
%% Set up folder paths
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

% Simulation variables
h_s_comms_disabled_no_victim_model = @()i_sim_comms_disabled();
h_s_comms_disabled_victim_model = @()i_sim_comms_disabled_victim_model();
h_s_comms_disabled_victim_model_5000 = @()i_sim_comms_disabled_victim_model_5000();
h_s_comms_disabled_victim_model_5000_local_map_r3 = @()i_sim_comms_disabled_victim_model_5000_local_map_r3();
h_s_comms_disabled_victim_model_5000_local_map_r5 = @()i_sim_comms_disabled_victim_model_5000_local_map_r5();
h_s_comms_disabled_victim_model_10000_local_map_r5 = @()i_sim_comms_disabled_victim_model_10000_local_map_r5();


% Environment
h_e_static = @(dt_e)i_env_basic_no_dynamics(dt_e);
h_e_dynamic = @(dt_e)i_env_basic_dynamics(dt_e);
h_e_dynamic_large = @(dt_e)i_env_basic_dynamics_large(dt_e);
h_e_complex_dynamics_large = @(dt_e)i_env_basic_dynamics_large_complex(dt_e);

% Agent
h_a_repeat_2 = @(environment_model, config)i_a_repeat_2(environment_model, config);
h_a_repeat_2_low_battery = @(environment_model, config)i_a_repeat_2_low_battery(environment_model, config);
h_a_repeat_2_mpc = @(environment_model, config)i_a_repeat_2_mpc(environment_model, config);
h_a_repeat_3 = @(environment_model, config)i_a_repeat_3(environment_model, config);
h_a_repeat_3_mpc = @(environment_model, config)i_a_repeat_3_mpc(environment_model, config);
h_a_repeat_4 = @(environment_model, config)i_a_repeat_4(environment_model, config);
h_a_repeat_4_mpc = @(environment_model, config)i_a_repeat_4_mpc(environment_model, config);
h_a_repeat_4_complex = @(environment_model, config)i_a_repeat_4_complex(environment_model, config);

% FIS
h_init_fis_2 = @(n_a)initialise_fis_t_response_priority(n_a);
h_init_fis_proximity = @(n_a)initialise_fis_t_response_priority_r_nextagent(n_a);
h_init_fis_mirko_4 = @(n_a)initialise_fis_mirko_4(n_a);
h_init_fis_mirko_4_recharge = @(n_a)initialise_fis_mirko_4_recharge(n_a);
h_init_fis_mod = @(n_a)initialise_fis_mod(n_a);  
   
% Controller Architecture
h_arch_fis = @(fisArray, agent_model)i_arch_fis(fisArray, agent_model);
h_arch_mpfc_input_exact = @(fisArray, agent_model)i_arch_mpfc_input_exact(fisArray, agent_model);
h_arch_mpfc_output_exact = @(fisArray, agent_model)i_arch_mpfc_output_exact(fisArray, agent_model);
h_arch_mpfc_output_exact_decentralised = @(fisArray, agent_model)i_arch_mpfc_output_exact_decentralised(fisArray, agent_model);
h_arch_mpc_exact = @(fisArray, agent_model)i_arch_mpc_exact(fisArray, agent_model);
h_arch_mpfc_input_prediction = @(fisArray, agent_model)i_arch_mpfc_input_prediction(fisArray, agent_model);
h_arch_mpfc_output_prediction = @(fisArray, agent_model)i_arch_mpfc_output_prediction(fisArray, agent_model);
h_arch_mpfc_output_prediction_decentralised = @(fisArray, agent_model)i_arch_mpfc_output_prediction_decentralised(fisArray, agent_model);
h_arch_mpc_prediction = @(fisArray, agent_model)i_arch_mpc_prediction(fisArray, agent_model);
h_arch_mpc_prediction_decentralised = @(fisArray, agent_model)i_arch_mpc_prediction_decentralised(fisArray, agent_model);

%% Simulation setup 

% SIMULATION BACKLOG
% 1. Demo Local prediciton
% 2. Local prediction large environment
% 3. Prediction horizon comparison

% -------------------------------------------------------------------------

% SIMULATION 1: Two agents static - DONE - V4
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_e_static, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS";
  % "sim_mpfc_centralised", h_s_comms_disabled_victim_model_5000, h_e_static, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised";
  % "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_e_static, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Decentralised";
  % "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_static, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "MPC";
% };  

% -------------------------------------------------------------------------

% SIMULATION 2: Two agents dynamic - DONE - V4
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Decentralised";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_exact, "MPC";
% };  

% -------------------------------------------------------------------------

% SIMULATION 3: Two agents dynamic - cell_fire_proximity vs m_dw FIS input - DONE - V4
% simulationSetup = { 
%   "sim_mpfc_centralised_mod", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_mod, h_arch_mpfc_output_prediction, "MPFC Centralised Downwind Input";
%   "sim_fis_mirko", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised Mirko Inputs";
%   "sim_mpc", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "MPC";
% };  

% -------------------------------------------------------------------------

% SIMULATION 4: % Three agents - DONE - V4
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_3, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Decentralised";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_3_mpc, h_init_fis_mirko_4, h_arch_mpc_exact, "MPC";
% };  

% -------------------------------------------------------------------------

% SIMULATION 5: % Four agents - DONE - V4
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Decentralised";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_exact, "MPC";
% };  

% -------------------------------------------------------------------------

% SIMULATION 6: ENVIRONMENT SIZE - DONE - V4
% % NOTE: agents depleting target list
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic_large, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_e_dynamic_large, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised Mirko Inputs";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_dynamic_large, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "MPC";
% };  

% -------------------------------------------------------------------------

% SIMULATION 7: PREDICTION MODES COMPARISON
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mfpc_prediction", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC PREDICTION";
%   "sim_mfpc_exact", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_exact, "MPFC EXACT";
% }; 

% -------------------------------------------------------------------------

% SIMULATION X: PROXIMITY FIS INPUT (SHOW PERCENT IMPROVEMENT) - DONE - V4

% -------------------------------------------------------------------------

% SIMULATION 8: Two agents, double population centre
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2, h_init_fis_2, h_arch_fis, "FIS";
%   "sim_mfpc_2", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2, h_init_fis_2, h_arch_mpfc_output_prediction, "MPFC 2 inputs";
  % "sim_mfpc_3", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2, h_init_fis_proximity, h_arch_mpfc_output_prediction, "MPFC 3 inputs";
%   "sim_mpc", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2_mpc, h_init_fis_2, h_arch_mpc_prediction, "MPC";
  % }; 

% -------------------------------------------------------------------------

% SIMULATION 9: Decentralised vs centralised structures for MPFC - DONE - V4
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Decentralised";
%   "sim_mpfc_centralised", h_s_comms_disabled_victim_model, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised";
%   }; 

% -------------------------------------------------------------------------

% SIMULATION 10: Compare FIS input parameters - DONE - V4
% simulationSetup = {
  % "sim_fis", h_s_comms_disabled_victim_model, h_e_dynamic_large, h_a_repeat_2, h_init_fis_2, h_arch_fis, "FIS";
  % "sim_mpfc_basic", h_s_comms_disabled_victim_model, h_e_dynamic_large, h_a_repeat_2, h_init_fis_2, h_arch_mpfc_output_prediction, "MPFC Basic";
  % "sim_mpfc_proximity", h_s_comms_disabled_victim_model, h_e_dynamic_large, h_a_repeat_2, h_init_fis_proximity, h_arch_mpfc_output_prediction, "MPFC Proximity Input";
  % "sim_mpfc_mirko", h_s_comms_disabled_victim_model, h_e_dynamic_large, h_a_repeat_2, h_init_fis_mirko, h_arch_mpfc_output_prediction, "MPFC Proximity Input";
  % "sim_mpc", h_s_comms_disabled_victim_model, h_e_dynamic_large, h_a_repeat_2_mpc, h_init_fis_2, h_arch_mpc_prediction, "MPC";
  % }; 

% -------------------------------------------------------------------------  
 
% SIMULATION 11: LARGE ENVIRONMENT 4 AGENTS - DONE - V4
% TODO: Plot results in 2d surface behaviour characterisation
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model_5000, h_e_dynamic_large, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "FIS";
%   "sim_mfpc", h_s_comms_disabled_victim_model_5000, h_e_dynamic_large, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_dynamic_large, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "MPC";
% }; 

% -------------------------------------------------------------------------

% SIMULATION X: Battery recharge
% simulationSetup = { 
%   "sim_fis_battery_recharge", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2_low_battery, h_init_fis_mirko_4_recharge, h_arch_fis, "FIS Battery Recharge";
%   "sim_fis_no_battery_recharge", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS No Battery Recharge";
%   % "sim_mpfc_battery_recharge", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2_low_battery, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Battery Recharge";
%   % "sim_mpfc_no_battery_recharge", h_s_comms_disabled_victim_model_5000, h_e_dynamic, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC No Battery Recharge";
% };  

% -------------------------------------------------------------------------

% SIMULATION 12: Simple local prediction maps - DONE - V4
% simulationSetup = {  
%   "sim_fis_global", h_s_comms_disabled_victim_model_5000, h_e_static, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS Global Maps";
%   "sim_fis_local_r_5", h_s_comms_disabled_victim_model_5000_local_map_r5, h_e_static, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS Local Maps R5";
%   "sim_mpfc_global", h_s_comms_disabled_victim_model_5000, h_e_static, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Global Maps";
%   "sim_mpfc_local_r_5", h_s_comms_disabled_victim_model_5000_local_map_r5, h_e_static, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Local Maps R5";
% };     

% -------------------------------------------------------------------------

% SIMULATION 13: COMPLEX LARGE DISASTER ENVIRONMENT, TWO POPULATION CENTRES - DONE - V4
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model_5000, h_e_complex_dynamics_large, h_a_repeat_4, h_init_fis_2, h_arch_fis, "FIS";
%   "sim_mfpc", h_s_comms_disabled_victim_model_5000, h_e_complex_dynamics_large, h_a_repeat_4, h_init_fis_2, h_arch_mpfc_output_prediction, "MPFC";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_e_complex_dynamics_large, h_a_repeat_4_mpc, h_init_fis_2, h_arch_mpc_prediction, "MPC";
% }; 

% -------------------------------------------------------------------------

% SIMULATION 14: COMPLEX LARGE DISASTER ENVIRONMENT, TWO POPULATION CENTRES, LOCAL
% PREDICTION MAPS - DONE - V4
simulationSetup = {  
  "sim_fis_global", h_s_comms_disabled_victim_model_5000, h_e_complex_dynamics_large, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS Global Maps";
  "sim_mpfc_local_r_3", h_s_comms_disabled_victim_model_5000_local_map_r3 , h_e_complex_dynamics_large, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Local Maps R3";
  "sim_mpfc_local_r_5", h_s_comms_disabled_victim_model_5000_local_map_r5 , h_e_complex_dynamics_large, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Local Maps R5";
};    

% -------------------------------------------------------------------------

% SIMULATION X: STABLE BEHAVIOUR CONVERGENCE DEMONSTRATION

% -------------------------------------------------------------------------

% SIMULATION X: ADAPTIVE BEHAVIOUR DEMONSTRATION - LOSS OF AGENT

% -------------------------------------------------------------------------

%% Define the number of iterations for each simulation setup
numIterations = 3;   
 
% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);

% Initialize a structure to store results from all setups 
data = struct();
 
%% Iterate over each simulation setup
for simSetup = 1:size(simulationSetup, 1)
    
  simulationName = simulationSetup{simSetup, 1};
  f_init_sim = simulationSetup{simSetup, 2};
  f_init_env = simulationSetup{simSetup, 3};  
  f_init_agent = simulationSetup{simSetup, 4};
  f_init_fis = simulationSetup{simSetup, 5};
  f_init_mpc = simulationSetup{simSetup, 6}; 

  fprintf("Simulation: %s \n", simulationName)

  % V2
  % Initialize a struct array to store results for this simulation setup
  % Note: Preallocate the struct array for efficiency
  results = struct('t_hist', cell(1, numIterations), 's_obj_hist', cell(1, numIterations), 'obj_hist', cell(1, numIterations));
   
  for iteration = 1:numIterations

      fprintf("Iteration: %i \n", iteration)
    
      %% Initialise models, plotting data, and timestep for saving variables
 
      % Generate and record a unique seed for each iteration
      seeds(iteration) = randi(10000); % or another method to generate a random seed

      % Simulation parameters
      config = f_init_sim();
      
      % Initialise Environment 
      environment_model = f_init_env(config); 

      % Initialise Agent
      agent_model = f_init_agent(environment_model, config);

      % Initialise FIS Path Planning
      [fisArray] = f_init_fis(agent_model.n_a);
 
      % Initialise Supervisory Control 
      mpc_model = f_init_mpc(fisArray, agent_model);
       
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
      prediction_duration = mpc_model.n_p * config.dk_pred * config.dt_s;
      total_time = config.t_f + prediction_duration;
      total_steps = ceil(total_time / config.dt_e) + 1;

      for k_precompute = 0:total_steps
          % Update the environment for the current step
          environment_model = model_environment(environment_model, k_precompute, config.dt_e);

      end 

      %% Simulation Loop
      while config.flag_finish == false
 
        % Start timer 
        t_sim = tic;
  
        %% Supervisory Control Model
        if (strcmp(mpc_model.architecture, 'mpc') || strcmp(mpc_model.architecture, 'mpfc')) && (config.k_mpc*config.dk_mpc <= config.k)
          [fisArray, mpc_model, agent_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model);
          config.k_mpc = config.k_mpc + 1;
        end  
 
        %% FIS Path Planning Model 
        if (strcmp(mpc_model.architecture, 'fis') || strcmp(mpc_model.architecture, 'mpfc')) && (config.k_c * config.dk_c <= config.k)
          if isfield(config, 'flag_local_maps') && config.flag_local_maps
            [agent_model] = model_fis_local(agent_model, environment_model.ang_w, environment_model.v_w, config, fisArray);
          else
            [agent_model] = model_fis_global(agent_model, environment_model.ang_w, environment_model.v_w, config, fisArray);
          end
            config.k_c = config.k_c + 1;
        end

        %% Agent Model 
        if config.k_a*config.dk_a <= config.k 
          agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, environment_model.m_f_series(:, :, config.k_e + 1), environment_model.m_dw_e_series(:, :, config.k_e + 1), config);
          config.k_a = config.k_a + 1; 
        end     
   
        %% Environment Model
        if config.k_e*config.dk_e <= config.k
          config.k_e = config.k_e + 1; 
        end

        %% Objective function evaluation

        % V3 REFACTOR
        [config.s_obj, config.obj] = calc_obj_v4(...
          config.weight, environment_model.m_dw_e_series(:, :, config.k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
          config.dt_s, config.s_obj, config.c_f_s);

        % % MIRKO OBJECTIVE
        % [config.s_obj, config.obj] = calc_obj_mirko(agent_model, fisArray, environment_model.v_w, environment_model.ang_w, config);               

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
      results(iteration).optimizationTimes = mpc_model.optimizationTimes; 

  end

    % Store results from this setup for later comparison
    data.(simulationName) = results;

end 

%% Statistical analysis

% Confidence interval level
alpha = 0.05;

% Calculate stats for obj_hist
[means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj] = calculateStats(data, simulationSetup, 'obj_hist', config.dt_s, alpha);

% % Calculate stats for optimizationTimes
[means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt] = calculateStats(data, simulationSetup, 'optimizationTimes', config.dt_s, alpha);

row_sums = cellfun(@sum, means_obj);

%% Plotting

% Plot stats for obj_hist
plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetup, 'Mean Objective Value', 'Objective Value');
plotStats(means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt, simulationSetup, 'Mean Optimisation Time', 'Optimisation Time');

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
saveSimulationResults(config.flag_save, config);

% %% Replot using only certain lines
% plotStats(means_obj(end-1:end), ci_lower_obj(end-1:end), ci_upper_obj(end-1:end), time_vector_obj(end-1:end), simulationSetup(end-1:end, :), 'Mean Objective Value', 'Objective Value');
% plotStats(means_t_opt(end-1:end), ci_lower_t_opt(end-1:end), ci_upper_t_opt(end-1:end), time_vector_t_opt(end-1:end), simulationSetup(end-1:end, :), 'Mean Optimisation Time', 'Optimisation Time');
