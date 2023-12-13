 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REFACTOR PROGRESS

% CURRENT TOPICS
% Inputs may need to be normalized for FIS
% 

% TROUBLESHOOTING
% - check coarsen ratios managed correctly - has been removed for now (set to 1)

% CHANGELOG
% - add flag_scan_task to initialization
% - flag_scan_task needs to be increased at each time step
% - time since last scanned
% - function: generateVictimsMap
% - function: generateBrownNoiseMatrix
% - function: generateMatrixFromPDFs
% - function: compareFunctionOutputs
% - function: runSimulationsAndComputeCI
% - m_victim_scan initialisation changed from environment to agent
% - m_victim_scan added to function inputs: MPC, obj
% - replaced m_t_dw with m_dw
% calc_obj - efficiency improvements
% - feature: fixed implementation of environment and agent maps. m_dw_e and m_dw
% now handled correctly
% - performance: removed environment history parameters for animation
% - update handling of maps: environment defined in environment, agent defined
% in agent. All parameters use respective size maps
% - feature: functions for statistical analysis and plotting confidence
% intervals

% TODO 
% - customisation of all inputs/initializations
% - cleanup main script
% - update victim model
% -- needs real victims in locations based on probability density & needs
% multiple - probability based as well - second probability variable
% -- update objective function - allow addition of 
% - update victim prediction
% - update fire prediction. Keep exact prediction as well
% - add localised predictions
% - add multiple prediction function
% - add other MPC steps from Mirko ()
% - check all functions correct
% - cell rescan
% -- track time since last scan
% -- update objective
% - configure simulation objectives: i.e. 
% - improve simulation progress reports
% - plotting improvements
% - scan progress - use percent of environment or max time? also check how long should it take to
% scan a cell
% - remove hard coded flags
% - add time limit constraint to simulation
% - test ability to return to original task (scan all cells only)
% - implement efficiency improvements (vectorization etc & testing)
% - remove refactor code - will be done properly soon
% - remove test_obj_sensitivity from input file
% - update victim model
% - update UAV objectives  & objective function
% - build multiple simulation & confidence interval
% - better management of seeds for repeatability 
% - see if Initialise plotting data can be removed
% - add m_victim to objective function & uav tasks
% - modify agent functions to use radius if defined
% - remove use of coarsen ratio in initialization of environment
% - add tracking of m_dw for plotting again (to agent model)
% - correct objective function calculation - using agent maps
% - get list of dependencies and remove unnecessary scripts: https://uk.mathworks.com/help/matlab/matlab_prog/identify-dependencies.html
% - EFFICIENCY IMPROVEMENTS: If no active fires can drop fire model
% - move files in main folder to functions folder (using github)
% - improvements: Write a function to convert raster data to a matrix, and save any other relevant variables as a separate variable. This will allow us to convert raster data first and then use the matrix as an input to the  calc_coarsenRatio function.
% - review timesteps method - especially for recording objective function
% values - possibly better to update each time cell is scanned with new
% value or save every  time step
% - update plotting function initialization and plotting function
% - improvement: write data to file after each simulation
% - remove concept of task queue for agents? 
% - update objective function with new m_scan model
% - correct objective function calculation - using agent maps
% TODO: change how simulations are defined


% BUGS
% - appears animate parameters not managed correctly - only one time step
% recorded
% - management of animation parameters slows simulation due to growing size - 1
% million data points in small use case. Rework to only plot for agent and
% record
% - simulations slow down a large amount towards end of simulation due to
% growing size of animation parameters. Refactor animations - separate function
% to record these


% SIMULATION
% - demonstrate confidence intervals
% - demonstrate sensitivity analysis (to initial settings: victim locations,
% fire locations, agent locations)
% - stable solution (no dynamic variable, design with 2/3 agent ideal solution in mind, small disaster environment)
% - stable solution & loss of agent mid-way

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
h_init_SIM_1 = @()initialise_simulation_SIM_1();

% Environment
h_initialise_environment_SIM_basic_no_dynamic= @(dt_e, k)initialise_environment_SIM_basic_no_dynamic(dt_e, k);
h_initialise_environment_SIM_basic_dynamic= @(dt_e, k)initialise_environment_SIM_basic_dynamic(dt_e, k);

% Agent
h_initialise_agent_SIM_single = @(m_bo, m_dw_e, l_x_e, l_y_e)initialise_agent_SIM_single(m_bo, m_dw_e, l_x_e, l_y_e);
h_initialise_agent_SIM_repeat = @(m_bo, m_dw_e, l_x_e, l_y_e)initialise_agent_SIM_repeat(m_bo, m_dw_e, l_x_e, l_y_e);

% FIS
h_init_pp_1 = @(m_bo_s, n_a)initialise_fis_SIM_1(m_bo_s, n_a);

% MPC
h_init_MPC_1 = @(fisArray, n_a)initialise_MPC_01(fisArray, n_a);
h_initialise_MPC_maxfunceval_50 = @(fisArray, n_a)initialise_MPC_maxfunceval_50(fisArray, n_a);

% Handles for solver test
h_init_MPC_SOLV_fminsearch = @(fisArray, n_a)initialise_MPC_ST01_fminsearch(fisArray, n_a);
h_init_MPC_SOLV_ga = @(fisArray, n_a)initialise_MPC_ST01_ga(fisArray, n_a);
h_init_MPC_SOLV_particleswarm = @(fisArray, n_a)initialise_MPC_ST01_particleswarm(fisArray, n_a);
h_init_MPC_SOLV_patternsearch = @(fisArray, n_a)initialise_MPC_ST01_patternsearch(fisArray, n_a);

simulationSetups = {
  "SIM01_sensitivity_FIS", h_init_SIM_1, h_initialise_environment_SIM_basic_no_dynamic, h_initialise_agent_SIM_repeat, h_init_pp_1, h_init_MPC_1;
%   "SIM01_sensitivity_MPC", h_init_SIM_1, h_initialise_environment_SIM_basic_dynamic, h_initialise_agent_SIM_single, h_init_pp_1, h_initialise_MPC_maxfunceval_50;
%   "SIM01_sensitivity_MPC", h_init_SIM_1, h_initialise_environment_SIM_basic_no_dynamic, h_initialise_agent_SIM_repeat, h_init_pp_1, h_initialise_MPC_maxfunceval_50;
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
  f_init_path = simulationSetups{simSetup, 5};
  f_init_mpc = simulationSetups{simSetup, 6};

  % Initialize an array to store results for this simulation setup
  results = struct('t_hist', [], 's_obj_hist', [], 'obj_hist', []);

  for iteration = 1:numIterations
      % Generate and record a unique seed for each iteration
      seeds(iteration) = randi(10000); % or another method to generate a random seed
      rng(seeds(iteration)); % Set the seed for RNG

      %% Initialise simulation data
      [test_fis_sensitivity, test_obj_sensitivity, test_solvers, fis_data, ...
      flag_data_exp, flag_fig_sim, flag_fig_simSet, exp_dir, ...
      t, t_f, dt_s, dk_a, dk_c, dk_e, dk_mpc, dk_prog, dt_a, dt_c, dt_e, dt_mpc, ...
      k, k_a, k_c, k_e, k_mpc, k_prog, endCondition, flag_finish, ...
      obj, s_obj, r_bo, r_fo] = f_init_sim();

      %% Initialise models, plotting data, and timestep for saving variables

      % Environment 
      [l_x_e, l_y_e, n_x_e, n_y_e, ...
        m_bo, m_s, m_f, m_bt, m_dw_e, m_p_ref, ...
        c_fs_1, c_fs_2, v_w, ang_w] = f_init_env(dt_e, k);
      
      % Agent
      [n_x_s, n_y_s, l_x_s, l_y_s, n_a, n_q, v_as, a_t_trav, ...
      t_scan_m, t_scan_c, a_task, a_loc, a_target, a_t_scan, ...
      m_scan, m_t_scan, m_bo_s, m_dw, m_victim_scan, config, c_f_s] = f_init_agent(m_bo, m_dw_e, l_x_e, l_y_e);

      % Path planning
      [c_prior_building, c_prior_open, m_prior, fisArray, n_MF_out] = f_init_path(m_bo_s, n_a);

      % MPC
      [flag_mpc, solver, options, n_p, fis_params, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, nvars] = ...
        f_init_mpc(fisArray, n_a);
      
      % Plotting data
      [axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
      m_f_hist, m_f_hist_animate, m_bt_hist_animate, m_dw_hist_animate, ...
      m_scan_hist, a_loc_hist, t_hist, fis_param_hist, obj_hist, s_obj_hist] ... 
      = initialise_plotting(m_p_ref, n_x_e, n_y_e, n_x_s, n_y_s, m_f, m_bt, n_a, a_loc, k, fis_params);
      
      %% Define timestep for saving variables
      % Number of desired data points
      n_prog_data = 500;
      % Estimated avg travel time
      k_trav_avg = (t_scan_c + l_x_s/v_as)/dt_s;
      % Estimated sim time
      k_sim_est = k_trav_avg * n_x_s * n_y_s / n_a;
      % Save data time
      dk_v = k_sim_est / n_prog_data;
      ct_v = 0;

      %% Simulation Loop
      while flag_finish == false

        % Start timer
        t_sim = tic;

        %% MPC
        if flag_mpc 
          if k_mpc*dk_mpc <= k
            [fisArray, ini_params, fis_param_hist] = ...
              model_mpc(fisArray, ini_params, fis_param_hist, ...
              solver, options, n_a, n_MF_out, ...
              nvars, A, b, Aeq, beq, lb, ub, nonlcon, ...
              test_fis_sensitivity, ...
              m_f, m_bo, m_bt, m_prior, m_s, m_scan, m_t_scan, m_victim_scan, ...
              dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seeds(iteration), ...
              n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
              a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
              l_x_s, l_y_s, c_f_s, ...
              c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
              r_bo, r_fo, fis_data, config, t);     
            
            % Counter 
            k_mpc = k_mpc + 1;
          end
        end

        %% Path planning
        if k_c*dk_c <= k
          % Counter
          k_c = k_c + 1;
          % Path planner
          [a_target, ~] = model_fis(...
            n_a, a_target, n_q, ...
            n_x_s, n_y_s, l_x_s, l_y_s, ...
            m_scan, m_t_scan, m_dw_e, m_prior, ...
            fisArray, ...
            a_t_trav, a_t_scan, ...
            ang_w, v_as, v_w, test_fis_sensitivity, [], c_f_s);   
        end

        %% Agent actions
        if k_a*dk_a <= k
          % Counter 
          k_a = k_a + 1;
          % Agent model
          [ m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, ...
            a_t_trav, a_t_scan] ...
              = model_agent( n_a, ...
              m_t_scan, m_scan, m_scan_hist, ...
              a_loc, a_loc_hist, a_task, a_target, ...
              a_t_trav, a_t_scan, ...
              l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, dt_s);
        end  

        %% Environment model
        if k_e*dk_e <= k
          % Counter 
          k_e = k_e + 1;
          % Environment map
          [m_f, m_bt, m_dw_e] = model_environment(...
            m_f, m_s, m_bo, m_bt, dt_e, k, seeds(iteration), n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2);
        end

        %% Objective function evaluation
        
        [s_obj, obj] = calc_obj(...
          config, m_f, m_bo, m_scan, m_victim_scan, ...
          dt_s, s_obj, c_f_s, t);
        
        %% Store variables
        if ct_v*dk_v <= k
          ct_v = ct_v + 1;
          t_hist(ct_v) = ct_v*dk_v*dt_s;
          s_obj_hist(ct_v)    = s_obj;
          obj_hist(ct_v)      = obj;
%           fis_hist(ct_v)      = ;
        end

        %% Advance timestep
        t = t + dt_s;
        k = k + 1;

        %% Progress report
        if k_prog * dk_prog <= t
          report_progress(endCondition, t, t_f, m_scan, n_x_s, n_y_s);
          k_prog = k_prog + 1;
        end

        %% Check end condition
        [flag_finish] = func_endCondition(endCondition, t, t_f, m_scan, n_x_s, n_y_s);

      end

      % Store results for each iteration
      results(iteration).t_hist = t_hist;
      results(iteration).s_obj_hist = s_obj_hist;
      results(iteration).obj_hist = obj_hist;

  end

    % Store results from this setup for later comparison
    allResults.(simulationName) = results;

end

%% Perform statistical analysis and plotting

alpha = 0.05;  % Confidence interval level

% Calculate stats for s_obj_hist
[means_s_obj, ci_lower_s_obj, ci_upper_s_obj, time_vector_s_obj] = calculateStats(allResults, simulationSetups, 's_obj_hist', dt_s, alpha);

% Calculate stats for obj_hist
[means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj] = calculateStats(allResults, simulationSetups, 'obj_hist', dt_s, alpha);

% Plot stats for s_obj_hist
plotStats(means_s_obj, ci_lower_s_obj, ci_upper_s_obj, time_vector_s_obj, simulationSetups, 'Sum of Objective History Across Simulation Setups', 'Sum of Objective Value');

% Plot stats for obj_hist
plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetups, 'Objective Values and Confidence Intervals Across Simulation Setups', 'Objective Value');




