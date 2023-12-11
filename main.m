 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REFACTOR PROGRESS
% COMPLETE

% PARTIAL
% calc_obj - efficiency improvements

% CHANGELOG
% - add flag_scan_task to initialization
% - flag_scan_task needs to be increased at each time step
% - time since last scanned
% - function: generateVictimsMap
% - function: generateBrownNoiseMatrix
% - function: generateMatrixFromPDFs
% - function: compareFunctionOutputs
% - function: runSimulationsAndComputeCI

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

% EFFICIENCY IMPROVEMENTS
% - If no active fires can drop fire model

% IMPROVEMENTS
% Write a function to convert raster data to a matrix, and save any other relevant variables as a separate variable. This will allow us to convert raster data first and then use the matrix as an input to the  calc_coarsenRatio function.

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
% TODO: change how simulations are defined

% Simulation variables - different mpc step sizes
h_init_sim_10 = @()initialise_simulation_10();
h_init_sim_50 = @()initialise_simulation_50();
h_init_sim_100 = @()initialise_simulation_100();
h_init_sim_200 = @()initialise_simulation_200();
h_init_SIM_1 = @()initialise_simulation_SIM_1();

h_init_env_1 = @(dt_e, k)initialise_environment_01(dt_e, k);
h_init_env_2 = @(dt_e, k)initialise_environment_02(dt_e, k);
h_init_env_SIM_1 = @(dt_e, k)initialise_environment_SIM_1(dt_e, k);

% Agent
h_init_agt_1 = @(m_bo, l_x_e, l_y_e)initialise_agent_01(m_bo, l_x_e, l_y_e);
h_init_agt_2 = @(m_bo, l_x_e, l_y_e)initialise_agent_02(m_bo, l_x_e, l_y_e);
h_init_agt_3 = @(m_bo, l_x_e, l_y_e)initialise_agent_03(m_bo, l_x_e, l_y_e);

% Path planning
h_init_pp_1 = @(m_bo_s, n_a)initialise_pathPlanning_01(m_bo_s, n_a);
h_init_pp_2 = @(m_bo_s, n_a)initialise_pathPlanning_02(m_bo_s, n_a);

% MPC not active
h_init_MPC_1 = @(fisArray, n_a)initialise_MPC_01(fisArray, n_a);

% MPC with optTermCond = 'MaxFunctionEvaluations'; / n_p = 1
h_init_MPC_maxfunceval_10 = @(fisArray, n_a)initialise_MPC_maxfunceval_10(fisArray, n_a);
h_init_MPC_maxfunceval_50 = @(fisArray, n_a)initialise_MPC_maxfunceval_50(fisArray, n_a);
h_init_MPC_maxfunceval_100 = @(fisArray, n_a)initialise_MPC_maxfunceval_100(fisArray, n_a);
h_init_MPC_maxfunceval_10_2 = @(fisArray, n_a)initialise_MPC_maxfunceval_10_2(fisArray, n_a);
h_init_MPC_maxfunceval_50_2 = @(fisArray, n_a)initialise_MPC_maxfunceval_50_02(fisArray, n_a);
h_init_MPC_maxfunceval_100_2 = @(fisArray, n_a)initialise_MPC_maxfunceval_100_02(fisArray, n_a);

% Handles for solver test
h_init_MPC_SOLV_fminsearch = @(fisArray, n_a)initialise_MPC_ST01_fminsearch(fisArray, n_a);
h_init_MPC_SOLV_ga = @(fisArray, n_a)initialise_MPC_ST01_ga(fisArray, n_a);
h_init_MPC_SOLV_particleswarm = @(fisArray, n_a)initialise_MPC_ST01_particleswarm(fisArray, n_a);
h_init_MPC_SOLV_patternsearch = @(fisArray, n_a)initialise_MPC_ST01_patternsearch(fisArray, n_a);

simulation_set_name = "SIM01";
multiSim = false;
simulation_set = {
  "SIM01-1", h_init_SIM_1, h_init_env_SIM_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
  };

simulation_set_names = [
  "Simulation 1";
  ];

% SIMULATION 1
% 
% SETUP
% - 
% 
% TODO
% - disable fire
% - update victim model
% - small environment & define building occupancy 
% - update UAV objectives  & objective function
% - build multiple simulation & confidence interval
% - better management of seeds for repeatability 
%   TODO: better method for seeds (& repeatable)

if ~multiSim
  numIterations = 1;
end

for iteration = 1:numIterations
  % Set seed for this iteration
  if multiSim
    seed = posixtime(datetime('now'));
  else
    seed = 0;
  end
    
  for sim = 1:size(simulation_set,1)

    % Export folder for simulation
    simulation_name = simulation_set{sim, 1};
    
    %% Initialise simulation data
    [test_fis_sensitivity, test_obj_sensitivity, test_solvers, fis_data, ...
    flag_data_exp, flag_fig_sim, flag_fig_simSet, exp_dir, ...
    t, t_f, dt_s, dk_a, dk_c, dk_e, dk_mpc, dk_prog, dt_a, dt_c, dt_e, dt_mpc, ...
    k, k_a, k_c, k_e, k_mpc, k_prog, endCondition, flag_finish, ...
    obj, s_obj, r_bo, r_fo] = simulation_set{sim,2}();
  
    %% Initialise models
    
    % Environment 
    [l_x_e, l_y_e, n_x_e, n_y_e, ...
      m_bo, m_s, m_f, m_bt, m_t_dw, m_p_ref, ...
      c_fs_1, c_fs_2, c_f_s, v_w, ang_w] = simulation_set{sim,3}(dt_e, k);
    
    % Agent
    [n_x_s, n_y_s, l_x_s, l_y_s, n_a, n_q, v_as, a_t_trav, ...
    t_scan_m, t_scan_c, a_task, a_loc, a_target, a_t_scan, ...
    m_scan, m_t_scan, m_bo_s, flag_scan_task] = simulation_set{sim,4}(m_bo, l_x_e, l_y_e);
  
    % Path planning
    [c_prior_building, c_prior_open, m_prior, fisArray, n_MF_out] = simulation_set{sim,5}(m_bo_s, n_a);
    
    % MPC
    [flag_mpc, solver, options, n_p, fis_params, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, nvars] = ...
      simulation_set{sim,6}(fisArray, n_a);

    %% Initialise plotting data
    [axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
    m_f_hist, m_f_hist_animate, m_bt_hist_animate, m_t_dw_hist_animate, ...
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

    %% Simulation
    while flag_finish == false
      % Start timer
      t_sim = tic;
      
      %% MPC
      if flag_mpc 
        if k_mpc*dk_mpc <= k
          [fisArray, ini_params, fis_param_hist] = ...
            model_MPC_module_02(fisArray, ini_params, fis_param_hist, ...
            solver, options, n_a, n_MF_out, ...
            nvars, A, b, Aeq, beq, lb, ub, nonlcon, ...
            test_fis_sensitivity, ...
            m_f, m_bo, m_bt, m_prior, m_s, m_scan, m_t_scan, ...
            dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seed, ...
            n_p, n_x_s, n_y_s, n_x_f, n_y_e, n_q, ...
            a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
            l_x_s, l_y_s, c_f_s, ...
            c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
            r_bo, r_fo, fis_data, flag_scan_task);
          % Counter 
          k_mpc = k_mpc + 1;
        end
      end

      %% Path planning
      if k_c*dk_c <= k
        % Counter
        k_c = k_c + 1;
        % Path planner
        [a_target, ~] = model_pathPlanning(...
          n_a, a_target, n_q, ...
          n_x_s, n_y_s, l_x_s, l_y_s, ...
          m_scan, m_t_scan, m_t_dw, m_prior, ...
          fisArray, ...
          a_t_trav, a_t_scan, ...
          ang_w, v_as, v_w, test_fis_sensitivity, []);   
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
            l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, false, flag_scan_task);
      end  
      
      %% Environment model
      if k_e*dk_e <= k
        % Counter 
        k_e = k_e + 1;
        % Environment map
        [m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, ...
          m_bt, m_t_dw] = model_environment(...
          m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, m_s, m_bo, m_bt, ...
          dt_e, k, seed, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, c_f_s, flag_mpc);
      end
     
      %% Objective function evaluation
      [s_obj, obj] = calc_obj(...
        m_f, m_bo, m_scan, ...
        r_bo, r_fo, dt_s, s_obj, ...
        n_x_e, n_y_e, c_f_s, flag_scan_task);
      
      %% Store variables
      if ct_v*dk_v <= k
        ct_v = ct_v + 1;
        t_hist(ct_v) = ct_v*dk_v*dt_s;
        s_obj_hist(ct_v)    = s_obj;
        obj_hist(ct_v)      = obj;
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

    % Simulation time  
    t_end = toc(t_sim);

    %% Postprocessing
    if numIterations > 1
      simulation_name = strcat(simulation_name, "_iter_", num2str(iteration));
    end
    exp_folder = strcat(exp_dir, "\", simulation_name);
    exp_file = strcat(exp_folder,"\", simulation_name);

    % Simulation figures
    if flag_fig_sim
%       % Generate and export figures 
      simulation_plots = {
        "m_f_hist", m_f_hist, "environment_map", true;
      };
%         "m_f_hist_animate", m_f_hist_animate, "animate", false;
%         "m_t_dw_hist_animate", m_t_dw_hist_animate, "animate", false;
%         "m_bo", m_bo, "environment_map", false;
%         "m_prior", m_prior, "search_map", false;
%         "m_scan_hist", m_scan_hist, "search_map", false;
%         "obj_hist", obj_hist, "variable", false;
%         "s_obj_hist", s_obj_hist, "variable", false;
%         "a_loc_hist", a_loc_hist, "agent", false;
%         "fis_param_hist", fis_param_hist, "fis", false;

      plot_simulationData( simulation_plots, exp_folder, ...
                  axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
                  t_f, n_x_s, n_y_s, n_a, ct_v, fisArray, dt_s)
    end

    % Export data 
    if flag_data_exp
      mkdir(exp_folder)
      save(exp_file)
    end
    
  end
  
  % Plot settings
  plots_simSet = { 
    "obj_hist", "variable", true;
    "obj_hist", "relative", true;
    "s_obj_hist", "variable", true;
    "s_obj_hist", "relative", true;
    "fis_param_hist", "fis", false;
    };
  
  % Plotting function
  plot_simulationComparisons(plots_simSet, exp_dir, simulation_set, simulation_set_name, simulation_set_names);
end

% SIMULATIONS FROM THESIS V1

%% Simulations initialising MPC with pre-tuned parameters and using max function evaluation limit
% Simulation set 1
% simulation_set_name = "SS01";
% multiSim = false;
% simulation_set = {
%   "SS01-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
%   "SS01-2", h_init_sim_10, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS01-3", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS01-4", h_init_sim_100, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS01-5", h_init_sim_10, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   "SS01-6", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   "SS01-7", h_init_sim_100, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   "Simulation 3";
%   "Simulation 4";
%   "Simulation 5";
%   "Simulation 6";
%   "Simulation 7";
%   ];
%  
% Simulation set 2 - n_q = 3
% simulation_set_name = "SS02";
% multiSim = false;
% simulation_set = {
%   "SS02-1", h_init_sim_50, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_1;
%   "SS02-2", h_init_sim_10, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS02-3", h_init_sim_50, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS02-4", h_init_sim_100, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS02-5", h_init_sim_10, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   "SS02-6", h_init_sim_50, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   "SS02-7", h_init_sim_100, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   "Simulation 3";
%   "Simulation 4";
%   "Simulation 5";
%   "Simulation 6";
%   "Simulation 7";
%   ];
% Simulation set 3 - n_a = 3
% simulation_set_name = "SS03";
% multiSim = false;
% simulation_set = {
%   "SS03-1", h_init_sim_50,   h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_1;
%   "SS03-2", h_init_sim_10,  h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS03-3", h_init_sim_50,   h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS03-4", h_init_sim_100,   h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_maxfunceval_50;
%   "SS03-5", h_init_sim_10,  h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   "SS03-6", h_init_sim_50,   h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   "SS03-7", h_init_sim_100,   h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_maxfunceval_50_2;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   "Simulation 3";
%   "Simulation 4";
%   "Simulation 5";
%   "Simulation 6";
%   "Simulation 7";
%   ];
% Simulation set 4 - n_mf_out = 2
% simulation_set_name = "SS04";
% multiSim = false;
% simulation_set = {
%   "SS04-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS04-2", h_init_sim_10, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   "SS04-3", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   "SS04-4", h_init_sim_100, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   "SS04-5", h_init_sim_10, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;
%   "SS04-6", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;
%   "SS04-7", h_init_sim_100, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;
%   "SS04-8", h_init_sim_200, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   "SS04-9", h_init_sim_200, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   "Simulation 3";
%   "Simulation 4";
%   "Simulation 5";
%   "Simulation 6";
%   "Simulation 7";
%   "Simulation 8";
%   "Simulation 9";
%   ];
% simulation_set_name = "SS04-comp-1";
% multiSim = false;
% simulation_set = {
%   "SS01-1";
%   "SS01-2";
%   "SS01-3";
%   "SS01-4";
%   "SS04-2";
%   "SS04-3";
%   "SS04-4";
%   };
% simulation_set_names = [
%   "SS01-1";
%   "SS01-5";
%   "SS01-6";
%   "SS01-7";
%   "SS04-5";
%   "SS04-6";
%   "SS04-7";
%   ];
% simulation_set_name = "SS04-comp-2";
% multiSim = false;
% simulation_set = {
%   "SS01-1";
%   "SS01-5";
%   "SS01-6";
%   "SS01-7";
%   "SS04-5";
%   "SS04-6";
%   "SS04-7";
%   };
% simulation_set_names = [
%   "SS01-1";
%   "SS01-2";
%   "SS01-3";
%   "SS01-4";
%   "SS04-2";
%   "SS04-3";
%   "SS04-4";
%   ];

% Simulation set 5 - number of function evaluations
% simulation_set_name = "SS05";
% multiSim = false;
% simulation_set = {
%   "SS05-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS05-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_10;
%   "SS05-3", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   "SS05-4", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_100;
%   "SS05-5", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_10_2;
%   "SS05-6", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;
%   "SS05-7", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_100_2;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   "Simulation 3";
%   "Simulation 4";
%   "Simulation 5";
%   "Simulation 6";
%   "Simulation 7";
%   ];
% % Simulation set 6 - with and without active fire 
% simulation_set_name = "SS06";
% multiSim = false;
% simulation_set = {
%   "SS06-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS06-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   "SS06-3", h_init_sim_50, h_init_env_2, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS06-4", h_init_sim_50, h_init_env_2, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   };
% simulation_set_names = [
%   "Fire model active, FIS";
%   "Fire model active, MPFC";
%   "Fire model not active, FIS";
%   "Fire model not active, MPFC";
%   ];
% simulation_set_name = "SS06-part1";
% multiSim = false;
% simulation_set = {
%   "SS06-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS06-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   ];
% simulation_set_names = [
%   "Fire model active, FIS";
%   "Fire model active, MPFC";
%   ];
% simulation_set_name = "SS06-part2";
% multiSim = false;
% simulation_set = {
%   "SS06-3", h_init_sim_50, h_init_env_2, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS06-4", h_init_sim_50, h_init_env_2, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50;
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   ];
% simulation_set_names = [
%   "Fire model not active, FIS";
%   "Fire model not active, MPFC";
%   ];

% Simulation set 7 - average performance by randomising rng seeding - think
% about how to do this - may need to write another script which calls main.m
% multiple times
% simulation_set_name = "SS07";
% multiSim = true;
% numIterations = 10;
% simulation_set = {
%   "SS07-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS07-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;  
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   ];
% simulation_set_name = "SS08";
% numIterations = 10;
% multiSim = true;
% simulation_set = {
%   "SS08-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS08-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_100;
%   "SS08-3", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;  
%   };
% simulation_set_names = [
%   "Simulation 1";
%   "Simulation 2";
%   "Simulation 3";
%   ];

%% Tests - currently incomplete
% % Solver test
% simulation_set_name = "ST01-SOLVER";
% simulation_set = {
%   "ST01-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_SOLV_fminsearch;
%   "ST01-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_SOLV_ga;
%   "ST01-3", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_SOLV_particleswarm;
%   "ST01-4", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_SOLV_patternsearch;
%   };

% % FIS sensitivity
% simulation_set_name = "ST02-FIS-SENSITIVITY";

% % Obj sensitivity
% simulation_set_name = "ST03-OBJ-SENSITIVITY";
% simulation_set = {
%   "ST03-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
%   "ST03-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
%   "ST03-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
%   "ST03-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
%   };

%% V1 ARCHIVE

%     %% Test setup
%     % Objective function sensitivity test setup
%     if test_obj_sensitivity
%       p1_i = fis_params(1);
%       p2_i = fis_params(2);
%       p3_i = fis_params(3);
%       p4_i = fis_params(4);
%       obj_hist_eval   = [];
%       obj_hist_sens   = [];
%       ct_mpc_eval     = 0;
%       ct_mpc_sens     = 0; 
%       ct_mpc_sens_fin = 2;
% 
%       % Text Variables
%       % Check these rangeps work properly
%       n_sens_1  = 3;
%       n_sens_2  = 3;
%       n_sens_3  = 3;
%       n_sens_4  = 3;
%       r_sens    = 1;
%       p1        = p1_i*linspace(1-r_sens, 1+r_sens, n_sens_1);
%       p2        = p2_i*linspace(1-r_sens, 1+r_sens, n_sens_2);
%       p3        = p3_i*linspace(1-r_sens, 1+r_sens, n_sens_3);
%       p4        = p4_i*linspace(1-r_sens, 1+r_sens, n_sens_4);
%     end