 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REFACTOR PROGRESS

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
% - update handling of maps: environment defined in environment, agent defined
% in agent. All parameters use respective size maps
% - remove use of coarsen ratio in initialization of environment
% - add tracking of m_dw for plotting again (to agent model)
% - correct objective function calculation - using agent maps
% - get list of dependencies and remove unnecessary scripts: https://uk.mathworks.com/help/matlab/matlab_prog/identify-dependencies.html

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

% Simulation variables
% TODO: rename
h_init_SIM_1 = @()initialise_simulation_SIM_1();

% Environment
h_initialise_environment_SIM_basic_no_dynamic= @(dt_e, k)initialise_environment_SIM_basic_no_dynamic(dt_e, k);
h_initialise_environment_SIM_basic_dynamic= @(dt_e, k)initialise_environment_SIM_basic_dynamic(dt_e, k);

% Agent
h_initialise_agent_SIM_single = @(m_bo, m_dw_e, l_x_e, l_y_e)initialise_agent_SIM_single(m_bo, m_dw_e, l_x_e, l_y_e);
h_initialise_agent_SIM_repeat = @(m_bo, m_dw_e, l_x_e, l_y_e)initialise_agent_SIM_repeat(m_bo, m_dw_e, l_x_e, l_y_e);

% Path planning
% TODO: rename
h_init_pp_1 = @(m_bo_s, n_a)initialise_pathPlanning_SIM_1(m_bo_s, n_a);

% MPC
% TODO: rename not active
h_init_MPC_1 = @(fisArray, n_a)initialise_MPC_01(fisArray, n_a);
h_initialise_MPC_maxfunceval_50 = @(fisArray, n_a)initialise_MPC_maxfunceval_50(fisArray, n_a);

% Handles for solver test
h_init_MPC_SOLV_fminsearch = @(fisArray, n_a)initialise_MPC_ST01_fminsearch(fisArray, n_a);
h_init_MPC_SOLV_ga = @(fisArray, n_a)initialise_MPC_ST01_ga(fisArray, n_a);
h_init_MPC_SOLV_particleswarm = @(fisArray, n_a)initialise_MPC_ST01_particleswarm(fisArray, n_a);
h_init_MPC_SOLV_patternsearch = @(fisArray, n_a)initialise_MPC_ST01_patternsearch(fisArray, n_a);

simulationSetups = {
  "SIM01_sensitivity_FIS", h_init_SIM_1, h_initialise_environment_SIM_basic_dynamic, h_initialise_agent_SIM_single, h_init_pp_1, h_init_MPC_1;
%   "SIM01_sensitivity_MPC", h_init_SIM_1, h_initialise_environment_SIM_basic_dynamic, h_initialise_agent_SIM_single, h_init_pp_1, h_initialise_MPC_maxfunceval_50;
  };

% Define the number of iterations for each simulation setup
numIterations = 2; 

% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);

% Initialize a structure to store results from all setups
allResults = struct();

% Initialize arrays to store results and seeds for statistical analysis
results = struct('t_hist', [], 's_obj_hist', [], 'obj_hist', []);

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
          % Include your simulation logic here...

          % Start timer
        t_sim = tic;

        %% MPC
        if flag_mpc 
          if k_mpc*dk_mpc <= k
            [fisArray, ini_params, fis_param_hist] = ...
              model_MPC_module(fisArray, ini_params, fis_param_hist, ...
              solver, options, n_a, n_MF_out, ...
              nvars, A, b, Aeq, beq, lb, ub, nonlcon, ...
              test_fis_sensitivity, ...
              m_f, m_bo, m_bt, m_prior, m_s, m_scan, m_t_scan, m_victim_scan, ...
              dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seeds(iteration), ...
              n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
              a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
              l_x_s, l_y_s, c_f_s, ...
              c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
              r_bo, r_fo, fis_data, config);
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
            m_scan, m_t_scan, m_dw, m_prior, ...
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
              l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, false, config);
        end  

        %% Environment model
        if k_e*dk_e <= k
          % Counter 
          k_e = k_e + 1;
          % Environment map
          [m_f, m_f_hist, m_f_hist_animate, m_dw_hist_animate, ...
            m_bt, m_dw] = model_environment(...
            m_f, m_f_hist, m_f_hist_animate, m_dw_hist_animate, m_s, m_bo, m_bt, ...
            dt_e, k, seeds(iteration), n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, flag_mpc);
        end

        %% Objective function evaluation
        
        [s_obj, obj] = calc_obj(...
          config, m_f, m_bo, m_scan, m_victim_scan, ...
          dt_s, s_obj, n_x_e, n_y_e, c_f_s);
        
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

      % Store results for each iteration
      results(iteration).t_hist = t_hist;
      results(iteration).s_obj_hist = s_obj_hist;
      results(iteration).obj_hist = obj_hist;

      % Postprocessing (if any)
      % ...
  end

    % Store results from this setup for later comparison
    allResults.(simulationName) = results;
    
    % TEST: plot fire map
    simulation_plots = {
      "m_f_hist", m_f_hist, "environment_map", true;
    };

    plot_simulationData( simulation_plots, "", ...
                axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
                t_f, n_x_s, n_y_s, n_a, ct_v, fisArray, dt_s, false)


end

%% Perform statistical analysis and plotting
% TODO: plot CI for s_obj_hist

close all

% Define the confidence level (e.g., 0.05 for 95% confidence)
alpha = 0.05;

% Initialize figures for plotting
figureObjValues = figure;
title('Objective Values and Confidence Intervals Across Simulation Setups');
xlabel('Time (s)');
ylabel('Objective Value');
hold on;

figureSObjHist = figure;
title('Sum of Objective History Across Simulation Setups');
xlabel('Time (s)');
ylabel('Sum of Objective Value');
hold on;

% Iterate over each simulation setup
for simSetup = 1:size(simulationSetups, 1)
    simulationName = simulationSetups{simSetup, 1};

    % Extract the objective history and s_obj_hist from allResults for the current setup
    obj_hist_data = allResults.(simulationName).obj_hist;
    s_obj_hist_data = allResults.(simulationName).s_obj_hist;

    % Calculate mean, standard deviation, and confidence intervals for obj_hist
    mean_obj_hist = mean(obj_hist_data, 3);
    std_obj_hist = std(obj_hist_data, 0, 3);
    ci_half_width = tinv(1 - alpha/2, numIterations - 1) * std_obj_hist / sqrt(numIterations);
    ci_lower = mean_obj_hist - ci_half_width;
    ci_upper = mean_obj_hist + ci_half_width;

    % Time vector for plotting
    time_vector = (1:size(mean_obj_hist, 2)) * dt_s;  % Assuming equal time steps (dt_s)

    % Plot Mean Objective Values and Confidence Intervals
    figure(figureObjValues);
    plot(time_vector, mean_obj_hist, 'DisplayName', strcat(simulationName, ' Mean'));
    plot(time_vector, ci_lower, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', '--');
    plot(time_vector, ci_upper, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', '--');
    
    % Plot Sum of Objective History
    figure(figureSObjHist);
    plot(time_vector, s_obj_hist_data, 'DisplayName', strcat(simulationName, ' Sum of Obj'));
end

% Add legend and grid to the Objective Values plot
figure(figureObjValues);
legend('show');
grid on;
% saveas(gcf, 'Objective_Values_and_Confidence_Intervals.png');

% Add legend and grid to the Sum of Objective History plot
figure(figureSObjHist);
legend('show');
grid on;
% saveas(gcf, 'Sum_of_Objective_History.png');

%% Additional plots

simulation_plots = {
  "m_f_hist", m_f_hist, "environment_map", true;
};

plot_simulationData( simulation_plots, "", ...
            axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
            t_f, n_x_s, n_y_s, n_a, ct_v, fisArray, dt_s, false)

%% Statistical Analysis
% % Include the full scripts for statistical analysis after this comment
% % Example:
% mean_s_obj_hist = mean(cat(1, results.s_obj_hist), 1);
% std_s_obj_hist = std(cat(1, results.s_obj_hist), 0, 1);
% 
% % Calculating Confidence Intervals
% % Assuming a 95% confidence interval and normal distribution
% alpha = 0.05;
% ci_half_width = tinv(1-alpha/2, numIterations-1) * std_s_obj_hist / sqrt(numIterations);
% ci_lower = mean_s_obj_hist - ci_half_width;
% ci_upper = mean_s_obj_hist + ci_half_width;
% 
% %% Plot results
% % Assuming you have already calculated mean_s_obj_hist, ci_lower, and ci_upper
% % as shown in the previous script
% 
% % Example: Plotting mean_s_obj_hist with confidence intervals
% 
% % Generate time vector if not already present
% % (Assuming each entry in mean_s_obj_hist corresponds to a regular time interval)
% time_vector = (1:length(mean_s_obj_hist)) * dt_s;  % dt_s is your timestep
% 
% % Create a figure for the plot
% figure;
% 
% % Plot mean values
% plot(time_vector, mean_s_obj_hist, 'LineWidth', 2);
% hold on;
% 
% % Add confidence interval as a shaded area
% fill([time_vector, fliplr(time_vector)], ...
%      [ci_lower, fliplr(ci_upper)], ...
%      'b', 'FaceAlpha', 0.1, 'EdgeAlpha', 0);  % Adjust color and transparency as needed
% 
% % Add labels and title
% xlabel('Time (s)');
% ylabel('Mean Objective Value');
% title('Mean Objective Value with 95% Confidence Interval');
% legend('Mean Objective Value', '95% Confidence Interval');
% 
% % Optionally, add grid for better readability
% grid on;
% 
% % Adjust other plot settings as needed
% set(gca, 'FontSize', 12);  % Adjust font size
% 
% % Save the plot if needed
% % saveas(gcf, 'path/to/save/plot.png');
% 
% % TODO: Create separate functions for the statistical analysis and plotting.
