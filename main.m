%% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology

% Clear workspace
clear all
close all

% Set up folder paths
addpath('functions', 'data')

%% Define simulation set
% Simulation sets are defined as a cell array of handles to initialisation
% functions, each row of which defines a simulation in the simulation set

% Define function handles
h_init_sim_1 = @()initialise_simulation();
h_init_env_1 = @(dt_e, k)initialise_environment_01(dt_e, k);
h_init_agt_1 = @(m_bo, l_x_e, l_y_e)initialise_agent_01(m_bo, l_x_e, l_y_e);
h_init_agt_2 = @(m_bo, l_x_e, l_y_e)initialise_agent_02(m_bo, l_x_e, l_y_e);
h_init_agt_3 = @(m_bo, l_x_e, l_y_e)initialise_agent_03(m_bo, l_x_e, l_y_e);
h_init_pp_1 = @(m_bo_s, n_a)initialise_pathPlanning_01(m_bo_s, n_a);
h_init_pp_2 = @(m_bo_s, n_a)initialise_pathPlanning_02(m_bo_s, n_a);
h_init_MPC_1 = @(fisArray, n_a)initialise_MPC_01(fisArray, n_a);
h_init_MPC_2 = @(fisArray, n_a)initialise_MPC_02(fisArray, n_a);
h_init_MPC_3 = @(fisArray, n_a)initialise_MPC_03(fisArray, n_a);
h_init_MPC_4 = @(fisArray, n_a)initialise_MPC_04(fisArray, n_a);

% % Simulation set 1
% simulation_set_name = "SS01";
% simulation_set = {
%   "SS01-1", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_1;
%   "SS01-2", h_init_sim_1, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_2;
%   "SS01-3", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_3;
%   "SS01-4", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_1, h_init_MPC_4;
%   };
% Simulation set 2
% simulation_set_name = "SS02";
% simulation_set = {
%   "SS02-1", h_init_sim_1, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_1;
%   "SS02-2", h_init_sim_1, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_2;
%   "SS02-3", h_init_sim_1, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_3;
%   "SS02-4", h_init_sim_1, h_init_env_1, h_init_agt_2, h_init_pp_1, h_init_MPC_4;
%   };
% Simulation set 3
simulation_set_name = "SS03";
simulation_set = {
  "SS03-1", h_init_sim_1, h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_1;
  "SS03-2", h_init_sim_1, h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_2;
  "SS03-3", h_init_sim_1, h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_3;
  "SS03-4", h_init_sim_1, h_init_env_1, h_init_agt_3, h_init_pp_1, h_init_MPC_4;
  };
% % Simulation set 4
% simulation_set_name = "SS04";
% simulation_set = {
%   "SS04-1", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%   "SS04-2", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_2;
%   "SS04-3", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_3;
%   "SS04-4", h_init_sim_1, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_4;
%   };
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
  m_scan, m_t_scan, m_bo_s] = simulation_set{sim,4}(m_bo, l_x_e, l_y_e);
  % Path planning
  [c_prior_building, c_prior_open, m_prior, fisArray] = simulation_set{sim,5}(m_bo_s, n_a);
  % MPC
  [flag_mpc, solver, n_p, fis_params, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, ...
  nvars, fminsearchOptions, gaOptions, patOptions, parOptions, t_opt] = simulation_set{sim,6}(fisArray, n_a);

  %% Initialise plotting data
  [axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
  m_f_hist, m_f_hist_animate, m_bt_hist_animate, m_t_dw_hist_animate, ...
  m_scan_hist, a_loc_hist, t_hist, fis_param_hist, obj_hist, s_obj_hist] ...
  = initialise_plotting(m_p_ref, n_x_e, n_y_e, n_x_s, n_y_s, m_f, m_bt, n_a, a_loc, k, fis_params);

  %% Define timestep for saving variables
  % Number of desired data points
  n_prog_data = 100;
  % Estimated avg travel time
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

  %% Simulation
  while flag_finish == false
    % Start timer
    t_sim = tic;
    %% MPC
    if flag_mpc
      if k_mpc*dk_mpc <= t
        [fisArray, ini_params, fis_param_hist] = ...
          model_MPC_module(fisArray, ini_params, fis_param_hist, ...
          solver, n_a, ...
          nvars, A, b, Aeq, beq, lb, ub, nonlcon, ...
          fminsearchOptions, gaOptions, patOptions, parOptions, ...
          test_fis_sensitivity, ...
          m_f, m_bo, m_bt, m_s, m_scan, m_t_scan, ...
          dk_a, dk_c, dk_e, dk_mpc, dt_s, k, ...
          n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
          a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
          l_x_s, l_y_s, c_f_s, ...
          c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
          r_bo, r_fo, fis_data);
        % Counter 
        k_mpc = k_mpc + 1;
      end
    end

    %% Path planning
    if k_c*dk_c <= k
      % Counter
      k_c = k_c + 1;
      % Path planner
      a_target = model_pathPlanning(...
        n_a, a_target, n_q, ...
        n_x_s, n_y_s, l_x_s, l_y_s, ...
        m_scan, m_t_scan, m_t_dw, m_prior, ...
        fisArray, ...
        a_t_trav, a_t_scan, ...
        ang_w, v_as, v_w, test_fis_sensitivity); 
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
          l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, false);
    end

    %% Environment model
    if k_e*dk_e <= k
      % Counter 
      k_e = k_e + 1;
      % Environment map
      [m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, ...
        m_bt, m_t_dw] = model_environment(...
        m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, m_s, m_bo, m_bt, ...
        dt_e, k, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, c_f_s, flag_mpc);
    end

    %% Objective function evaluation
    [s_obj, obj] = calc_obj(...
      m_f, m_bo, m_scan, ...
      r_bo, r_fo, dt_s, s_obj, ...
      n_x_e, n_y_e, n_x_s, n_y_s, c_f_s, ...
      test_obj_sensitivity);
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
  exp_folder = strcat(exp_dir, "\", simulation_name);
  exp_file = strcat(exp_folder,"\", simulation_name);

  % Simulation figures
  if flag_fig_sim
    % Generate and export figures 
    simulation_plots = {
      "m_f_hist_animate", m_f_hist_animate, "animate", false;
      "m_t_dw_hist_animate", m_t_dw_hist_animate, "animate", false;
      "m_bo", m_bo, "environment_map", true;
      "m_prior", m_prior, "search_map", true;
      "m_f_hist", m_f_hist, "environment_map", true;
      "m_scan_hist", m_scan_hist, "search_map", true;
      "obj_hist", obj_hist, "variable", false;
      "s_obj_hist", s_obj_hist, "variable", false;
      "a_loc_hist", a_loc_hist, "agent", false;
      "fis_param_hist", fis_param_hist, "fis", false;
    };

    plot_simulationData( simulation_plots, exp_folder, ...
                axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
                t_f, n_x_s, n_y_s, n_a, ct_v, fisArray)
  end

  % Export data
  if flag_data_exp
    mkdir(exp_folder)
    save(exp_file)
  end
end

%% TO DO
%% Simulation set plots
% Plot settings
simulationSet_plots = {
  "obj_hist", false;
  "s_obj_hist", false;
  "fis_param_hist", false;
  };
% Export folder
exp_folder = strcat(exp_dir, "\", simulation_set_name);

if flag_fig_simSet
  % Close open figures
  close all;
  for fig = 1:size(simulationSet_plots,2)
    data_name = simulationSet_plots{fig, 1};

    f = figure("name", data_name);
    hold on;
    [lab_title, lab_x, lab_y, ~] = func_plot_labels(data_name);
    lab_legend = {};

    for sim = 1:size(simulation_set,1)
      % Get file
      data_file = simulation_set{sim, 1};
      simulation_name = strcat(exp_dir, ...
                            "\", data_file, ...
                            "\", data_file, ".mat");
      data_struct = load(simulation_name, data_name, "t_hist");

      % Extract data
      fields = fieldnames(data_struct);
      data = getfield(data_struct, fields{1});
      axis_t = getfield(data_struct, fields{2});

      % Plot
      plot(axis_t, data);
      lab_legend{sim} = data_file;
      % Compare data - using annotation: https://uk.mathworks.com/help/matlab/ref/annotation.html
    end
    % Labels
    title(lab_title);
    xlabel(lab_x);
    ylabel(lab_y);
    lab_legend_arr = string(lab_legend);
    legend(lab_legend_arr); 
    opengl software
  end
  %% Save figures as .fig and .jpg files
  mkdir(exp_folder)
  fig_list = findobj(allchild(0), "Type", "figure");
  for iFig = 1:length(fig_list)
    h_fig = fig_list(iFig); 
    fig_name   = get(h_fig, "Name");
    exp_fig = strcat(exp_folder, "\", fig_name);
    savefig(h_fig, strcat(exp_fig, ".fig"));
    saveas(h_fig, strcat(exp_fig, ".jpg"));
  end
end