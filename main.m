 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology 

% FEATURE BACKLOG
% - Feature: Add battery recharge model to FIS
% - Feature: MPFC/MPC prediction with clustering
% - Feature: Implement slow dynamic environment variable
% - Feature: Deterministic threshold prediction mode
% - Feature: Scheduled cells methodology can be removed with new FIS inputs formulation
% - Feature: Remove downwind map - defunct if not used as a FIS input
% - Feature: Remove options_firstEval - just use same options for all MPC evals

% FEATURE - in future simulations initialise randomly using e.g.
  % numCell = n_x_e*n_y_e;
  % randIndices = randperm(numCell, 4);
  % m_f(randIndices) = 3;

% Main script
% 1. In section 1, handles to the initialisation functions are defined
% 2. 

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
  
%% 1. Define function handles 

% Simulation variables
h_s_comms_disabled_no_victim_model = @()i_sim_comms_disabled();
h_s_comms_disabled_victim_model = @()i_sim_comms_disabled_victim_model();
h_s_comms_disabled_victim_model_5000 = @()i_sim_comms_disabled_victim_model_5000();
h_s_comms_disabled_victim_model_5000_local_map_r3 = @()i_sim_comms_disabled_victim_model_5000_local_map_r3();
h_s_comms_disabled_victim_model_5000_local_map_r5 = @()i_sim_comms_disabled_victim_model_5000_local_map_r5();
h_s_comms_disabled_victim_model_10000_local_map_r5 = @()i_sim_comms_disabled_victim_model_10000_local_map_r5();
h_s_comms_disabled_victim_model_5000_local_map_r7 = @()i_sim_comms_disabled_victim_model_5000_local_map_r7();

h_s_comms_disabled_victim_model_20000 = @()i_sim_comms_disabled_victim_model_20000();
h_s_comms_disabled_victim_model_20000_r_5 = @()i_sim_comms_disabled_victim_model_20000_r_5();
h_s_comms_disabled_victim_model_20000_r_10 = @()i_sim_comms_disabled_victim_model_20000_r_10();
h_s_comms_disabled_victim_model_20000_r_15 = @()i_sim_comms_disabled_victim_model_20000_r_15();

h_sim_comms_disabled_victim_model_mpc_2_pred_17= @()i_sim_comms_disabled_victim_model_mpc_2_pred_17();
h_sim_comms_disabled_victim_model_mpc_5_pred_20= @()i_sim_comms_disabled_victim_model_mpc_5_pred_20();
h_sim_comms_disabled_victim_model_mpc_15_pred_30= @()i_sim_comms_disabled_victim_model_mpc_15_pred_30();
h_sim_comms_disabled_victim_model_mpc_30_pred_30= @()i_sim_comms_disabled_victim_model_mpc_30_pred_30();
h_sim_comms_disabled_victim_model_mpc_30_pred_60= @()i_sim_comms_disabled_victim_model_mpc_30_pred_60();
h_sim_comms_disabled_victim_model_mpc_30_pred_75= @()i_sim_comms_disabled_victim_model_mpc_30_pred_75();
h_sim_comms_disabled_victim_model_mpc_45_pred_60= @()i_sim_comms_disabled_victim_model_mpc_45_pred_60();
h_sim_comms_disabled_victim_model_mpc_60_pred_75= @()i_sim_comms_disabled_victim_model_mpc_60_pred_75();

% Environment
h_env_static_40 = @(dt_e)i_env_static_40(dt_e);
h_env_dynamics_20 = @(dt_e)i_env_dynamics_20(dt_e);
h_env_dynamics_40 = @(dt_e)i_env_dynamics_40(dt_e);
h_env_dynamics_60 = @(dt_e)i_env_dynamics_60(dt_e);
h_env_dynamics_80 = @(dt_e)i_env_dynamics_80(dt_e);
h_env_static_200_dualCentre = @(dt_e)i_env_static_200_dualCentre(dt_e);
h_env_dynamics_200_dualCentre = @(dt_e)i_env_dynamics_200_dualCentre(dt_e);
h_env_dynamics_100_dualCentre = @(dt_e)i_env_dynamics_100_dualCentre(dt_e);

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
h_init_fis_mirko_4_type2 = @(n_a)initialise_fis_mirko_4_type2(n_a);
h_init_fis_mirko_4_recharge = @(n_a)initialise_fis_mirko_4_recharge(n_a);
h_init_fis_mod = @(n_a)initialise_fis_mod(n_a);  
   
% Controller Architecture
h_arch_fis = @(fisArray, agent_model)i_arch_fis(fisArray, agent_model);
% h_arch_mpfc_input_exact = @(fisArray, agent_model)i_arch_mpfc_input_exact(fisArray, agent_model);
h_arch_mpfc_output_exact = @(fisArray, agent_model)i_arch_mpfc_output_exact(fisArray, agent_model);
h_arch_mpfc_output_exact_decentralised = @(fisArray, agent_model)i_arch_mpfc_output_exact_decentralised(fisArray, agent_model);
h_arch_mpc_exact = @(fisArray, agent_model)i_arch_mpc_exact(fisArray, agent_model);
% h_arch_mpfc_input_prediction = @(fisArray, agent_model)i_arch_mpfc_input_prediction(fisArray, agent_model);
h_arch_mpfc_output_prediction = @(fisArray, agent_model)i_arch_mpfc_output_prediction(fisArray, agent_model);
h_arch_mpfc_output_prediction_decentralised = @(fisArray, agent_model)i_arch_mpfc_output_prediction_decentralised(fisArray, agent_model);
h_arch_mpc_prediction = @(fisArray, agent_model)i_arch_mpc_prediction(fisArray, agent_model);
h_arch_mpc_prediction_decentralised = @(fisArray, agent_model)i_arch_mpc_prediction_decentralised(fisArray, agent_model);

%% 2. Simulation setup 

seeds = [8904, 6149, 5712, 3194, 6791];  % Set seeds if required
simulationSetup = { 
  "sim_predMode_probThreshold", h_sim_comms_disabled_victim_model_mpc_30_pred_30, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Probability Threshold Prediction";
  "sim_predMode_exact", h_sim_comms_disabled_victim_model_mpc_30_pred_30, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_exact, "MPFC Exact Prediction";
};  

% SIMULATION BACKLOG
% 1. Demo Local prediciton
% 2. Local prediction large environment
% 3. Prediction horizon comparison

% -------------------------------------------------------------------------

% SIMULATION 1: Two agents static - DONE - V4
% Simulation: 
% Plots: 
% simulationSetup = { 
  % "sim_fis_mirko", h_s_comms_disabled_victim_model _5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
  % "sim_mpfc_centralised", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
  % "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
  % "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };  

% SIMULATION 2: Two agents dynamic - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
  % "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
  % "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
  % "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_exact, "Centralised MPC";
% };  

% NOTE: remove battery recharge locations from env model / agent model
% % SIMULATION: Loss of agent (STATIC)
% % Simulation:
% % Plots: 
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_2_low_battery, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mpfc_centralised", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_2_low_battery, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
% };  

% SIMULATION 3: Two agents dynamic - cell_fire_proximity vs m_dw FIS input - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = { 
%   "sim_mpfc_centralised_mod", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mod, h_arch_mpfc_output_prediction, "MPFC Centralised Downwind Input";
%   "sim_fis_mirko", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised Mirko Inputs";
%   "sim_mpc", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };  

% SIMULATION 4: % Three agents - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_3_mpc, h_init_fis_mirko_4, h_arch_mpc_exact, "Centralised MPC";
% };  

% SIMULATION 5: % Four agents - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_exact, "Centralised MPC";
% };  

% SIMULATION 6: ENVIRONMENT SIZE - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mpfc_centralised_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised Mirko Inputs";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };  

% SIMULATION 6: ENVIRONMENT SIZE - 80 (sensitivity)
% Plots: 
% simulationSetup = { 
%   "sim_fis_mirko", h_s_comms_disabled_victim_model_5000, h_env_dynamics_80, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FLC ENV 80";
%   "sim_mpfc_centralised", h_s_comms_disabled_victim_model_5000, h_env_dynamics_80, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Centralised ENV 80";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model_5000, h_env_dynamics_80, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Decentralised ENV 80";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_80, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "MPC ENV 80";
% };  

% SIMULATION 7: PREDICTION MODES COMPARISON
% Simulation: DONE
% Plots: 
% seeds = [8904, 6149, 5712, 3194, 6791];  % Set seeds if required
% simulationSetup = { 
%   "sim_predMode_probThreshold", h_sim_comms_disabled_victim_model_mpc_60_pred_75, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpfc_output_exact, "MPFC Probability Threshold Prediction";
%   "sim_predMode_exact", h_sim_comms_disabled_victim_model_mpc_60_pred_75, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Exact Prediction";
% };  

% SIMULATION X: PROXIMITY FIS INPUT (SHOW PERCENT IMPROVEMENT) - DONE - V4

% SIMULATION 8: Two agents, double population centre
% Simulation: DONE
% Plots: 
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2, h_init_fis_2, h_arch_fis, "Pre-tuned FLC";
%   "sim_mfpc_2", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2, h_init_fis_2, h_arch_mpfc_output_prediction, "MPFC 2 inputs";
  % "sim_mfpc_3", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2, h_init_fis_proximity, h_arch_mpfc_output_prediction, "MPFC 3 inputs";
%   "sim_mpc", h_s_comms_disabled_victim_model, h_e_, h_a_repeat_2_mpc, h_init_fis_2, h_arch_mpc_prediction, "Centralised MPC";
  % }; 

% SIMULATION 9: Decentralised vs centralised structures for MPFC - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mpfc_decentralised", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "sim_mpfc_centralised", h_s_comms_disabled_victim_model, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   }; 

% SIMULATION 10: Compare FIS input parameters - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = {
  % "sim_fis", h_s_comms_disabled_victim_model, h_env_dynamics_60, h_a_repeat_2, h_init_fis_2, h_arch_fis, "Pre-tuned FLC";
  % "sim_mpfc_basic", h_s_comms_disabled_victim_model, h_env_dynamics_60, h_a_repeat_2, h_init_fis_2, h_arch_mpfc_output_prediction, "MPFC Basic";
  % "sim_mpfc_proximity", h_s_comms_disabled_victim_model, h_env_dynamics_60, h_a_repeat_2, h_init_fis_proximity, h_arch_mpfc_output_prediction, "MPFC Proximity Input";
  % "sim_mpfc_mirko", h_s_comms_disabled_victim_model, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko, h_arch_mpfc_output_prediction, "MPFC Proximity Input";
  % "sim_mpc", h_s_comms_disabled_victim_model, h_env_dynamics_60, h_a_repeat_2_mpc, h_init_fis_2, h_arch_mpc_prediction, "Centralised MPC";
  % }; 

% SIMULATION 11: LARGE ENVIRONMENT 4 AGENTS - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "sim_mfpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% }; 

% SIMULATION X: Battery recharge
% Simulation: 
% Plots: 
% simulationSetup = { 
%   "sim_fis_battery_recharge", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_low_battery, h_init_fis_mirko_4_recharge, h_arch_fis, "FIS Battery Recharge";
%   "sim_fis_no_battery_recharge", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS No Battery Recharge";
%   % "sim_mpfc_battery_recharge", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_low_battery, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Battery Recharge";
%   % "sim_mpfc_no_battery_recharge", h_s_comms_disabled_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC No Battery Recharge";
% };  

% SIMULATION 12: Simple local prediction maps - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = {  
%   "sim_fis_global", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS Global Maps";
%   "sim_fis_local_r_5", h_s_comms_disabled_victim_model_5000_local_map_r5, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS Local Maps R5";
%   "sim_mpfc_global", h_s_comms_disabled_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Global Maps";
%   "sim_mpfc_local_r_5", h_s_comms_disabled_victim_model_5000_local_map_r5, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Local Maps R5";
% };     

% SIMULATION 13: COMPLEX LARGE DISASTER ENVIRONMENT, TWO POPULATION CENTRES - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = {
%   "sim_fis", h_s_comms_disabled_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_4, h_init_fis_2, h_arch_fis, "Pre-tuned FLC";
%   "sim_mfpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_4, h_init_fis_2, h_arch_mpfc_output_prediction, "MPFC";
%   "sim_mpc", h_s_comms_disabled_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_4_mpc, h_init_fis_2, h_arch_mpc_prediction, "Centralised MPC";
% }; 

% % SIMULATION 14: COMPLEX LARGE DISASTER ENVIRONMENT, TWO POPULATION CENTRES, LOCAL
% % PREDICTION MAPS - DONE - V4
% Simulation: DONE
% Plots: 
% simulationSetup = {  
%   "sim_mpfc_local_global", h_s_comms_disabled_victim_model_5000 , h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Global Maps";
%   "sim_mpfc_local_r_7", h_s_comms_disabled_victim_model_5000_local_map_r7 , h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Local Maps R7";
%   "sim_mpfc_local_r_3", h_s_comms_disabled_victim_model_5000_local_map_r3 , h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC Local Maps R3";
% };    

% SIMULATION 7: LOCAL MAPS DETAILED DEMO
% simulationSetup = { 
%   "sim_mpfc_r_5", h_s_comms_disabled_victim_model_20000_r_5, h_env_dynamics_100_dualCentre, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC r_local 5";
%   "sim_mpfc_r_10", h_s_comms_disabled_victim_model_20000_r_10, h_env_dynamics_100_dualCentre, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC r_local 10";
%   "sim_mpfc_r_15", h_s_comms_disabled_victim_model_20000_r_15, h_env_dynamics_100_dualCentre, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "MPFC r_local 15";
% };  

% Type 1 vs Type 2 Sugeno FLC
% Simulation: DONE
% Plots: 
% simulationSetup = {  
%   "sim_mpfc_type2", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC Sugeno Type 2";
%   "sim_mpfc_type1", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "MPFC Sugeno Type 1";
%   "sim_fis_type1", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "FIS Sugeno Type 1";
% };    

% k_MPC
% Simulation: DONE
% Plots: DONE
% simulationSetup = {
  % "sim_mpfc_k_mpc_2", h_sim_comms_disabled_victim_model_mpc_2_pred_17, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_MPC = 2";
  % "sim_mpfc_k_mpc_5", h_sim_comms_disabled_victim_model_mpc_5_pred_20, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_MPC = 5";
  % "sim_mpfc_k_mpc_15", h_sim_comms_disabled_victim_model_mpc_15_pred_30, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_MPC = 15";
  % "sim_mpfc_k_mpc_30", h_s_comms_disabled_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_MPC = 30";
  % "sim_mpfc_k_mpc_45", h_sim_comms_disabled_victim_model_mpc_45_pred_60, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_MPC = 45";
  % "sim_mpfc_k_mpc_60", h_sim_comms_disabled_victim_model_mpc_60_pred_75, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_MPC = 60";  
% };    

% k_pred
% Simulation: DONE
% Plots: DONE
% simulationSetup = {  
%   "sim_mpfc_k_pred_30", i_sim_comms_disabled_victim_model_mpc_30_pred_30, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_pred = 30";
%   "sim_mpfc_k_pred_60", i_sim_commbs_disabled_victim_model_mpc_30_pred_60, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_pred = 60";
%   "sim_mpfc_k_pred_75", i_sim_comms_disabled_victim_model_mpc_30_pred_75, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction_decentralised, "MPFC k_pred = 75";  
% };    

%% 3. Define the number of iterations for each simulation setup
numIterations = 5;   
 
% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);

% Initialize a structure to store results from all setups 
data = struct();
 
%% 4. Iterate over each simulation setup
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

%% 5. Statistical analysis

% Confidence interval level
alpha = 0.05;

% Calculate stats for obj_hist
[means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj] = calculateStats(data, simulationSetup, 'obj_hist', config.dt_s, alpha);

% % Calculate stats for optimizationTimes
[means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt] = calculateStats(data, simulationSetup, 'optimizationTimes', config.dt_s, alpha);

row_sums = cellfun(@sum, means_obj);

%% 6. Plotting

% Plot stats for obj_hist
plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetup, 'Mean Objective Value', 'Objective Value');
plotStats(means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt, simulationSetup, 'Mean Optimisation Time', 'Optimisation Time');

%% 7. Plot Geographical

% Parameters List
env_parameter_list = {'m_s', 'm_bo'};
env_labels = {'M^{\mathrm{structure}}', 'M^{\mathrm{building}}'};
search_parameter_list = {'m_bo_s', 'm_victim_s'};
search_labels = {'M^{\mathrm{search}_{\mathrm{building}}}', 'M^{\mathrm{search}_{\mathrm{victim}}}'};

% Items and their locations
items = {'UAV'}; 
item_locations = {agent_model.a_loc};
markerSizes = [10]; % List of marker sizes for items

m_f_series_indexes = [1, 21, 41, 61, 81];

plotGeographical(agent_model, environment_model, env_parameter_list, env_labels, search_parameter_list, search_labels, items, item_locations, markerSizes, m_f_series_indexes)

%% 8. Save and export simulation results

% Call the function to save results and figures
saveSimulationResults(config.flag_save, config);


% mean_values = cellfun(@mean, means_obj);

% TEMP plotting functions
% %% Replot using only certain lines
% simIndex = [2, 3];
% plotStats(means_obj(simIndex), ci_lower_obj(simIndex), ci_upper_obj(simIndex), time_vector_obj(simIndex), simulationSetup(simIndex, :), 'Mean Objective Value', 'Objective Value');
% plotStats(means_t_opt(simIndex), ci_lower_t_opt(simIndex), ci_upper_t_opt(simIndex), time_vector_t_opt(simIndex), simulationSetup(simIndex, :), 'Mean Optimisation Time', 'Optimisation Time');

% simulationSetup{1, 7} = "Pre-tuned FIS";
% simulationSetup{2, 7} = "Centralised MPFC";
% simulationSetup{3, 7} = "Decentralised MPFC";
% simulationSetup{4, 7} = "Centralised MPC";

% simulationSetup{1, 7} = "Centralised MPFC TSK Type-2";
% simulationSetup{2, 7} = "Centralised MPFC TSK Type-1";
% simulationSetup{3, 7} = "Pre-tuned FIS TSK Type-1";

% simulationSetup{1, 7} = "MPFC Global Map";
% simulationSetup{2, 7} = "MPFC Local Map ($r^{\mathrm{local}} = 7$)";
% simulationSetup{3, 7} = "MPFC Local Map ($r^{\mathrm{local}} = 5$)";
% simulationSetup{4, 7} = "MPFC Local Map ($r^{\mathrm{local}} = 3$)";
% simulationSetup{5, 7} = "Pre-tuned FIS Local Map)";

% TODO: can trim out the outliers from time plot - simulations which took longer
% e.g. due to pause of laptop

