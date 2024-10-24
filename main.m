 %% Main simulation script
% Author: Craig Maxwell
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University of
% Technology 

% FEATURE BACKLOG
% - Feature: Complete VALIDATION CASE
% - Feature: Add battery recharge model to FIS
% - Feature: MPFC/MPC prediction with clustering
% - Feature: Implement slow dynamic environment variable
% - Feature: Deterministic threshold prediction mode
% - Feature: Scheduled cells methodology can be removed with new FIS inputs formulation
% - Feature: Remove downwind map - defunct if not used as a FIS input
% - Feature: Remove options_firstEval - just use same options for all MPC evals
% - Feature: Random initialisation of fire locations
% - Bugfix: set m_f = ones instead of m_s in initialisation files
% - Feature: Setup fire model constants for realistic simulation

% Main script user guide
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
  'matlab/initialisation/pathPlanning', ... 
  'matlab/initialisation/mpc', ...
  'matlab/initialisation/environment', ...
  'matlab/initialisation/agent');
  
%% 1. Define function handles 

% Simulation variables
h_s_no_victim_model = @()i_sim_comms_disabled();
h_s_victim_model = @()i_sim_comms_disabled_victim_model();
h_s_victim_model_5000 = @()i_sim_comms_disabled_victim_model_5000();
h_s_victim_model_5000_local_map_r3 = @()i_sim_comms_disabled_victim_model_5000_local_map_r3();
h_s_victim_model_5000_local_map_r5 = @()i_sim_comms_disabled_victim_model_5000_local_map_r5();
h_s_victim_model_5000_local_map_r7 = @()i_sim_comms_disabled_victim_model_5000_local_map_r7();

h_s_victim_model_20000 = @()i_sim_comms_disabled_victim_model_20000();

h_s_victim_model_mpc_2_pred_17= @()i_sim_comms_disabled_victim_model_mpc_2_pred_17();
h_s_victim_model_mpc_5_pred_20= @()i_sim_comms_disabled_victim_model_mpc_5_pred_20();
h_s_victim_model_mpc_15_pred_30= @()i_sim_comms_disabled_victim_model_mpc_15_pred_30();
h_s_victim_model_mpc_30_pred_30= @()i_sim_comms_disabled_victim_model_mpc_30_pred_30();
h_s_victim_model_mpc_30_pred_60= @()i_sim_comms_disabled_victim_model_mpc_30_pred_60();
h_s_victim_model_mpc_30_pred_75= @()i_sim_comms_disabled_victim_model_mpc_30_pred_75();
h_s_victim_model_mpc_45_pred_60= @()i_sim_comms_disabled_victim_model_mpc_45_pred_60();
h_s_victim_model_mpc_60_pred_75= @()i_sim_comms_disabled_victim_model_mpc_60_pred_75();

% Environment
h_env_static_40 = @(dt_e)i_env_static_40(dt_e);
h_env_dynamics_20 = @(dt_e)i_env_dynamics_20(dt_e);
h_env_dynamics_40 = @(dt_e)i_env_dynamics_40(dt_e);
h_env_dynamics_60 = @(dt_e)i_env_dynamics_60(dt_e);
h_env_dynamics_200_dualCentre = @(dt_e)i_env_dynamics_200_dualCentre(dt_e);
h_env_dynamics_60_complex = @(dt_e)i_env_dynamics_60_complex(dt_e);

% Agent
h_a_repeat_2 = @(environment_model, config)i_a_repeat_2(environment_model, config);
h_a_repeat_2_mpc = @(environment_model, config)i_a_repeat_2_mpc(environment_model, config);
h_a_repeat_3 = @(environment_model, config)i_a_repeat_3(environment_model, config);
h_a_repeat_3_mpc = @(environment_model, config)i_a_repeat_3_mpc(environment_model, config);
h_a_repeat_4 = @(environment_model, config)i_a_repeat_4(environment_model, config);
h_a_repeat_4_mpc = @(environment_model, config)i_a_repeat_4_mpc(environment_model, config);

% FIS
h_init_fis_2 = @(n_a)initialise_fis_t_response_priority(n_a);
h_init_fis_proximity = @(n_a)initialise_fis_t_response_priority_r_nextagent(n_a);
h_init_fis_mirko_4 = @(n_a)initialise_fis_mirko_4(n_a);
h_init_fis_mirko_4_type2 = @(n_a)initialise_fis_mirko_4_type2(n_a);
   
% Controller Architecture
h_arch_fis = @(fisArray, agent_model)i_arch_fis(fisArray, agent_model);

h_arch_mpfc_output_exact = @(fisArray, agent_model)i_arch_mpfc_output_exact(fisArray, agent_model);
h_arch_mpfc_output_exact_decentralised = @(fisArray, agent_model)i_arch_mpfc_output_exact_decentralised(fisArray, agent_model);
h_arch_mpfc_output_prediction = @(fisArray, agent_model)i_arch_mpfc_output_prediction(fisArray, agent_model);
h_arch_mpfc_output_prediction_decentralised = @(fisArray, agent_model)i_arch_mpfc_output_prediction_decentralised(fisArray, agent_model);

h_arch_mpc_exact = @(fisArray, agent_model)i_arch_mpc_exact(fisArray, agent_model);
h_arch_mpc_prediction = @(fisArray, agent_model)i_arch_mpc_prediction(fisArray, agent_model);
h_arch_mpc_prediction_decentralised = @(fisArray, agent_model)i_arch_mpc_prediction_decentralised(fisArray, agent_model);

%% 2. Define the number of iterations for each simulation setup
numIterations = 1;   

% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);
 
% Initialize a structure to store results from all setups 
data = struct();

%% 3. Simulation setup 
% The simulations run for my thesis are listed in the comments below.
% To run a previous simulation, uncomment the simulationSetup parameter and 
% copy the simulationSeeds parameter into the section 3 of the code.

% Define line colors for each controller type
flc_color = "blue"; % Blue for FLC # #1b9e77
mpfc_color = "red"; % Red for MPFC # #d95f02
mpc_color = "green"; % Green for MPC # #7570b3

% Define line styles
solid = '-';
dash = '--';

% % VALIDATION CASE
simulationSetup = { 
  "flc", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
  "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
};
lineStyles = {
    {solid, 'Color', flc_color}, ... 
    {solid, 'Color', mpfc_color}, ... 
};
seeds = [1, 2, 3, 4, 5];
simIndex = [2, 3];
% 
% % 4.2.1 - Two-Agent System in Small Static Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {solid, 'Color', mpc_color}  ... 
% };
% seeds = [6586, 9364, 1009, 3473, 9463];
% simIndex = [2, 3];
% 
% % 4.2.2 - Two-Agent System in Small Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {solid, 'Color', mpc_color}  ... 
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% simIndex = [2, 3];
% 
% % 4.2.3 - Four-Agent System in Small Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {solid, 'Color', mpc_color}  ... 
% };
% seeds = [1755, 8611, 6476, 3092, 5726];
% simIndex = [2, 3];
% 
% % 4.2.4 - Decentralised vs Centralised MPFC Controller Architectures: Two-agent system
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
% };
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {dash, 'Color', mpfc_color}  ... 
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% simIndex = [2, 3];
% 
% % 4.2.4 - Decentralised vs Centralised MPFC Controller Architectures: Four-agent system
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
% };
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {dash, 'Color', mpfc_color}  ... 
% };
% seeds = [1755, 8611, 6476, 3092, 5726];
% simIndex = [2, 3];
% 
% % 4.2.5 - Two-Agent System in Complex Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_60_complex, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_60_complex, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_60_complex, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {solid, 'Color', mpc_color}  ... 
% };
% seeds = [803, 6063, 5333, 9967, 9982];
% simIndex = [2, 3];
% 
% % 4.3.1 - Sensitivity Analysis: Number of Agents
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {dash, 'Color', mpfc_color}, ... 
%     {solid, 'Color', mpc_color},  ... 
%     {dash, 'Color', mpc_color}
% };

% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
%   "mpc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction_decentralised, "Decentralised MPC";
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% simIndex = [2, 3, 4, 5];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
%   "mpc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpc_prediction_decentralised, "Decentralised MPC";
% };
% seeds = [3866, 348, 8024, 8344, 1252];
% simIndex = [2, 3, 4, 5];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
%   "mpc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpc_prediction_decentralised, "Decentralised MPC";
% };
% seeds = [1755, 8611, 6476, 3092, 5726];
% simIndex = [2, 3, 4, 5];
% 
% % 4.3.2 - Sensitivity Analysis: Disaster Environment Size
% lineStyles = {
%     {solid, 'Color', flc_color}, ... 
%     {solid, 'Color', mpfc_color}, ... 
%     {solid, 'Color', mpc_color}
% };

% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [4767, 2357, 6936, 3167, 6246];
% simIndex = [2, 3];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% simIndex = [2, 3];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [8721, 7857, 1151, 9093, 6561];
% simIndex = [2, 3];
% 
% % 4.3.3 - Sensitivity Analysis: MPC Step Size
% simulationSetup = { 
%   "mpfc_centralised_mpc_2", h_s_victim_model_mpc_2_pred_17, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 2";
%   "mpfc_centralised_mpc_5", h_s_victim_model_mpc_5_pred_20, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 5";
%   "mpfc_centralised_mpc_15", h_s_victim_model_mpc_15_pred_30, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 30";
%   "mpfc_centralised_mpc_30", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 45";
%   "mpfc_centralised_mpc_45", h_s_victim_model_mpc_45_pred_60, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 60";
%   "mpfc_centralised_mpc_60", h_s_victim_model_mpc_60_pred_75, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 75";
% };
% seeds = [9933, 4258, 9696, 6016, 7584];
% simIndex = [1, 2, 3, 4, 5, 6];
% 
% % 4.3.4 - Sensitivity Analysis: Prediction Step Size
% simulationSetup = { 
%   "mpfc_centralised_pred_17", h_s_victim_model_mpc_2_pred_17, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 17";
%   "mpfc_centralised_pred_30", h_s_victim_model_mpc_5_pred_20, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 30";
%   "mpfc_centralised_pred_45", h_s_victim_model_mpc_15_pred_30, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 45";
%   "mpfc_centralised_pred_60", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 60";
%   "mpfc_centralised_pred_75", h_s_victim_model_mpc_45_pred_60, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 75";
% };
% seeds = [2239, 7961, 6896, 8912, 833];
% simIndex = [1, 2, 3, 4, 5];
% 
% % 4.4.1 - Design Exploration: Prediction Modes
% simulationSetup = { 
%   "mpfc_centralised_exact", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_exact, "Centralised MPFC, Exact";
%   "mpfc_centralised_prediction", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, Prediction";
% };
% lineStyles = {
%     {solid, 'Color', mpfc_color}, ... 
%     {dot, 'Color', mpfc_color}
% };
% seeds = [8904, 6149, 5712, 3194, 6791];
% simIndex = [1, 2];
% 
% % 4.4.2 - Design Exploration: Type-1 vs Type-2 FLC
% simulationSetup = { 
%   "mpfc_centralised_type1", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, Type 1 FLC";
%   "mpfc_centralised_type2", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction, "Centralised MPFC, Type 2 FLC";
% };
% lineStyles = {
%     {solid, 'Color', mpfc_color}, ... 
%     {dot, 'Color', mpfc_color}
% };
% seeds = [9359, 4746, 839, 2634, 8288];
% simIndex = [1, 2];
% 
% % 4.4.3 - Design Exploration: Local Prediction Maps - Small Static Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_centralised_local_r5", h_s_victim_model_5000_local_map_r5, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 5";
% };
% seeds = [3717, 2940, 4349];
% numIterations = 3;   
% simIndex = [2, 3];
% 
% % 4.4.3 - Design Exploration: Local Prediction Maps - Large Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_centralised_local_r3", h_s_victim_model_5000_local_map_r3, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 3";
%   "mpfc_centralised_local_r5", h_s_victim_model_5000_local_map_r5, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 5";
%   "mpfc_centralised_local_r7", h_s_victim_model_5000_local_map_r7, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 7";
% };
% seeds = [8675, 2155, 278];
% numIterations = 3;   
% simIndex = [2, 3, 4, 5];
 
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
  results = struct('t_hist', cell(1, numIterations), 's_obj_hist', cell(1, numIterations), 'obj_hist', cell(1, numIterations));
   
  for iteration = 1:numIterations

      fprintf("Iteration: %i \n", iteration)
    
      %% Initialise models, plotting data, and timestep for saving variables
 
      % % Generate and record a unique seed for each iteration
      rng(seeds(iteration))

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

      %% Initialise parameters for animation (if necessary)
      config.trackParams = true; % Uncomment to plot animations of simulation parameters
      
      % Calculate the length for initialization
      num_timesteps = floor(config.t_f / (config.dt_s * config.dk_mpc));

      % Initialize flc_params_hist as a cell array to hold all timesteps
      flc_params_hist = cell(agent_model.n_a, num_timesteps);      

      agent_model.a_task_hist = agent_model.a_task;
      agent_model.a_loc_hist  = agent_model.a_loc;       

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

          % Save FLC parameters for animation
          if config.trackParams
            % Loop through each agent to save the FLC parameters for each output
            for agent_idx = 1:agent_model.n_a
              % Initialize a cell array to hold the membership function parameters for all outputs
              mf_params = cell(1, numel(fisArray(agent_idx).Outputs));
      
              % Loop through each output to collect MF parameters
              for i = 1:numel(fisArray(agent_idx).Outputs)
                output = fisArray(agent_idx).Outputs(i);
                num_mfs = numel(output.MembershipFunctions);
                mf_params{i} = cell(1, num_mfs);
    
                % Collect parameters for each membership function in the output
                for j = 1:num_mfs
                  mf_params{i}{j} = output.MembershipFunctions(j).Parameters;
                end
              end
      
              % Save the collected parameters to the history for the current agent and time step
              flc_params_hist{agent_idx, config.k_mpc} = mf_params;
            end
          end

        end  
 
        %% FIS Path Planning Model 
        if (strcmp(mpc_model.architecture, 'fis') || strcmp(mpc_model.architecture, 'mpfc')) && (config.k_c * config.dk_c <= config.k)
          if isfield(config, 'flag_local_maps') && config.flag_local_maps
            [agent_model] = model_fis_local(agent_model, environment_model.ang_w, environment_model.v_w, config, fisArray, environment_model.m_f_series(:, :, config.k_e + 1));
          else
            [agent_model] = model_fis_global(agent_model, environment_model.ang_w, environment_model.v_w, config, fisArray, environment_model.m_f_series(:, :, config.k_e + 1));
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
        [config.s_obj, config.obj] = calc_obj_v4(...
          config.weight, environment_model.m_dw_e_series(:, :, config.k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
          config.dt_s, config.s_obj, config.c_f_s);
        
        % Advance timestep
        config.t = config.t + config.dt_s;
        config.k = config.k + 1;

        %% Store variables 
        t_hist(config.k) = config.k * config.dt_s;  % Store current time
        s_obj_hist(config.k) = config.s_obj;        % Store specific objective value
        obj_hist(config.k) = config.obj;            % Store objective value
        
        % Store agent location and task history for timestep k if tracking is enabled
        if config.trackParams
            agent_model.a_loc_hist(:, :, config.k) = agent_model.a_loc;  % Store agent locations
            agent_model.a_task_hist(:, config.k) = agent_model.a_task;           % Store agent tasks
            % Other parameters to store for full FLC animation: 
            % agent_model.m_fire_risk_hist,
            % agent_model.m_response_hist, 
            % agent_model.m_scan_certainty_hist,
            % agent_model.m_priority_hist, agent_model.m_att_hist
        end
        
        %% Progress report
        if config.k_prog * config.dk_prog <= config.k
          report_progress(config.endCondition, config.t, config.t_f, agent_model.m_scan, agent_model.n_x_s, agent_model.n_y_s);
          config.k_prog = config.k_prog + 1;
        end
 
        %% Check end condition
        [config.flag_finish] = func_endCondition(config.endCondition, config.t, config.t_f, agent_model.m_scan, agent_model.n_x_s, agent_model.n_y_s);

      end

      % Correctly store results for each iteration
      results(iteration).t_hist = t_hist;        
      results(iteration).s_obj_hist = s_obj_hist; 
      results(iteration).obj_hist = obj_hist;    
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

% Calculate stats for optimizationTimes
[means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt] = calculateStats(data, simulationSetup, 'optimizationTimes', config.dt_s, alpha);

%% 6. Plot Simulation Performance Parameters

% Plot stats for obj_hist
plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetup, 'Mean Objective Value', 'Objective Value', lineStyles);

% Plot stats for optimizationTimes
plotStats(means_t_opt(simIndex), ci_lower_t_opt(simIndex), ci_upper_t_opt(simIndex), time_vector_t_opt(simIndex), simulationSetup(simIndex, :), 'Mean Optimisation Time', 'Optimisation Time', lineStyles);

%% 7. Plot Geographical Parameters

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

%% 8. Plot animations
animateFLCParams(flc_params_hist, fisArray, "animation_mpfc_flc_params_hist")
animateAgentFireEnvironment(agent_model, environment_model, config, "animation_agent_actions")

%% 9. Save and export simulation results

% Call the function to save results and figures
saveSimulationResults(config.flag_save, config);

