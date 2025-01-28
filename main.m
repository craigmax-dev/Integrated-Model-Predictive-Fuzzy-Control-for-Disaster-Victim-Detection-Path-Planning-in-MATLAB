% ========================================================================
% Main Script for Simulation of UAV-based Search and Rescue Operations
% ========================================================================
% This script simulates various configurations of UAVs in search and rescue 
% (SAR) operations using different control architectures, environment 
% models, and agent behaviors. The aim is to evaluate their performance 
% across a range of scenarios by analyzing the objective function, 
% optimization times, and other metrics. This guide explains each section 
% of the script and how to customize it for specific simulation setups.

% =========================================================================
% USER GUIDE
% =========================================================================
% 1. Select a Scenario: Uncomment the relevant `simulationSetup` and 
%    `lineStyles` blocks that match the scenario you want to run, or define 
%     a new one. Update `seeds` to control random variability.
%
% 2. Set the Number of Iterations: Modify `numIterations` to specify 
%    how many times each scenario should be repeated. Increasing the number 
%    of iterations increases the statistical certainty of controller 
%    performance. 
%
% 3. Run the Simulation: Run the script after making your selections.
%
% 4. Check the Results: View the output plots and animations to evaluate 
%    the performance of each scenario.
%
% 5. Save Data and Plots: Ensure `config.flag_save` is `true` to save 
%    results.

% =========================================================================
% USER GUIDE
% =========================================================================
% Note that a full change log is available in the Git repository.
% Some slight changes to the MPC algorithm have been made, meaning that
% simulation results will not exactly match 

% ========================================================================
% BACKLOG
% ========================================================================
% - Feature: Complete VALIDATION CASE
% - Feature: Add battery recharge model to FIS
% - Feature: MPFC/MPC prediction with clustering
% - Feature: Implement slow dynamic environment variable
% - Feature: Deterministic threshold prediction mode
% - Feature: Scheduled cells methodology can be removed with new FIS inputs
% formulation
% - Feature: Remove downwind map - defunct if not used as a FIS input
% - Feature: Remove options_firstEval - just use same options for all MPC 
% evals
% - Feature: Random initialisation of fire locations
% - Bugfix: set m_f = ones instead of m_s in initialisation files
% - Feature: Setup fire model constants for realistic simulation

% =========================================================================
% AUTHOR AND CONTACT INFORMATION
% =========================================================================
% This script was developed as part of a thesis research project on the use 
% of UAVs for search and rescue operations in disaster environments.
% 
% Faculty: Control and Simulation, Aerospace Enigneering, Delft University 
% of Technology 
% 
% For any questions or collaboration inquiries, please contact:
% Craig Maxwell - C.M.Maxwell@student.tudelft.nl / cmax.inbox@gmail.com
% =========================================================================

% Clear workspace 
clear all  
close all
 
%% Set up folder paths
addpath('matlab', ...
  'matlab/functions', ...
  'matlab/models', ...
  'matlab/initialisation/simulation', ...
  'matlab/initialisation/pathPlanning', ... 
  'matlab/initialisation/mpc', ...
  'matlab/initialisation/environment', ...
  'matlab/initialisation/agent');
  
%% 1. Define function handles 
%    ------------------------
%    This section defines function handles that specify the simulation 
%    variables, environment conditions, agent behavior, FIS parameters, and 
%    control architectures.
%    
%    - Simulation: Different victim models, communication settings, and 
%      MPC parameters are available.
%    - Environment: Static and dynamic environment configurations are 
%      provided.
%    - Agents: Defines the different agent behaviors, such as the number 
%      of agents and repeated missions.
%    - FIS: Initial fuzzy inference system setups used for UAV path 
%      planning.
%    - Controller Architectures: Includes different supervisory control 
%      strategies like FLC (Fuzzy Logic Control), MPFC (Model Predictive 
%      Fuzzy Control), and MPC (Model Predictive Control).

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
h_env_static_20 = @(dt_e)i_env_static_20(dt_e);
h_env_static_40 = @(dt_e)i_env_static_40(dt_e);
h_env_dynamics_20 = @(dt_e)i_env_dynamics_20(dt_e);
h_env_dynamics_30 = @(dt_e)i_env_dynamics_30(dt_e);
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
numIterations = 5;   

% Generate and store seeds for all iterations
seeds = randi(10000, numIterations, 1);
 
% Initialize a structure to store results from all setups 
data = struct();

%% 3. Simulation setup 
%    ----------------
%    This section specifies the different scenarios (or setups) to simulate.
%    To run a simulation from the thesis, uncomment the block of code 
%     corresponding to the desired simulation and run the script.
%    - `simulationSetup`: List of setups to simulate, each setup defined by 
%      the combination of victim model, environment, agent, FIS, and 
%      control architecture.
%    - `lineStyles`: Specifies line styles for plotting the results of each 
%      controller type. Solid or dashed lines indicate centralized or 
%      decentralized strategies, respectively.

% Define line colours for each controller type
flc_colour = '#1b9e77'; % Blue for FLC
mpfc_colour = '#d95f02'; % Orange for MPFC
mpc_colour = '#7570b3'; % Yellow for MPC

% Define line styles
solid = '-';
dash = '--';

% % VALIDATION CASE
simulationSetup = { 
  "flc", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
  "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
};
lineStyles = {
    {solid, flc_colour}, ... 
    {solid, mpfc_colour}, ... 
};
seeds = [1, 2, 3, 4, 5];

% % 4.2.1 - Two-Agent System in Small Static Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_static_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_static_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {solid, mpc_colour},  ... 
% };
% seeds = [6586, 9364, 1009, 3473, 9463];
% 
% % 4.2.2 - Two-Agent System in Small Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {solid, mpc_colour},  ... 
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% 
% % 4.2.3 - Four-Agent System in Small Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
% %   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
% %     {dash, mpfc_colour}, ... 
%     {solid, mpc_colour}  ... 
% };
% seeds = [1755, 8611, 6476, 3092, 5726];
% 
% 4.2.4 - Decentralised vs Centralised MPFC Controller Architectures: Two-agent system
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}  ... 
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% 
% % 4.2.4 - Decentralised vs Centralised MPFC Controller Architectures: Four-agent system
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}  ... 
% };
% seeds = [1755, 8611, 6476, 3092, 5726];
% 
% % 4.2.5 - Two-Agent System in Complex Dynamic Disaster Environment
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_60_complex, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_60_complex, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_60_complex, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {solid, mpc_colour}  ... 
% };
% seeds = [803, 6063, 5333, 9967, 9982];
% 
% % 4.3.1 - Sensitivity Analysis: Number of Agents
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}, ... 
%     {solid, mpc_colour},  ... 
%     {dash, mpc_colour}
% };

% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
%   "mpc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction_decentralised, "Decentralised MPC";
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
%   "mpc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_3_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction_decentralised, "Decentralised MPC";
% };
% seeds = [3866, 348, 8024, 8344, 1252];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4, h_init_fis_mirko_4, h_arch_mpfc_output_prediction_decentralised, "Decentralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
%   "mpc_decentralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_4_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction_decentralised, "Decentralised MPC";
% };
% seeds = [1755, 8611, 6476, 3092, 5726];
% 
% % 4.3.2 - Sensitivity Analysis: Disaster Environment Size
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {solid, mpc_colour}
% };
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_20, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [8721, 7857, 1151, 9093, 6561];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_30, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [4767, 2357, 6936, 3167, 6246];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [265, 5052, 9173, 1171, 7530];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_50, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_50, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_50, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [8721, 7857, 1151, 9093, 6561];
% 
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpc_centralised", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2_mpc, h_init_fis_mirko_4, h_arch_mpc_prediction, "Centralised MPC";
% };
% seeds = [8721, 7857, 1151, 9093, 6561];
% 
% % 4.3.3 - Sensitivity Analysis: MPC Step Size
% simulationSetup = { 
%   "mpfc_centralised_mpc_2", h_s_victim_model_mpc_2_pred_17, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 2";
%   "mpfc_centralised_mpc_5", h_s_victim_model_mpc_5_pred_20, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 5";
%   "mpfc_centralised_mpc_15", h_s_victim_model_mpc_15_pred_30, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 30";
%   "mpfc_centralised_mpc_30", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 45";
%   "mpfc_centralised_mpc_45", h_s_victim_model_mpc_45_pred_60, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 60";
%   "mpfc_centralised_mpc_60", h_s_victim_model_mpc_60_pred_75, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_MPC = 75";
% };
% seeds = [9933, 4258, 9696, 6016, 7584];
% 
% % 4.3.4 - Sensitivity Analysis: Prediction Step Size
% simulationSetup = { 
  % "mpfc_centralised_pred_17", h_s_victim_model_mpc_2_pred_17, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 17";
  % "mpfc_centralised_pred_20", h_s_victim_model_mpc_5_pred_20, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 30";
  % "mpfc_centralised_pred_30", h_s_victim_model_mpc_15_pred_30, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 45";
  % "mpfc_centralised_pred_45", h_s_victim_model_5000, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 60";
  % "mpfc_centralised_pred_60", h_s_victim_model_mpc_45_pred_60, h_env_dynamics_60, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, k_pred = 75";
% };
% seeds = [2239, 7961, 6896, 8912, 833];
% 
% 4.4.1 - Design Exploration: Prediction Modes
% % dk_30
% simulationSetup = { 
%   "flc", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised_prediction", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, Probability Threshold";
%   "mpfc_centralised_exact", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_exact, "Centralised MPFC, Exact";
% };
% dk_60
% simulationSetup = { 
%   "flc", h_s_victim_model_mpc_60_pred_75, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "mpfc_centralised_prediction", h_s_victim_model_mpc_60_pred_75, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, Probability Threshold";
%   "mpfc_centralised_exact", h_s_victim_model_mpc_60_pred_75, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_exact, "Centralised MPFC, Exact";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}
% };
% seeds = [8904, 6149, 5712, 3194, 6791];
% 
% 4.4.2 - Design Exploration: Type-1 vs Type-2 FLC
% simulationSetup = { 
%   "mpfc_centralised_type2", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_fis, "Pre-tuned FLC, Type 2 FLC";
%   "mpfc_centralised_type1", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC, Type 1 FLC";
% };
% lineStyles = {
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}
% };
% seeds = [9359, 4746, 839, 2634, 8288];

% 4.4.2 - Design Exploration: Type-1 vs Type-2 MPFC
% simulationSetup = { 
%   "mpfc_centralised_type1", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC, Type 1 FLC";
%   "mpfc_centralised_type2", h_s_victim_model_5000, h_env_dynamics_40, h_a_repeat_2, h_init_fis_mirko_4_type2, h_arch_mpfc_output_prediction, "Centralised MPFC, Type 2 FLC";
% };
% lineStyles = {
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}
% };
% seeds = [9359, 4746, 839, 2634, 8288];
% 
% % 4.4.3 - Design Exploration: Local Prediction Maps - Small Static Disaster Environment
% simulationSetup = { 
%   "fis_global", h_s_victim_model_5000, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
%   "fis_local_r_5", h_s_victim_model_5000_local_map_r5, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Pre-tuned FLC Local Map R = 5";
%   "mpfc_global", h_s_victim_model_5000, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_local_r_5", h_s_victim_model_5000_local_map_r5, h_env_static_20, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 5";
% };
% lineStyles = {
%     {solid, flc_colour}, ... 
%     {dash, flc_colour}, ...
%     {solid, mpfc_colour}, ... 
%     {dash, mpfc_colour}, ...
% };
% seeds = [3717, 2940, 4349];
% numIterations = 3;   
% 
% 4.4.3 - Design Exploration: Local Prediction Maps - Large Dynamic Disaster Environment
% simulationSetup = { 
%   "mpfc_centralised", h_s_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC";
%   "mpfc_centralised_local_r7", h_s_victim_model_5000_local_map_r7, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 7";
%   "mpfc_centralised_local_r5", h_s_victim_model_5000_local_map_r5, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 5";
%   "mpfc_centralised_local_r3", h_s_victim_model_5000_local_map_r3, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_mpfc_output_prediction, "Centralised MPFC Local Map R = 3";
%   "flc", h_s_victim_model_5000, h_env_dynamics_200_dualCentre, h_a_repeat_2, h_init_fis_mirko_4, h_arch_fis, "Pre-tuned FLC";
% };
% lineStyles = {
%     {solid, '#e66101'}, ... 
%     {solid, '#fdb863'}, ... 
%     {solid, '#b2abd2'}, ... 
%     {solid, '#5e3c99'}, ... 
%     {solid, flc_colour}, ... 
% };
% seeds = [8675, 2155, 278];
% numIterations = 3;   

%% 4. Iterate over each simulation setup
%    ----------------
%    Iterates over each setup defined in `simulationSetup` and runs the 
%    simulation for the given number of iterations.
%    Each iteration is run sequentially updating model parameters at each
%    timestup until the termination condition (finish time).
%    Calculates mean performance before continuing to the next setup.

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
    
      %% Initialise models, plotting data, timestep for saving variables
 
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
      config.trackParams = true;
      
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

      % Set seed for iteration
      rng(seeds(iteration));  
      
      % Calculate total number of steps needed for the pre-computation phase
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
[means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt] = calculateStats(data, simulationSetup, 'optimizationTimes', config.dt_mpc, alpha);

%% 6. Plot Simulation Performance Parameters
%    ---------------------------------------
%    `plotStats()` produces plots of:
%    - Mean objective values with confidence intervals.
%    - Mean optimization times with confidence intervals.

% % Define the threshold
% threshold = 0.2e4;
% 
% % Loop through each selected simulation index
% for idx = 1:length(simIndex)
%     i = simIndex(idx);
% 
%     % Get the data for means, ci_lower, and ci_upper
%     means = means_t_opt{i};
%     ci_lower = ci_lower_t_opt{i};
%     ci_upper = ci_upper_t_opt{i};
%     time_vector = time_vector_t_opt{i};
% 
%     % Find the indices of values below the threshold
%     validIndices = means <= threshold;
% 
%     % Filter the data based on the valid indices
%     means_t_opt{i} = means(validIndices);
%     ci_lower_t_opt{i} = ci_lower(validIndices);
%     ci_upper_t_opt{i} = ci_upper(validIndices);
%     time_vector_t_opt{i} = time_vector(validIndices);
% end

% Select which simulations to plot
simIndex = [1, 2, 3];

% Plot stats for obj_hist
% plotStats(means_obj, ci_lower_obj, ci_upper_obj, time_vector_obj, simulationSetup, "Objective Function, $\overline{J}$", lineStyles);
plotStats(means_obj(simIndex), ci_lower_obj(simIndex), ci_upper_obj(simIndex), time_vector_obj(simIndex), simulationSetup(simIndex, :), "Objective Function, $\overline{J}$", lineStyles(simIndex));

% Plot stats for optimizationTimes
% plotStats(means_t_opt, ci_lower_t_opt, ci_upper_t_opt, time_vector_t_opt, simulationSetup, "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles);
plotStats(means_t_opt(simIndex), ci_lower_t_opt(simIndex), ci_upper_t_opt(simIndex), time_vector_t_opt(simIndex), simulationSetup(simIndex, :), "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles(simIndex));

%% 7. Plot Geographical Parameters
%    -----------------------------
%    Plot environment and agent states:

% Parameters List
% env_parameter_list = {'m_s', 'm_bo'};
% env_labels = {'\mathbf{M}^{\mathrm{structure}}', '\mathbf{M}^{\mathrm{building}}'};
% search_parameter_list = {'m_bo_s', 'm_victim_s'};
% search_labels = {'\mathbf{M}^{\mathrm{c,building}}', '\mathbf{M}^{\mathrm{victim}}'};
env_parameter_list = {'m_s'};
env_labels = {'\mathbf{M}^{\mathrm{structure}}'};
search_parameter_list = {'m_victim_s'};
search_labels = {'\mathbf{M}^{\mathrm{victim}}'};

% Items and their locations
items = {'UAV'}; 
item_locations = {agent_model.a_loc};
markerSizes = [10]; % List of marker sizes for items

m_f_series_indexes = [1, 21, 41, 61, 81];

plotGeographical(agent_model, environment_model, env_parameter_list, env_labels, search_parameter_list, search_labels, items, item_locations, markerSizes, m_f_series_indexes)

%% 8. Plot animations
%    ----------------
%    Plots and saves in .gif format animations of:
animateMPFCSimulation(agent_model, environment_model, config, flc_params_hist, results, "animation_agent_actions.gif")

% Fire spread animation
animateFireHistory(environment_model.m_f_series, config, "animation_fire_map", "animation_fire_map.gif")

%% 9. Save and export simulation results
%    -----------------------------------
%    `saveSimulationResults()` - saves workspace parameters and figures in 
%     a new folder

% Call the function to save results and figures
saveSimulationResults(config.flag_save, config);
close all

%% 10. Sensitivity Analysis Plots
%     ---------------------------
%     Sensitivity analysis plots can be generated here using the data from 
%     the simulations run during this study. Note that the pre-filled data is
%     from simulations used in the thesis.

%% Prediction Mode
% % V2
% fisMeanObj = 1.0e+03*[2.6022, 2.4509];
% 
% probThresh_MeanObj = 1.0e+03*[2.1918, 2.1121];
% probThresh_MeanObj_confLower = 1.0e+03 *[1.9576, 1.9219];
% probThresh_MeanObj_confUpper = 1.0e+03 *[2.4259, 2.3023];
% probThresh_MeanTime = [44.7809, 108];
% probThresh_MeanTime_confLower = [42.7516, 105];
% probThresh_MeanTime_confUpper = [46.8101, 110];
% 
% exact_MeanObj = 1.0e+03*[2.2119, 2.1055];
% exact_MeanObj_confLower = 1.0e+03 *[2.0454, 1.9160];
% exact_MeanObj_confUpper = 1.0e+03 *[2.3784, 2.2950];
% exact_MeanTime = [44.1336, 119];
% exact_MeanTime_confLower = [42.5252, 107];
% exact_MeanTime_confUpper = [45.7420, 132];
% 
% predMode_MeanObj            = [probThresh_MeanObj           ; exact_MeanObj           ];
% predMode_MeanObj_confLower  = [probThresh_MeanObj_confLower ; exact_MeanObj_confLower ];
% predMode_MeanObj_confUpper  = [probThresh_MeanObj_confUpper ; exact_MeanObj_confUpper ];
% predMode_MeanTime           = [probThresh_MeanTime          ; exact_MeanTime          ];
% predMode_MeanTime_confLower = [probThresh_MeanTime_confLower; exact_MeanTime_confLower];
% predMode_MeanTime_confUpper = [probThresh_MeanTime_confUpper; exact_MeanTime_confUpper];
% 
% simNames = {'Probability Threshold', 'Exact'};
% t_mpc = 15*[30, 60];
% lineStyles = {
%     {solid, mpfc_colour}, ...         % Centralised MPFC
%     {dash, mpfc_colour}, ...          % Decentralised MPFC
% };
% 
% % Call the functions
% plotScatterTrends(predMode_MeanObj, simNames, t_mpc, 1, predMode_MeanObj_confLower, predMode_MeanObj_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)", "Objective Function, $\overline{J}$", lineStyles);
% plotScatterTrends(predMode_MeanTime, simNames, t_mpc, 1, predMode_MeanTime_confLower, predMode_MeanTime_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)", "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles);
% 
% 
% % simNames = {'Probability Threshold'};
% plotScatterTrendsNormalised([probThresh_MeanObj; exact_MeanObj], ...
%     fisMeanObj, simNames, t_mpc, 1, ...
%     [probThresh_MeanObj_confLower; exact_MeanObj_confLower], ...
%     [probThresh_MeanObj_confUpper; exact_MeanObj_confUpper], ...
%     "MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)", ...
%     "Normalised Objective Function, $\overline{J} (\Delta \%)$", lineStyles);

%% n_a %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Data
% fisMeanObj = 1.0e+03 *[2.5506, 2.2616, 2.1763];
% 
% mpfcCentralisedMeanTime = [48.2061, 70.7112, 96.8939];
% mpfcCentralisedMeanTime_confLower = [47.4827, 68.4695, 94.7810];
% mpfcCentralisedMeanTime_confUpper = [48.9295, 72.9529, 99.0068];
% mpfcCentralisedMeanObj = 1.0e+03 *[2.3214, 2.0171, 1.8487];
% mpfcCentralisedMeanObj_confLower = 1.0e+03 *[2.0998, 1.7781, 1.5523];
% mpfcCentralisedMeanObj_confUpper = 1.0e+03 *[2.5430, 2.2561, 2.1451];
% 
% mpfcDecentralisedMeanTime = [23.5540, 23.7515, 23.3633];
% mpfcDecentralisedMeanTime_confLower = [23.3618, 20.6987, 23.1581];
% mpfcDecentralisedMeanTime_confUpper = [23.7462, 26.8044, 23.5684];
% mpfcDecentralisedMeanObj = 1.0e+03 *[2.2642, 2.0330, 1.8070];
% mpfcDecentralisedMeanObj_confLower = 1.0e+03 *[1.9367, 1.8009, 1.5147];
% mpfcDecentralisedMeanObj_confUpper = 1.0e+03 *[2.5918, 2.2651, 2.0993];
% 
% mpcCentralisedMeanTime = [125.4170, 125.6490,  147.7781];
% mpcCentralisedMeanTime_confLower = [94.6109, 96.1807, 106.4945];
% mpcCentralisedMeanTime_confUpper = [156.2232, 155.1172, 189.0617];
% mpcCentralisedMeanObj = 1.0e+03 *[2.6187, 2.3124, 2.0442];
% mpcCentralisedMeanObj_confLower = 1.0e+03 *[2.3745, 2.1004, 1.8818];
% mpcCentralisedMeanObj_confUpper = 1.0e+03 *[2.8629, 2.5244, 2.2067];
% 
% mpcDecentralisedMeanTime = [124.2137, 122.6140, 129.4289];
% mpcDecentralisedMeanTime_confLower = [91.9590, 90.1793, 90.4822];
% mpcDecentralisedMeanTime_confUpper = [156.4684, 155.0486, 168.3756];
% mpcDecentralisedMeanObj = 1.0e+03 *[2.3870, 2.1690, 1.9107];
% mpcDecentralisedMeanObj_confLower = 1.0e+03 *[2.1491, 2.0529, 1.6598];
% mpcDecentralisedMeanObj_confUpper = 1.0e+03 *[2.6250, 2.2850, 2.1617];
% 
% simNames = {'Centralised MPC', 'Decentralised MPC', 'Centralised MPFC', 'Decentralised MPFC'};
% agentCounts = [2, 3, 4];
% lineStyles = {
%     {solid, mpc_colour}, ...         % Centralised MPC
%     {dash, mpc_colour}, ...          % Decentralised MPC
%     {solid, mpfc_colour}, ...        % Centralised MPFC
%     {dash, mpfc_colour}              % Decentralised MPFC
% };
% 
% % Call the functions
% plotScatterTrends([mpcCentralisedMeanTime; mpcDecentralisedMeanTime; mpfcCentralisedMeanTime; mpfcDecentralisedMeanTime], simNames, agentCounts, 1, [mpcCentralisedMeanTime_confLower; mpcDecentralisedMeanTime_confLower; mpfcCentralisedMeanTime_confLower; mpfcDecentralisedMeanTime_confLower], [mpcCentralisedMeanTime_confUpper; mpcDecentralisedMeanTime_confUpper; mpfcCentralisedMeanTime_confUpper; mpfcDecentralisedMeanTime_confUpper], "Number of Agents, $n^{a}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles);
% plotScatterTrendsNormalised([mpcCentralisedMeanObj; mpcDecentralisedMeanObj; mpfcCentralisedMeanObj; mpfcDecentralisedMeanObj], fisMeanObj, simNames, agentCounts, 1, [mpcCentralisedMeanObj_confLower; mpcDecentralisedMeanObj_confLower; mpfcCentralisedMeanObj_confLower; mpfcDecentralisedMeanObj_confLower], [mpcCentralisedMeanObj_confUpper; mpcDecentralisedMeanObj_confUpper; mpfcCentralisedMeanObj_confUpper; mpfcDecentralisedMeanObj_confUpper], "Number of Agents, $n^{a}$", "Normalised Objective Function, $\overline{J} (\Delta \%)$", lineStyles);

%% t_MPC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t_MPC_MeanObj = 1.0e+03*[7.1370, 7.0530, 7.2312, 7.0589, 7.7787, 8.0764];
% t_MPC_MeanObj_confLower = 1.0e+03 *[6.6995, 6.6821, 6.7562, 6.5413, 7.2780, 7.3175];
% t_MPC_MeanObj_confUpper = 1.0e+03 *[7.5746, 7.4238, 7.7063, 7.5765, 8.2794, 8.8353];
% t_MPC_MeanTime = [46.0835, 54.1516, 74.7733, 121.3524, 161.9631, 202.1015];
% t_MPC_MeanTime_confLower = [44.6674, 52.4588, 69.8264, 114.3734, 160.5813, 200.6613];
% t_MPC_MeanTime_confUpper = [47.4997, 55.8444, 79.7201, 128.3313, 163.3449, 203.5417];
% 
% simNames = {'Centralised MPFC'};
% t_mpc = 15*[2, 5, 15, 30, 45, 60];
% lineStyles = {
%     {solid, mpfc_colour}, ...        % Centralised MPFC
% };
% 
% % Call the functions
% plotScatterTrends(t_MPC_MeanObj, simNames, t_mpc, 2, t_MPC_MeanObj_confLower, t_MPC_MeanObj_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)", "Objective Function, $\overline{J}$", lineStyles);
% plotScatterTrends(t_MPC_MeanTime, simNames, t_mpc, 1, t_MPC_MeanTime_confLower, t_MPC_MeanTime_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)", "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles);

% % t_pred %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t_pred_MeanObj = 1.0e+03*[7.77, 7.7787, 7.805, 7.684];
% t_pred_MeanObj_confLower = 1.0e+03 *[7.3310, 7.2780, 7.3690, 7.3288];
% t_pred_MeanObj_confUpper = 1.0e+03 *[8.2091, 8.2794, 8.2409, 8.0391];
% t_pred_MeanTime = [78.2812, 161.9631, 159.9497, 202.9531];
% t_pred_MeanTime_confLower = [68.6694, 160.5813, 133.5439, 172.0526];
% t_pred_MeanTime_confUpper = [87.8929, 163.3449, 186.3556, 233.8537];
% 
% simNames = {'Centralised MPFC'};
% t_pred = 15*[30, 45, 60, 75];
% lineStyles = {
%     {solid, mpfc_colour}, ...        % Centralised MPFC
% };
% 
% % Call the functions
% plotScatterTrends(t_pred_MeanObj, simNames, t_pred, 1, t_pred_MeanObj_confLower, t_pred_MeanObj_confUpper, "Prediction Horizon, $\Delta t^{\mathrm{pred}}$", "Objective Function, $\overline{J}$", lineStyles);
% plotScatterTrends(t_pred_MeanTime, simNames, t_pred, 1, t_pred_MeanTime_confLower, t_pred_MeanTime_confUpper, "Prediction Horizon, $\Delta t^{\mathrm{pred}}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles);

%% Disaster environment size %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Data
% V2
% fisMeanObj = 1.0e+03*[0.207, 2.5986, 7.6638];
% 
% mpfcCentralisedMeanTime = [21.5653, 96.8939, 168.0375];
% mpfcCentralisedMeanTime_confLower = [20.8904, 94.7810, 165.8786];
% mpfcCentralisedMeanTime_confUpper = [22.2401, 99.0068, 170.1964];
% mpfcCentralisedMeanObj = 1.0e+03 *[0.1908, 2.2754, 6.9163];
% mpfcCentralisedMeanObj_confLower = 1.0e+03 *[0.156, 2.0890, 6.4595];
% mpfcCentralisedMeanObj_confUpper = 1.0e+03 *[0.225, 2.4618, 7.3732];
% 
% 
% mpcMeanTime = [42.9858, 129.7036, 246.8798];
% mpcMeanTime_confLower = [31.6914, 100.7676, 189.8676];
% mpcMeanTime_confUpper = [54.2802, 158.6395, 303.8921];
% mpcMeanObj = 1.0e+03 *[0.2697, 2.5545, 7.7265];
% mpcMeanObj_confLower = 1.0e+03 *[0.218, 2.3512, 7.4081];
% mpcMeanObj_confUpper = 1.0e+03 *[0.320, 2.7578, 8.0449];
% 
% simNames = {'Centralised MPC', 'Centralised MPFC'};
% envSize = [400, 1600, 3600];
% lineStyles = {
%     {solid, mpc_colour}, ...        % Centralised MPC
%     {solid, mpfc_colour}, ...        % Centralised MPFC
% };
% 
% % Call the functions
% plotScatterTrends([mpcMeanTime; mpfcCentralisedMeanTime], simNames, envSize, 1, [mpcMeanTime_confLower; mpfcCentralisedMeanTime_confLower], [mpcMeanTime_confUpper; mpfcCentralisedMeanTime_confUpper], "Number of Environment Cells, $n^{env^{x}} \cdot n^{env^{y}}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)", lineStyles);
% plotScatterTrendsNormalised([mpcMeanObj; mpfcCentralisedMeanObj], fisMeanObj, simNames, envSize, 1, [mpcMeanObj_confLower; mpfcCentralisedMeanObj_confLower], [mpcMeanObj_confUpper; mpfcCentralisedMeanObj_confUpper], "Number of Environment Cells, $n^{env^{x}} \cdot n^{env^{y}}$", "Normalised Objective Function, $\overline{J} (\Delta \%)$", lineStyles);


