%% Function initialise_MPC
% Initialise MPC model

% V2
% CHANGELOG
% Refactor: mpc_model structure
% Feature: mpc and mpfc architectures

function mpc_model = i_arch_mpfc(fisArray, agent_model)

  % Prediction horizon
  n_p = 1;                  
  
  % Prediction model and tuning parameters setup
  architecture = 'mpfc'; % Options: mpc, mpfc, fis
  mpc_tuning_parameters = 'input'; % Options: input, output
  solver = 'patternsearch'; % Options: fmincon, ga, particleswarm, patternsearch

  % Prediction mode
  % 'deterministic_exact' - In the “Deterministic Exact ” mode, the exact environment parameters that occur in the simulation are passed to the MPC.
  % 'deterministic_prediction' - In the “Deterministic Prediction” mode, the environment parameters at the current timestep are passed to the MPC, which then uses them to predict the future evolution of the environment parameters over the prediction horizon.
  % 'deterministic_threshold' - TBC - In the “Deterministic Threshold” mode, a predefined threshold and the environment parameters at the current timestep are passed to the MPC. The MPC then performs a prediction using the threshold to determine whether any probabilistic parameters are propagated.
  % 'probabilistic' - TBC - In this proposed mode, the MPC predicts the probability of the propagation of all environment variables at each timestep. The difficulty is that many functions rely on a deterministic propagation of the probabilistic parameters.
  prediction_model = 'deterministic_exact';

  % Initialise optimisation parameters and constraints based on architecture
  if strcmp(architecture, 'mpc')
      % 'mpc' architecture settings
      ini_params = repmat([1, 1], 1, agent_model.n_a * n_p); % Initial guess within valid range
      lb = ones(1, agent_model.n_a * 2 * n_p); % Lower bound of 1 for all parameters
      ub = reshape([repmat(agent_model.n_x_s, 1, agent_model.n_a * n_p); repmat(agent_model.n_y_s, 1, agent_model.n_a * n_p)], 1, []); % Interleave x and y bounds
      A = [];
      b = [];
  elseif strcmp(architecture, 'mpfc')
      % 'mpfc' architecture settings: Reintegrate original FIS parameter constraints
      fis_params = [];
      for a = 1:agent_model.n_a
          fis_params = [fis_params, fisArray(a).Outputs.MembershipFunctions.Parameters];
      end
      ini_params = [];
      for i = 1:n_p
          ini_params = [ini_params, fis_params];
      end
      lb = zeros(size(ini_params)); % Lower bound of 0 for all FIS parameters
      ub = repmat(1000, size(ini_params)); % Upper bound for FIS parameters
      
      % Reintegrate original constraints for FIS parameters
      numMFs = 3; % Assuming 3 membership functions per input as an example
      totalParams = numMFs * 2 * 3; % Example setup, adjust as necessary
      A = [];
      b = [];
      for i = 1:totalParams/3
          idx = (i-1)*3 + 1;
          A_temp = zeros(2, totalParams);
          A_temp(1, idx:idx+1) = [1, -1]; % a - b <= 0
          A_temp(2, idx+1:idx+2) = [1, -1]; % b - c <= 0
          A = [A; A_temp];
          b = [b; 0; 0];
      end

  elseif strcmp(architecture, 'fis')
    ini_params = [];
    A = [];
    b = [];
    lb = [];
    ub = [];
  else
      error("Invalid mpc_architecture value. Must be 'mpc' or 'mpfc'.");
  end

  % Set optimization options conditionally
  options_firstEval = optimoptions(solver, 'Display', 'iter', 'MaxFunEvals', 500, 'MeshTolerance', 1e-3, 'StepTolerance', 1e-3);
  options_subsequentEval = optimoptions(solver, 'Display', 'iter', 'MaxFunEvals', 500, 'MeshTolerance', 1e-4, 'StepTolerance', 1e-4);
  
  % Other setup remains consistent
  Aeq     = [];
  beq     = [];
  nonlcon = [];
  nvars = numel(ini_params);

  optimizationTimes = []; % Array to store each optimization time
  
  % Structure
  mpc_model = struct('architecture', architecture, 'solver', solver, ...
                     'options_firstEval', options_firstEval, 'options_subsequentEval', options_subsequentEval, ...
                     'n_p', n_p, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, ...
                     'lb', lb, 'ub', ub, 'nonlcon', nonlcon, 'nvars', nvars, 'prediction_model', prediction_model, ...
                     'mpc_tuning_parameters', mpc_tuning_parameters, 'optimizationTimes', optimizationTimes);

end
