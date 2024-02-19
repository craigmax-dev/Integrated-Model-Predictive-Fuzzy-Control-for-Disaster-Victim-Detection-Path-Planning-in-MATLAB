% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen mpc_model.solver and returns 
% fisArray with optimised parameters

% Updates initial optimisation parameter guess with previous results

% CHANGELOG
% - removed fis_param_hist
% - Feature: Added prediction modes

% V2.2
function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)

    % Optimization termination parameters for first eval vs subsequent
    if config.k_mpc == 0
      options = mpc_model.options_firstEval;
    else
      options = mpc_model.options_subsequentEval;
    end

    % Initialize mpc_environment_model to ensure it has the necessary fields
    mpc_environment_model = environment_model; % Copy environment model for MPC prediction
    current_step = config.k_e + 1; %NOTE CHECK IT IS THIS
     prediction_steps = round(( mpc_model.n_p * config.dt_mpc ) / config.dt_e); % Remaining steps to predict

    % Prediction Modes
    if mpc_model.prediction_model == "deterministic_exact"

      % For deterministic_exact, directly use the simulation's future states if available
      mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step:(current_step+prediction_steps));
      mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step:(current_step+prediction_steps));

    elseif mpc_model.prediction_model == "deterministic_prediction"
        
      % For deterministic_prediction, simulate the environment forward using a deterministic model
      mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step);
      mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step);

      % NOTE: Could move to separate function as currently used twice in
      % simulation
      k_precompute = 0;
      while k_precompute * config.dt_e < config.dt_mpc

        mpc_environment_model = model_environment(mpc_environment_model, k_precompute, config.dt_e);
        k_precompute = k_precompute + 1;

      end
      
    elseif mpc_model.prediction_model == "deterministic_threshold"
        % deterministic_threshold mode might require a different approach or function

    elseif mpc_model.prediction_model == "probabilistic"
        % probabilistic mode might require a different approach or function
    else
      fprintf("Warning: unrecognised MPC prediction mode selected. \n")
    end

  % Function handle
  h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);

  % Optimisation
  if mpc_model.solver == "fminsearch"
    [mpc_params, ~] = fminsearch(h_mpc, mpc_model.ini_params, options);
  elseif mpc_model.solver == "ga"
    [mpc_params,~] = ga(h_mpc, mpc_model.nvars, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, options);   
  elseif mpc_model.solver == "patternsearch"
    [mpc_params,~] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, options);   
  elseif mpc_model.solver == "particleswarm"
    [mpc_params,~] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, options);   
  end

  % Update FIS Parameters
  range = 1;
  for a = 1:agent_model.n_a
      % Number of inputs in the FIS for agent 'a'
      numInputs = numel(fisArray(a).Inputs);

      % Number of parameters per output MF = number of inputs + 1
      numParamsPerMF = numInputs + 1;

      % Assuming there is only one output, hence fisArray(a).Outputs(1)
      numMFs = numel(fisArray(a).Outputs(1).MembershipFunctions);

      for mf = 1:numMFs
          % Extract the correct range of parameters for this MF
          fis_params = mpc_params(range:range + numParamsPerMF - 1);

          % Assign parameters to the MF
          fisArray(a).Outputs(1).MembershipFunctions(mf).Parameters = fis_params;

          % Update the range for the next MF
          range = range + numParamsPerMF;
      end
  end

  % Update initial guess
  mpc_model.ini_params = mpc_params;

end

% 
% % V2.1
% function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)
% 
%   % Optimization termination parameters for first eval vs subsequent
%   if config.k_mpc == 0
%     options = mpc_model.options_firstEval;
%   else
%     options = mpc_model.options_subsequentEval;
%   end
% 
%   % Prediction Modes
%   if mpc_model.prediction_model == "deterministic_exact"
%     mpc_environment_model = environment_model;
% 
%   elseif mpc_model.prediction_model == "deterministic_prediction"
%     mpc_environment_model = environment_model;
% 
%   elseif mpc_model.prediction_model == "probabilistic_threshold"
% 
%   else
%     fprintf("Warning: unrecognised MPC prediction mode selected. \n")
%   end
% 
%   % Function handle
%   h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
% 
%   % Optimisation
%   if mpc_model.solver == "fminsearch"
%     [mpc_params, ~] = fminsearch(h_mpc, mpc_model.ini_params, options);
%   elseif mpc_model.solver == "ga"
%     [mpc_params,~] = ga(h_mpc, mpc_model.nvars, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, options);   
%   elseif mpc_model.solver == "patternsearch"
%     [mpc_params,~] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, options);   
%   elseif mpc_model.solver == "particleswarm"
%     [mpc_params,~] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, options);   
%   end
% 
%   % Update FIS Parameters
%   range = 1;
%   for a = 1:agent_model.n_a
%       % Number of inputs in the FIS for agent 'a'
%       numInputs = numel(fisArray(a).Inputs);
% 
%       % Number of parameters per output MF = number of inputs + 1
%       numParamsPerMF = numInputs + 1;
% 
%       % Assuming there is only one output, hence fisArray(a).Outputs(1)
%       numMFs = numel(fisArray(a).Outputs(1).MembershipFunctions);
% 
%       for mf = 1:numMFs
%           % Extract the correct range of parameters for this MF
%           fis_params = mpc_params(range:range + numParamsPerMF - 1);
% 
%           % Assign parameters to the MF
%           fisArray(a).Outputs(1).MembershipFunctions(mf).Parameters = fis_params;
% 
%           % Update the range for the next MF
%           range = range + numParamsPerMF;
%       end
%   end
% 
%   % Update initial guess
%   mpc_model.ini_params = mpc_params;
% 
% end
