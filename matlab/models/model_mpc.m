% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen mpc_model.solver and returns 
% fisArray with optimised parameters

% Updates initial optimisation parameter guess with previous results

% CHANGELOG
% - removed fis_param_hist
% - Feature: Prediction modes
% - Feature: Controller architectures: mpc and mpfc

% V2.3
function [fisArray, mpc_model, agent_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)

    % Set optimization options
    options = setOptimizationOptions(config, mpc_model);

    % Initialize the environment model for MPC
    [current_step, prediction_steps, mpc_environment_model] = initializeEnvironmentModel(config, mpc_model, environment_model);

    % Setup the prediction mode
    mpc_environment_model = setupPredictionMode(mpc_model, mpc_environment_model, environment_model, current_step, prediction_steps, config);

    % Perform optimization
    mpc_params = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray);

    % Update model based on architecture
    [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, mpc_params);

    % Update initial guess for next optimization
    mpc_model.ini_params = mpc_params;
    % mpc_model.ini_params = randomiseAgentTaskQueue(agent_model.n_a, agent_model.n_q, agent_model.n_x_s, agent_model.n_y_s);

end

function options = setOptimizationOptions(config, mpc_model)
  if config.k_mpc == 0
    options = mpc_model.options_firstEval;
  else
    options = mpc_model.options_subsequentEval;
  end
end

function [current_step, prediction_steps, mpc_environment_model] = initializeEnvironmentModel(config, mpc_model, environment_model)
  mpc_environment_model = environment_model; % Copy environment model for MPC prediction
  current_step = config.k_e + 1; 
  prediction_steps = round((mpc_model.n_p * config.dt_mpc) / config.dt_e); % Remaining steps to predict
end

function mpc_environment_model = setupPredictionMode(mpc_model, mpc_environment_model, environment_model, current_step, prediction_steps, config)
  
  % Prediction Modes
  if mpc_model.prediction_model == "deterministic_exact"

    % For deterministic_exact, directly use the simulation's future states if available
    mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step:(current_step+prediction_steps));
    mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step:(current_step+prediction_steps));

  elseif mpc_model.prediction_model == "deterministic_prediction"

    % For deterministic_prediction, simulate the environment forward using a deterministic model
    mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step);
    mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step);

    % NOTE: Can move inside environment model
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
end

function [mpc_params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray)
    tic; % Start timing the optimization
    h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
    [mpc_params, ~] = optimize_solver(mpc_model, h_mpc);
    optimizationTime = toc; % End timing and store the elapsed time
    
    % Update the mpc_model with the optimization time
    mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
end

% function mpc_params = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray)
%     h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
%     [mpc_params, ~] = optimize_solver(mpc_model, h_mpc);
% end

function [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, mpc_params)
    if strcmp(mpc_model.architecture, 'mpfc')
        [fisArray] = updateFISParameters(fisArray, mpc_params, agent_model);
    elseif strcmp(mpc_model.architecture, 'mpc')
        [agent_model] = updateAgentModelTarget(agent_model, mpc_params);
    else
        error('Invalid architecture value. Must be "mpfc" or "mpc".');
    end
end

function [fisArray] = updateFISParameters(fisArray, mpc_params, agent_model)

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
end

function [agent_model] = updateAgentModelTarget(agent_model, mpc_params)
    % Initialize the 'range' variable
    range = 1;
    
    % Iterate through agents and queues/targets
    for a = 1:agent_model.n_a
        for q = 1:size(agent_model.a_target, 3)
            % Ensure 'range' does not exceed the length of 'mpc_params'
            if range + 1 <= length(mpc_params)
                % Update 'a_target' with parameters from 'mpc_params'
                agent_model.a_target(a, :, q) = mpc_params(range:range+1);
                % Increment 'range' to point to the next set of parameters
                range = range + 2;
            else
                error('updateAgentModelTarget:OutOfRange', 'The range variable exceeds the length of mpc_params.');
            end
        end
    end
end

function [mpc_params, fval] = optimize_solver(mpc_model, h_mpc)
    % Initialize output variables to default values
    mpc_params = [];
    fval = Inf;

    % Solver selection and optimization execution
    switch mpc_model.solver
        case "fminsearch"
            [mpc_params, fval] = fminsearch(h_mpc, mpc_model.ini_params, mpc_model.options_firstEval);
        case "ga"
            [mpc_params, fval] = ga(h_mpc, mpc_model.nvars, [], [], [], [], mpc_model.lb, mpc_model.ub, [], mpc_model.intCon, mpc_model.options_firstEval);
        case "patternsearch"
            [mpc_params, fval] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, mpc_model.options_firstEval);
        case "particleswarm"
            [mpc_params, fval] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, mpc_model.options_firstEval);
        otherwise
            error('Unrecognized solver: %s', mpc_model.solver);
    end

    % Ensure mpc_params has been assigned; otherwise, handle the error or fallback case
    if isempty(mpc_params)
        % Handle the case where mpc_params could not be assigned due to an unrecognized solver or other error
        warning('mpc_params was not assigned. Check the solver configuration and input parameters.');
        % Consider setting mpc_params to default values or handling the error appropriately
    end
end


% % V2.2
% function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)
% 
%     % Optimization termination parameters for first eval vs subsequent
%     if config.k_mpc == 0
%       options = mpc_model.options_firstEval;
%     else
%       options = mpc_model.options_subsequentEval;
%     end
% 
%     % Initialize mpc_environment_model to ensure it has the necessary fields
%     mpc_environment_model = environment_model; % Copy environment model for MPC prediction
%     current_step = config.k_e + 1; %NOTE CHECK IT IS THIS
%     prediction_steps = round(( mpc_model.n_p * config.dt_mpc ) / config.dt_e); % Remaining steps to predict
% 
%     % Prediction Modes
%     if mpc_model.prediction_model == "deterministic_exact"
% 
%       % For deterministic_exact, directly use the simulation's future states if available
%       mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step:(current_step+prediction_steps));
%       mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step:(current_step+prediction_steps));
% 
%     elseif mpc_model.prediction_model == "deterministic_prediction"
% 
%       % For deterministic_prediction, simulate the environment forward using a deterministic model
%       mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step);
%       mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step);
% 
%       % NOTE: Could move to separate function as currently used twice in
%       % simulation
%       k_precompute = 0;
%       while k_precompute * config.dt_e < config.dt_mpc
% 
%         mpc_environment_model = model_environment(mpc_environment_model, k_precompute, config.dt_e);
%         k_precompute = k_precompute + 1;
% 
%       end
% 
%     elseif mpc_model.prediction_model == "deterministic_threshold"
%         % deterministic_threshold mode might require a different approach or function
% 
%     elseif mpc_model.prediction_model == "probabilistic"
%         % probabilistic mode might require a different approach or function
%     else
%       fprintf("Warning: unrecognised MPC prediction mode selected. \n")
%     end
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

