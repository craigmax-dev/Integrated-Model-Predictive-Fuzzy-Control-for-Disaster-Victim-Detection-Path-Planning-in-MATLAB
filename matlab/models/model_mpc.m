% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen mpc_model.solver and returns 
% fisArray with optimised parameters

% Updates initial optimisation parameter guess with previous results

% CHANGELOG
% - removed fis_param_hist
% - Feature: Prediction modes
% - Feature: Controller architectures: mpc and mpfc
% - Feature: Controller structures: centralised, decentralised, clustered (TBC)
% - Feature: Implemented prediction horizon using mpc_pred 
% - Bugfix: Corrected use of mpc_environment_model parameters

% V2.4 - STRUCTURES
% TO DO
% Cleanup - Clear m_f etc from mpc_environment_model as they are not relevant
% Bugfix - fix proper use of k_precompute * config.dk_e <= config.dk_pred + config.dk_e

function [fisArray, mpc_model, agent_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)
    mpc_model = setOptimizationOptions(config, mpc_model);
    mpc_environment_model = setupPredictionMode(mpc_model, environment_model, config);

    if strcmp(mpc_model.structure, 'centralised')
        [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray);
        [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params);
    elseif strcmp(mpc_model.structure, 'decentralised')
        params = [];
        for agentIndex = 1:agent_model.n_a
            [individualParams, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray, agentIndex);
            params = [params, individualParams];
        end
        [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params);
    else
        error('Unrecognised MPC structure: %s', mpc_model.structure);
    end

    mpc_model.ini_params = params;
end

function mpc_model = setOptimizationOptions(config, mpc_model)
  if config.k_mpc == 0
    mpc_model.options = mpc_model.options_firstEval;
  else
    mpc_model.options = mpc_model.options_subsequentEval;
  end
end

function mpc_environment_model = setupPredictionMode(mpc_model, environment_model, config)
    % Initialize mpc_environment_model
    mpc_environment_model = environment_model; % Copy environment model for MPC prediction
    current_step = config.k_e + 1; % Note: check if it should be config.k_e or config.k_e + 1
    prediction_steps = ceil((mpc_model.n_p * config.dk_pred) / config.dk_e); % Remaining steps to predict

    % Prediction Modes
    if mpc_model.prediction_model == "deterministic_exact"

        % For deterministic_exact, directly use the simulation's future states if available
        mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step:(current_step+prediction_steps+1));
        mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step:(current_step+prediction_steps+1));

    elseif mpc_model.prediction_model == "deterministic_prediction"

        % For deterministic_prediction, simulate the environment forward using a deterministic model
        mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step);
        mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step);

        % Precompute future states based on the current state
        k_precompute = 0;
        while k_precompute * config.dk_e <= config.dk_pred + config.dk_e
            mpc_environment_model = model_environment(mpc_environment_model, k_precompute, config.dt_e);
            k_precompute = k_precompute + 1; 
        end

    else
        fprintf("Warning: unrecognised MPC prediction mode selected. \n")
    end

end

% V5
function [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray, agentIndex)
    tic; % Start timing the optimization
    params = []; % Initialize params to ensure it has a value

    % Prepare the optimization function handle based on the structure
    if strcmp(mpc_model.structure, 'decentralised')
        % For decentralised optimization, ensure the parameter bounds match the subset of parameters being optimized.
        totalParams = length(mpc_model.ini_params);
        paramsPerAgent = totalParams / agent_model.n_a;
        paramStartIndex = round((agentIndex-1) * paramsPerAgent) + 1;
        paramEndIndex = round(agentIndex * paramsPerAgent);

        % Extract the subset of parameters and corresponding bounds for the current agent
        localParamsInit = mpc_model.ini_params(paramStartIndex:paramEndIndex);
        localLb = mpc_model.lb(paramStartIndex:paramEndIndex);
        localUb = mpc_model.ub(paramStartIndex:paramEndIndex);

        % Decentralized optimization: Optimize only the parameters of the current agent
        h_mpc = @(localParams)mpc_prediction(localParams, agent_model, config, mpc_environment_model, fisArray, mpc_model, agentIndex);

        % Adjust the optimize_solver call to use the local bounds
        [params, ~] = optimize_solver(mpc_model, h_mpc, localParamsInit, localLb, localUb);
    else
        % Centralized optimization uses the full parameter set and bounds
        h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
        [params, ~] = optimize_solver(mpc_model, h_mpc, mpc_model.ini_params, mpc_model.lb, mpc_model.ub);

    end

    optimizationTime = toc; % End timing and store the elapsed time
    mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
end

% 
% % V4
% function [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray, agentIndex)
%     tic; % Start timing the optimization
%     % Initialize params to ensure it has a value
%     params = [];
% 
%     % Calculate the number of parameters per agent dynamically
%     totalParams = length(mpc_model.ini_params); % Assume ini_params contains the full parameter set for all agents
%     paramsPerAgent = totalParams / agent_model.n_a; % Assuming an even distribution of parameters
% 
%     % Prepare the optimization function handle based on the structure
%     if strcmp(mpc_model.structure, 'decentralised')
%         % Calculate the start and end indices for the parameters of the current agent
%         paramStartIndex = round((agentIndex-1) * paramsPerAgent) + 1;
%         paramEndIndex = round(agentIndex * paramsPerAgent);
% 
%         % Extract the subset of parameters for the current agent
%         localParamsInit = mpc_model.ini_params(paramStartIndex:paramEndIndex);
% 
%         % Decentralized optimization: Optimize only the parameters of the current agent
%         h_mpc = @(localParams)mpc_prediction(localParams, agent_model, config, mpc_environment_model, fisArray, mpc_model, agentIndex);
% 
%         % Perform optimization using the subset of parameters
%         [optimizedLocalParams, ~] = optimize_solver(mpc_model, h_mpc, localParamsInit);
% 
%         % Update the global params array with optimized parameters for the current agent
%         params = mpc_model.ini_params; % Start with the initial parameters
%         params(paramStartIndex:paramEndIndex) = optimizedLocalParams; % Update with optimized values
%     else
%         % Centralized optimization: Optimize all parameters at once
%         h_mpc = @(globalParams)mpc_prediction(globalParams, agent_model, config, mpc_environment_model, fisArray, mpc_model);
%         [params, ~] = optimize_solver(mpc_model, h_mpc, mpc_model.ini_params);
%     end
% 
%     optimizationTime = toc; % End timing and store the elapsed time
%     mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
% end


% % % V3
% function [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray, agentIndex)
%     tic; % Start timing the optimization
%     params = []; % Initialize params to ensure it has a value
% 
%     % Prepare the optimization function handle based on the structure
%     if strcmp(mpc_model.structure, 'decentralised')
%         % Decentralised optimization: Pass the agentIndex to the prediction function
%         h_mpc = @(localParams)mpc_prediction(localParams, agent_model, config, mpc_environment_model, fisArray, mpc_model, agentIndex);
%     else
%         % Centralised optimization
%         h_mpc = @(localParams)mpc_prediction(localParams, agent_model, config, mpc_environment_model, fisArray, mpc_model);
%     end
% 
%     try
%         [params, ~] = optimize_solver(mpc_model, h_mpc);
%     catch ME
%         warning('Optimization did not complete successfully: %s', ME.message);
%         % Handle cases where optimization fails or is not applicable
%         % Consider setting params to a default or previous value if optimization is critical
%     end
% 
%     optimizationTime = toc; % End timing and store the elapsed time
%     mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
% end


% % V2
% function [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray, agentIndex)
%   tic; % Start timing the optimization
%   if strcmp(mpc_model.structure, 'decentralised')
%       % Decentralised optimization: Pass the agentIndex to the prediction function
%       h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model, agentIndex);
%   else
%       % Centralised optimization
%       h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
%   end
% 
%   [params, ~] = optimize_solver(mpc_model, h_mpc);
%   optimizationTime = toc; % End timing and store the elapsed time
% 
%   % Update the mpc_model with the optimization time
%   mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
% end


% function [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray)
%     tic; % Start timing the optimization
%     h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
%     [params, ~] = optimize_solver(mpc_model, h_mpc);
%     optimizationTime = toc; % End timing and store the elapsed time
% 
%     % Update the mpc_model with the optimization time
%     mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
% end

% function [params, fval] = optimize_solver(mpc_model, h_mpc)
%     % Initialize output variables to default values
%     params = [];
%     fval = Inf;
% 
%     % Solver selection and optimization execution
%     switch mpc_model.solver
%         case "fminsearch"
%             [params, fval] = fminsearch(h_mpc, mpc_model.ini_params, mpc_model.options);
%         case "ga"
%             [params, fval] = ga(h_mpc, mpc_model.nvars, [], [], [], [], mpc_model.lb, mpc_model.ub, [], mpc_model.intCon, mpc_model.options);
%         case "patternsearch"
%             [params, fval] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, mpc_model.options);
%         case "particleswarm"
%             [params, fval] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, mpc_model.options);
%         otherwise
%             error('Unrecognised solver: %s', mpc_model.solver);
%     end
% 
%     % Ensure params has been assigned; otherwise, handle the error or fallback case
%     if isempty(params)
%         % Handle the case where params could not be assigned due to an unrecognised solver or other error
%         warning('params was not assigned. Check the solver configuration and input parameters.');
%     end
% end

function [params, fval] = optimize_solver(mpc_model, h_mpc, initialParams, lb, ub)
    % Initialization for output variables
    params = [];
    fval = Inf;

    % Adjusting solver selection and optimization execution to utilize passed bounds
    switch mpc_model.solver
        case "fminsearch"
            % Note: fminsearch does not directly support bounds, consider using a bounded solver for optimization with bounds
            [params, fval] = fminsearch(h_mpc, initialParams, mpc_model.options);
        case "ga"
            % Genetic algorithm supports bounds
            [params, fval] = ga(h_mpc, length(initialParams), [], [], [], [], lb, ub, [], mpc_model.intCon, mpc_model.options);
        case "patternsearch"
            % Pattern search supports bounds
            [params, fval] = patternsearch(h_mpc, initialParams, [], [], [], [], lb, ub, [], mpc_model.options);
        case "particleswarm"
            % Particle swarm optimization supports bounds
            [params, fval] = particleswarm(h_mpc, length(initialParams), lb, ub, mpc_model.options);
        otherwise
            error('Unrecognized solver: %s', mpc_model.solver);
    end

    if isempty(params)
        % Handle the case where optimization did not return parameters
        warning('Optimization did not return parameters. Check the solver configuration and input parameters.');
    end
end


% function [params, fval] = optimize_solver(mpc_model, h_mpc, ini_params)
%     % Initialize output variables to default values
%     fval = Inf;
% 
%     % Solver selection and optimization execution
%     switch mpc_model.solver
%         % Add the ini_params argument to each solver call
%         case "fminsearch"
%             [params, fval] = fminsearch(h_mpc, ini_params, mpc_model.options);
%         case "ga"
%             [params, fval] = ga(h_mpc, length(ini_params), [], [], [], [], mpc_model.lb, mpc_model.ub, [], mpc_model.intCon, mpc_model.options);
%         case "patternsearch"
%             [params, fval] = patternsearch(h_mpc, ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, mpc_model.options);
%         case "particleswarm"
%             [params, fval] = particleswarm(h_mpc, length(ini_params), mpc_model.lb, mpc_model.ub, mpc_model.options);
%         otherwise
%             error('Unrecognized solver: %s', mpc_model.solver);
%     end
% end
