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
    mpc_model = setOptimizationOptions(config, mpc_model);

    % Setup the prediction mode and initialize mpc_environment_model within it
    mpc_environment_model = setupPredictionMode(mpc_model, environment_model, config);

    % Perform optimization
    [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray);

    % Update model based on architecture
    [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params);

    % Update initial guess for next optimization
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
    current_step = config.k_e + 1;
    prediction_steps = round((mpc_model.n_p * config.dt_mpc) / config.dt_e); % Remaining steps to predict
    
    % Prediction Modes
    if mpc_model.prediction_model == "deterministic_exact"
        
        % For deterministic_exact, directly use the simulation's future states if available
        mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step:(current_step+prediction_steps));
        mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step:(current_step+prediction_steps));

    elseif mpc_model.prediction_model == "deterministic_prediction"
        
        % For deterministic_prediction, simulate the environment forward using a deterministic model
        mpc_environment_model.m_f_series = environment_model.m_f_series(:, :, current_step);
        mpc_environment_model.m_dw_e_series = environment_model.m_dw_e_series(:, :, current_step);
        
        % Precompute future states based on the current state
        k_precompute = 0;
        while k_precompute * config.dt_e < config.dt_mpc
            mpc_environment_model = model_environment(mpc_environment_model, k_precompute, config.dt_e);
            k_precompute = k_precompute + 1;
        end

    else
        fprintf("Warning: unrecognised MPC prediction mode selected. \n")
    end
end

function [params, mpc_model] = performOptimization(mpc_model, agent_model, config, mpc_environment_model, fisArray)
    tic; % Start timing the optimization
    h_mpc = @(params)mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model);
    [params, ~] = optimize_solver(mpc_model, h_mpc);
    optimizationTime = toc; % End timing and store the elapsed time
    
    % Update the mpc_model with the optimization time
    mpc_model.optimizationTimes = [mpc_model.optimizationTimes, optimizationTime];
end

function [params, fval] = optimize_solver(mpc_model, h_mpc)
    % Initialize output variables to default values
    params = [];
    fval = Inf;

    % Solver selection and optimization execution
    switch mpc_model.solver
        case "fminsearch"
            [params, fval] = fminsearch(h_mpc, mpc_model.ini_params, mpc_model.options);
        case "ga"
            [params, fval] = ga(h_mpc, mpc_model.nvars, [], [], [], [], mpc_model.lb, mpc_model.ub, [], mpc_model.intCon, mpc_model.options);
        case "patternsearch"
            [params, fval] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, mpc_model.options);
        case "particleswarm"
            [params, fval] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, mpc_model.options);
        otherwise
            error('Unrecognized solver: %s', mpc_model.solver);
    end

    % Ensure params has been assigned; otherwise, handle the error or fallback case
    if isempty(params)
        % Handle the case where params could not be assigned due to an unrecognized solver or other error
        warning('params was not assigned. Check the solver configuration and input parameters.');
        % Consider setting params to default values or handling the error appropriately
    end
end
