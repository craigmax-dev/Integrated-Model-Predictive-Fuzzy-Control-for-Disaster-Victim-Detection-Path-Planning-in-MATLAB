% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen mpc_model.solver and returns 
% fisArray with optimised parameters

% Updates initial optimisation parameter guess with previous results

% CHANGELOG
% - removed fis_param_hist
% - Implement three different MPC architectures: centralised, distributed,
% clustering

% TODO
% Testing V2.2


% % V2.2
% function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model, architecture, radius)
% 
%   % Define the optimization function based on solver
%   h_mpc = @(params) mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model);
% 
%   % Optimization termination parameters for first eval vs subsequent
%   if config.k_mpc == 0
%     options = options_firstEval;
%   else
%     options = options_subsequentEval;
%   end
% 
%   % Handle architecture-specific optimization
%   switch architecture
%       case 'centralised'
%           % Centralised architecture: Run the optimization as it is
%           mpc_params = runOptimization(mpc_model, h_mpc, options);
% 
%       case 'distributed'
%           % Distributed architecture: Run a separate optimization for each agent
%           mpc_params = [];
%           for a = 1:agent_model.n_a
%               % Adjust optimization function for each agent
%               h_mpc_agent = @(params) mpc_prediction(params, agent_model, config, environment_model, fisArray(a), mpc_model);
% 
%               % Run optimization for each agent and concatenate parameters
%               agent_params = runOptimization(mpc_model, h_mpc_agent, options);
%               mpc_params = [mpc_params, agent_params];
%           end
% 
%       case 'clustered'
%           % Clustered architecture: Group agents and run optimization per cluster
%           clusters = groupAgentsIntoClusters(agent_model, radius); % Implement this function based on your needs
%           mpc_params = [];
%           for c = 1:numel(clusters)
%               % Adjust optimization function for cluster
%               h_mpc_cluster = @(params) mpc_cluster_prediction(params, clusters{c}, config, environment_model, fisArray, mpc_model);
% 
%               % Run optimization for each cluster and concatenate parameters
%               cluster_params = runOptimization(mpc_model, h_mpc_cluster, options);
%               mpc_params = [mpc_params, cluster_params];
%           end
%   end
% 
%   % Update FIS Parameters using the logic already provided in your function
%   % Assuming the logic provided earlier in your function is used here
%   % ...
% 
%   % Update initial guess
%   mpc_model.ini_params = mpc_params;
% 
% end
% 
% function mpc_params = runOptimization(mpc_model, h_mpc, options)
%     % Run optimization based on the solver specified in mpc_model
%     if mpc_model.solver == "fminsearch"
%         [mpc_params, ~] = fminsearch(h_mpc, mpc_model.ini_params, options);
%     elseif mpc_model.solver == "ga"
%         [mpc_params,~] = ga(h_mpc, mpc_model.nvars, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, options);   
%     elseif mpc_model.solver == "patternsearch"
%         [mpc_params,~] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, options);   
%     elseif mpc_model.solver == "particleswarm"
%         [mpc_params,~] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, options);   
%     end
% end

% % Placeholder for a function to group agents into clusters based on proximity
% % You will need to implement this based on your specific requirements
% function clusters = groupAgentsIntoClusters(agent_model, radius)
%     % Implement clustering logic here
%     % For example, use spatial clustering algorithms like DBSCAN, k-means (with precomputed distances), etc.
%     % This should return a cell array where each cell contains the indices or identifiers of agents in a cluster
%     clusters = {}; % Placeholder
% end

% % % V2.2
% function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)
%     % Perform environment prediction based on the specified mode
%     if strcmp(mpc_model.prediction_mode, 'deterministic_prediction') || strcmp(mpc_model.prediction_mode, 'probabilistic_threshold')
%         % Precompute the environmental predictions for the entire horizon
%         [predicted_m_f, predicted_m_dw_e] = precompute_environment_predictions(environment_model, config, mpc_model);
% 
%         % Pass the predicted environment states to the MPC prediction function
%         h_mpc = @(params)mpc_prediction(params, agent_model, config, predicted_m_f, predicted_m_dw_e, fisArray, mpc_model);
%     else
%         % For deterministic_exact, use the current environment model directly
%         h_mpc = @(params)mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model);
%     end
% 
% 
% 
%     % Update initial guess
%     mpc_model.ini_params = mpc_params;
% end
% 
% function [predicted_m_f, predicted_m_dw_e] = precompute_environment_predictions(environment_model, config, mpc_model)
%     % Initialize matrices to store predicted m_f and m_dw_e
%     % Assuming prediction_horizon and dt_s are defined in config or mpc_model
%     prediction_steps = config.prediction_horizon / config.dt_s;
%     predicted_m_f = zeros([size(environment_model.m_f), prediction_steps]);
%     predicted_m_dw_e = zeros([size(environment_model.m_dw_e), prediction_steps]);
% 
%     % Populate predicted_m_f and predicted_m_dw_e based on the prediction mode
%     % This will involve iterating over prediction steps and applying the 
%     % deterministic or probabilistic fire spread model accordingly
%     % Implementation of this depends on the details of the environment prediction model
% end

% V2.1
function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)

  % Function handle
  h_mpc = @(params)mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model);

  % Optimization termination parameters for first eval vs subsequent
  if config.k_mpc == 0
    options = mpc_model.options_firstEval;
  else
    options = mpc_model.options_subsequentEval;
  end

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
