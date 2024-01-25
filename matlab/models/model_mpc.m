% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen mpc_model.solver and returns 
% fisArray with optimised parameters

% Updates initial optimisation parameter guess with previous results

% CHANGELOG
% - removed fis_param_hist

% TODO
% Implement three different MPC architectures: centralised, distributed,
% clustering

function [fisArray, mpc_model] = model_mpc(fisArray, agent_model, config, environment_model, mpc_model)

  % Function handle
  h_mpc = @(params)mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model);

  % Optimisation
  if mpc_model.solver == "fminsearch"
    [mpc_params, ~] = fminsearch(h_mpc, mpc_model.ini_params, mpc_model.options);
  elseif mpc_model.solver == "ga"
    [mpc_params,~] = ga(h_mpc, mpc_model.nvars, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, mpc_model.options);   
  elseif mpc_model.solver == "patternsearch"
    [mpc_params,~] = patternsearch(h_mpc, mpc_model.ini_params, mpc_model.A, mpc_model.b, mpc_model.Aeq, mpc_model.beq, mpc_model.lb, mpc_model.ub, mpc_model.nonlcon, mpc_model.options);   
  elseif mpc_model.solver == "particleswarm"
    [mpc_params,~] = particleswarm(h_mpc, mpc_model.nvars, mpc_model.lb, mpc_model.ub, mpc_model.options);   
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
