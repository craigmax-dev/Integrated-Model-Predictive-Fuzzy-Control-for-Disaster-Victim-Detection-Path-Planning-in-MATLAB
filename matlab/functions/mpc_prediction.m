%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

% V2

% CHANGELOG
% Removed flag_mpc
% Structures refactor

% TODO
% Check that models are only updated during function and not returned to main
% function. Also check correct data is accessed in MPC prediction. E.g. do we
% initialise m_dw_e as 0 for the prediction? Other parameters of concern: k
% - Validate V2.2 function

% % V2.2
% function s_obj_pred = mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model)
% 
%   %% Initialize Variables
%   k_pred = 0;
%   s_obj_pred = 0;
%   range = 1;
% 
%   %% Prediction Loop
%   while k_pred < config.dk_mpc*mpc_model.n_p
% 
%     %% Update FIS parameters if needed
%     if mod(k_pred, config.dk_mpc) == 0
%       % Depending on the architecture, params may represent a single agent, multiple agents, or clusters.
%       % The structure of params needs to correspond to how many agents or clusters are being optimized.
%       % Here we assume params is structured correctly for the current architecture.
%       range = updateFISParameters(fisArray, params, range);
%     end
% 
%     %% Execute models (Path planning, Agent model, Environment model)
%     [agent_model, environment_model] = executeModels(agent_model, environment_model, config, fisArray, k_pred);
% 
%     %% Objective function evaluation
%     [s_obj_pred, ~] = calc_obj(...
%       config.weight, environment_model.m_f, agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
%       config.dt_s, config.s_obj, agent_model.c_f_s, config.t);
% 
%     %% Advance timestep
%     k_pred = k_pred + 1;
%     config.k = config.k + 1;
% 
%   end
% end
% 
% function range = updateFISParameters(fisArray, params, startRange)
%   range = startRange;
%   for a = 1:length(fisArray)
%       for o = 1:numel(fisArray(a).Outputs)
%           for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
%               numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
%               newParams = params(range:range+numParams-1);
%               fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
%               range = range + numParams;
%           end
%       end
%   end
% end
% 
% function [agent_model, environment_model] = executeModels(agent_model, environment_model, config, fisArray, k_pred)
%   if mod(k_pred, config.dk_c) == 0
%     [agent_model] = model_fis(agent_model, environment_model, config, fisArray);
%   end
%   if mod(k_pred, config.dk_a) == 0
%     agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
%   end
%   if mod(k_pred, config.dk_e) == 0
%     environment_model = model_environment(environment_model, config.dt_e);          
%   end
% end

% V2.1
function s_obj_pred = mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model)
      
  %% Variables
  % Counters
  k_pred  = 0;
  k_a     = 0;
  k_c     = 0;
  k_e     = 0;
  k_mpc   = 0;
  s_obj_pred   = 0;
  
  % Range
  range = 1;
  
  %% Prediction
  while k_pred < config.dk_mpc*mpc_model.n_p
    
    %% Update FIS parameters
    if k_mpc*config.dk_mpc <= k_pred
      
      % V2
      for a = 1:agent_model.n_a
          % Iterate over each output of the FIS
          for o = 1:numel(fisArray(a).Outputs)
              % Iterate over each MF of the output
              for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
                  % Determine the number of parameters for this MF
                  numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
                  
                  % Extract the parameters from 'params'
                  newParams = params(range:range+numParams-1);
                  range = range + numParams;
                  
                  % Assign the new parameters to the MF
                  fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
              end
          end
      end

     
      k_mpc = k_mpc + 1;
    end

    
    %% Path planning
    if k_c*config.dk_c <= k_pred
      [agent_model] = model_fis(agent_model, environment_model.v_w, environment_model.ang_w, config, fisArray);
      k_c = k_c + 1;
    end
    
    %% Agent model
    if k_a*config.dk_a <= k_pred
          agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
      k_a = k_a + 1;
    end
    
    %% V2 - Environment model - Use predicted states instead of recalculating
    % TODO: FINISH

    % V1 - Environment model 
    if k_e*config.dk_e <= k_pred
      environment_model = model_environment(environment_model, k_e, config.dt_e); 
      k_e = k_e + 1;
    end
    
    %% Objective function evaluation
    [s_obj_pred, ~] = calc_obj(...
      config.weight, environment_model.m_f, agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
      config.dt_s, config.s_obj, agent_model.c_f_s, config.t);
          
    %% Advance timestep
    k_pred = k_pred + 1;
    config.k      = config.k + 1;

  end
end
