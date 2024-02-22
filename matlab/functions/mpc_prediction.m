%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

% V2

% CHANGELOG
% Removed flag_mpc
% Refactor: Structures
% Feature: Controller architectures
% Refactor: Modularisation

% % V2.3
function s_obj_pred = mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model)

  %% Variables
  % Counters
  k_pred  = 0;
  k_a     = 0;
  k_c     = 0;
  k_e     = 0;
  k_mpc   = 0;
  s_obj_pred   = 0;

  % Range for parameters
  range = 1;

  %% Prediction
  while k_pred < config.dk_mpc*mpc_model.n_p

    if k_mpc*config.dk_mpc <= k_pred

        if strcmp(mpc_model.architecture, 'mpfc')

            % Modularized FIS parameter update
            fisArray = updateFISParameters(fisArray, params, range);

        elseif strcmp(mpc_model.architecture, 'mpc')

            % Modularized agent target update
            agent_model = updateAgentTargets(agent_model, params, range);

        else
            error('Invalid architecture value. Must be "mpfc" or "mpc".');
        end

      k_mpc = k_mpc + 1;

    end

    %% FIS Path planning
    if ~strcmp(mpc_model.architecture, 'mpc')
      if k_c*config.dk_c <= k_pred
        [agent_model] = model_fis(agent_model, environment_model.v_w, environment_model.ang_w, config, fisArray);
        k_c = k_c + 1;
      end
    end

    %% Agent model
    if k_a*config.dk_a <= k_pred
          agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
      k_a = k_a + 1;
    end

    % V2 - Environment model - Use predicted states instead of recalculating
    if k_e*config.dk_e <= k_pred
      k_e = k_e + 1;
    end

    %% Objective function evaluation
    [s_obj_pred, ~] = calc_obj(...
      config.weight, environment_model.m_f(k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
      config.dt_s, config.s_obj, agent_model.c_f_s, config.t);

    %% Advance timestep
    k_pred = k_pred + 1;
    config.k      = config.k + 1;
  end
end

function fisArray = updateFISParameters(fisArray, params, range)
    for a = 1:numel(fisArray)
        for o = 1:numel(fisArray(a).Outputs)
            for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
                numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
                if range + numParams - 1 <= length(params)
                    newParams = params(range:range+numParams-1);
                    fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
                    range = range + numParams;
                else
                    error('updateFISParameters:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
                end
            end
        end
    end
end

function agent_model = updateAgentTargets(agent_model, params, range)
    for a = 1:agent_model.n_a
        for q = 1:size(agent_model.a_target, 3)
            if range + 1 <= length(params)
                agent_model.a_target(a, :, q) = params(range:range+1);
                range = range + 2; % Move to the next set of parameters
            else
                error('updateAgentTargets:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
            end
        end
    end
end

% % V2.2
% function s_obj_pred = mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model)
% 
%   %% Variables
%   % Counters
%   k_pred  = 0;
%   k_a     = 0;
%   k_c     = 0;
%   k_e     = 0;
%   k_mpc   = 0;
%   s_obj_pred   = 0;
% 
%   % Range for parameters
%   range = 1;
% 
%   %% Prediction
%   while k_pred < config.dk_mpc*mpc_model.n_p
% 
%     if k_mpc*config.dk_mpc <= k_pred
% 
%       if strcmp(mpc_model.architecture, 'mpfc')
% 
%         % Update FIS parameters
%         for a = 1:agent_model.n_a
%           for o = 1:numel(fisArray(a).Outputs)
%             for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
%               numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
%               newParams = params(range:range+numParams-1);
%               range = range + numParams;
%               fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
%             end
%           end
%         end
% 
%       elseif strcmp(mpc_model.architecture, 'mpc')
% 
%         % Update agent_model.a_target using params
%         for a = 1:agent_model.n_a
%           for q = 1:size(agent_model.a_target, 3)
%             % Check if 'range' can safely access 'params'
%             if range + 1 <= length(params)
%               agent_model.a_target(a, :, q) = params(range:range+1);
%               range = range + 2; % Move to the next set of parameters
%             else
%               error('mpc_prediction:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
%             end
%           end
%         end
% 
%       else
%         error('Invalid architecture value. Must be "mpfc" or "mpc".');
%       end
% 
%       k_mpc = k_mpc + 1;
%     end
% 
%     %% FIS Path planning
%     if ~strcmp(mpc_model.architecture, 'mpc')
%       if k_c*config.dk_c <= k_pred
%         [agent_model] = model_fis(agent_model, environment_model.v_w, environment_model.ang_w, config, fisArray);
%         k_c = k_c + 1;
%       end
%     end
% 
%     %% Agent model
%     if k_a*config.dk_a <= k_pred
%           agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
%       k_a = k_a + 1;
%     end
% 
%     % V2 - Environment model - Use predicted states instead of recalculating
%     if k_e*config.dk_e <= k_pred
%       k_e = k_e + 1;
%     end
% 
%     %% Objective function evaluation
%     [s_obj_pred, ~] = calc_obj(...
%       config.weight, environment_model.m_f(k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
%       config.dt_s, config.s_obj, agent_model.c_f_s, config.t);
% 
%     %% Advance timestep
%     k_pred = k_pred + 1;
%     config.k      = config.k + 1;
%   end
% end

% % V2.1
% function s_obj_pred = mpc_prediction(params, agent_model, config, environment_model, fisArray, mpc_model)
% 
%   %% Variables
%   % Counters
%   k_pred  = 0;
%   k_a     = 0;
%   k_c     = 0;
%   k_e     = 0;
%   k_mpc   = 0;
%   s_obj_pred   = 0;
% 
%   % Range
%   range = 1;
% 
%   %% Prediction
%   while k_pred < config.dk_mpc*mpc_model.n_p
% 
%     %% Update FIS parameters
%     if k_mpc*config.dk_mpc <= k_pred
% 
%       % V2
%       for a = 1:agent_model.n_a
%           % Iterate over each output of the FIS
%           for o = 1:numel(fisArray(a).Outputs)
%               % Iterate over each MF of the output
%               for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
%                   % Determine the number of parameters for this MF
%                   numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
% 
%                   % Extract the parameters from 'params'
%                   newParams = params(range:range+numParams-1);
%                   range = range + numParams;
% 
%                   % Assign the new parameters to the MF
%                   fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
%               end
%           end
%       end
% 
% 
%       k_mpc = k_mpc + 1;
%     end
% 
% 
%     %% Path planning
%     if k_c*config.dk_c <= k_pred
%       [agent_model] = model_fis(agent_model, environment_model.v_w, environment_model.ang_w, config, fisArray);
%       k_c = k_c + 1;
%     end
% 
%     %% Agent model
%     if k_a*config.dk_a <= k_pred
%           agent_model = model_agent(agent_model, environment_model.v_w, environment_model.ang_w, config.dt_a, config.k, config.dt_s);
%       k_a = k_a + 1;
%     end
% 
%     % V2 - Environment model - Use predicted states instead of recalculating
%     if k_e*config.dk_e <= k_pred
%       k_e = k_e + 1;
%     end
% 
%     %% Objective function evaluation
%     [s_obj_pred, ~] = calc_obj(...
%       config.weight, environment_model.m_f(k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
%       config.dt_s, config.s_obj, agent_model.c_f_s, config.t);
% 
%     %% Advance timestep
%     k_pred = k_pred + 1;
%     config.k      = config.k + 1;
% 
%   end
% end
