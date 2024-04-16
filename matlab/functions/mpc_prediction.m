%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

% V2

% CHANGELOG
% Removed flag_mpc
% Refactor: Structures
% Feature: Controller architectures
% Refactor: Modularisation

% NOTES
% Current implementation will not work for multiple prediction steps (which could be beneficial for MPC)

% TO DO
% - check and fix environment model

% % V2.3
function s_obj_pred = mpc_prediction(params, agent_model, config, mpc_environment_model, fisArray, mpc_model, agentIndex)

  %% Variables
  % Counters
  k_pred  = 0;
  k_a     = 0;
  k_c     = 0;
  k_e     = 0;
  k_mpc   = 0;
  s_obj_pred   = 0;

  %% Prediction
  while k_pred < config.dk_pred*mpc_model.n_p

    if k_mpc*config.dk_pred <= k_pred

      % Update model based on architecture and structure
      if strcmp(mpc_model.structure, 'decentralised')
          % For decentralised, update parameters for the specific agent
          [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params, agentIndex);
      else
          % For centralised or if agentIndex is not provided, update parameters globally
          [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params);
      end

      k_mpc = k_mpc + 1;

    end

    %% FIS Path planning
    if ~strcmp(mpc_model.architecture, 'mpc')
      if k_c*config.dk_c <= k_pred
        [agent_model] = model_fis(agent_model, mpc_environment_model.ang_w, mpc_environment_model.v_w, config, fisArray);
        k_c = k_c + 1;
      end
    end

    %% Agent model
    if k_a*config.dk_a <= k_pred
          agent_model = model_agent(agent_model, mpc_environment_model.v_w, mpc_environment_model.ang_w, mpc_environment_model.m_f_series(:, :, k_e + 1), mpc_environment_model.m_dw_e_series(:, :, k_e + 1), config);
      k_a = k_a + 1;
    end

    % V2 - Environment model - Use predicted states instead of recalculating
    if k_e*config.dk_e <= k_pred
      k_e = k_e + 1;
    end

    %% Objective function evaluation
    % % V1
    % [s_obj_pred, ~] = calc_obj(...
    %   config.weight, mpc_environment_model.m_f(k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
    %   config.dt_s, config.s_obj, config.c_f_s, config.t);

    % V2 REFACTOR
    [s_obj_pred, ~] = calc_obj_v2(...
      config.weight, mpc_environment_model.m_f_series(:, :, k_e + 1), agent_model.m_bo_s, agent_model.m_scan, agent_model.m_victim_s, ...
      config.dt_s, config.s_obj, config.c_f_s);

    %% Advance timestep
    k_pred = k_pred + 1;
    config.k      = config.k + 1;
  end
end
