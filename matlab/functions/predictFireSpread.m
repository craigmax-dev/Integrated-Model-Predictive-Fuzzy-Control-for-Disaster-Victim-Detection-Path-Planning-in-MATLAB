% TODO
% - validate function
% - add optional flag to use this function vs other options

function F_likelihood = predictFireSpread(environment_model, prediction_horizon, dt_e, mpc_model)
    % Initialize the likelihood map with the appropriate size
    F_likelihood = zeros(environment_model.n_x_e, environment_model.n_y_e);
    
    % Choose prediction method based on mpc_model.prediction_mode
    switch mpc_model.prediction_mode
        case 'deterministic'
            % Deterministic fire spread prediction
            F_likelihood = deterministicFireSpreadPrediction(environment_model, prediction_horizon, dt_e);
            
        case 'probabilistic'
            % Probabilistic prediction of fire spread
            F_likelihood = probabilisticFireSpreadPrediction(environment_model, prediction_horizon, dt_e);
            
        case 'downwind_approximation'
            % Simple model using downwind map
            F_likelihood = downwindMapPrediction(environment_model, prediction_horizon, dt_e);
            
        otherwise
            error('Invalid prediction mode: %s', mpc_model.prediction_mode);
    end
end


% function F_likelihood = predictFireSpreadLikelihood(environment_model, prediction_horizon, dt_e)
%     % Initialize the fire spread likelihood map with zeros
%     F_likelihood = zeros(environment_model.n_x_e, environment_model.n_y_e);
% 
%     % Temporary environment model to manipulate during prediction
%     temp_environment = environment_model;
% 
%     % Iterate over the prediction horizon in steps of dt_e
%     for t = 0:dt_e:prediction_horizon
%         % Calculate wind spread matrix for current conditions
%         W = calculateWindSpreadMatrix(temp_environment.r_w, temp_environment.c_wm_1, temp_environment.c_wm_2, temp_environment.c_wm_d, temp_environment.ang_w, temp_environment.v_w);
% 
%         % Initialize a temporary fire spread probability map for this step
%         F_temp = zeros(temp_environment.n_x_e, temp_environment.n_y_e);
% 
%         % Update fire states and probabilities without changing the actual environment model
%         [~, F_temp] = updateFireStatesAndProbabilities(temp_environment, F_temp, W, dt_e);
% 
%         % Update the likelihood map - aggregate probabilities over time
%         F_likelihood = F_likelihood + (1 - F_likelihood) .* F_temp;
% 
%         % Apply a simplified version of fire spread for prediction purposes
%         % This is a probabilistic application without altering fire states
%         for i = 1:temp_environment.n_x_e
%             for j = 1:temp_environment.n_y_e
%                 if temp_environment.m_f(i, j) == 1 % If the cell is unburned
%                     % Update likelihood based on the aggregated probability
%                     % Note: This step is simplified and does not simulate exact fire spread,
%                     % but accumulates the likelihood of spread over time.
%                 end
%             end
%         end
%     end
% 
%     % Normalize the likelihood map to ensure values are within [0, 1]
%     F_likelihood(F_likelihood > 1) = 1;
% end
