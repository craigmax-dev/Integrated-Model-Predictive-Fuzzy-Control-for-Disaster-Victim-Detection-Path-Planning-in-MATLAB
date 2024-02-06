% TODO: 
% - Fix fire model
% - change dt_e
% - validate function
% - implement in mpc model

function F_likelihood = calculateFireSpreadLikelihood(environment_model, config, time_period)
    % Initialize the likelihood map with zeros
    F_likelihood = zeros(environment_model.n_x_e, environment_model.n_y_e);
    
    % Wind influence matrix
    W = calculateWindSpreadMatrix(environment_model.r_w, environment_model.c_wm_1, environment_model.c_wm_2, environment_model.c_wm_d, environment_model.ang_w, environment_model.v_w);

    % Assume that the entire burn time window is possible for each cell
    % within the given time period
    for t = 0:config.dt_e:time_period
        % Iterate over each cell
        for i = 1:environment_model.n_x_e
            for j = 1:environment_model.n_y_e
                % Skip cells that are already burnt or burning
                if environment_model.m_f(i, j) == 3 || environment_model.m_f(i, j) == 4
                    continue;
                end

                % Calculate spread probability for current time
                p = calculateSpreadProbability(t, environment_model.t_i, environment_model.t_b);
                % Update fire spread probability for current cell
                F_likelihood = updateFireSpreadProbability(F_likelihood, W, p, i, j, environment_model.c_fs_1, environment_model.c_fs_2, environment_model.m_s, environment_model.m_bo, environment_model.n_x_e, environment_model.n_y_e, environment_model.r_w);
            end
        end
    end
    
    % Normalize the likelihood map to represent probability
    F_likelihood(F_likelihood > 1) = 1;
end

% NOTE: code for validation
% % Calculate the likelihood of fire spread over a given time period
% time_period = 300; % Define time period for prediction
% F_likelihood = calculateFireSpreadLikelihood(environment_model, config, time_period);
% 
% % Update environment model to get actual fire spread (for comparison)
% for t = 1:time_period/config.dt_e
%     environment_model = model_environment(environment_model);
% end
% 
% % Plot likelihood estimation vs actual fire spread
% figure;
% subplot(1,2,1);
% imagesc(F_likelihood);
% title('Likelihood of Fire Spread');
% colorbar;
% 
% subplot(1,2,2);
% imagesc(environment_model.m_f);
% title('Actual Fire Spread');
% colorbar;