%% Function attCalc()
% Return attraction of cell

% V2

% CHANGELOG
% removed valid_cells calculation for now
% Fixed bug with m_dw changing dimensions
% added valid_cells calculation to exclude cells in queue
% Removed t_dw and m_scan from inputs
% - Feature: FIS input range - compress inputs to predefined ranges. Add warning
% message. To decide if we are to keep.

% TODO
% validate function - initial test shows huge performance improvements
% refactor valid_cells calculation? must allow rescan
% when removing cell retrictions - will keep scanning same cells. Optimize FIS?
% Fix bug - repeat in queue. Should not assign cell which is already assigned
% Add m_dw to attraction calculation? Or is this priority instead?
% Refactor: change how valid_cells is defined. If an agent runs out of battery
% we need to clear/reset parameters related to it. We also cannot consider plan
% by agents at higher level.
% Remove recalculation of n_x_s, n_y_s
% Print message when FIS inputs compressed?

% Refactor:
% Valid cells different for each agent

% TODO
% Refactor function - does not seem to work correctly. scheduled_cells is not the correct size 

% % V2.5 
% % Adjust to >2 FIS input parameters
% function m_att = calc_att(fis, m_t_response, m_prior, a_target, communication_enabled, distanceMatrix)
%     % Initialize the attraction map
%     m_att = NaN(size(m_t_response));
% 
%     % Create a logical matrix indicating which cells are scheduled
%     scheduled_cells = false(size(m_t_response));
%     if communication_enabled
%         % Communication enabled: consider all scheduled cells by all agents
%         for agentIdx = 1:size(a_target, 3)
%             for targetIdx = 1:size(a_target, 1)
%                 x = a_target(targetIdx, 1, agentIdx);
%                 y = a_target(targetIdx, 2, agentIdx);
%                 if x > 0 && y > 0 && x <= size(scheduled_cells,1) && y <= size(scheduled_cells,2)
%                     scheduled_cells(x, y) = true;
%                 end
%             end
%         end
%     else
%         % Communication disabled: consider only the agent's own scheduled cells
%         for targetIdx = 1:size(a_target, 1)
%             x = a_target(targetIdx, 1);
%             y = a_target(targetIdx, 2);
%             if x > 0 && y > 0 && x <= size(scheduled_cells,1) && y <= size(scheduled_cells,2)
%                 scheduled_cells(x, y) = true;
%             end
%         end
%     end
% 
%     % Identify valid cells for processing
%     valid_cells = ~scheduled_cells & ~isnan(m_t_response) & ~isnan(m_prior) & ~isnan(distanceMatrix);
% 
%     % Exit if no valid cells
%     if ~any(valid_cells, 'all')
%         fprintf("No valid cells in calc_att\n");
%         return;
%     end
% 
%     % Prepare the inputs for FIS based on available FIS inputs
%     fisInputs = getFisInputs(fis);
%     fisInputValues = [];
% 
%     % Flatten fisInputs for direct string comparison
%     fisInputsFlat = cellfun(@(c) c{1}, fisInputs, 'UniformOutput', false);
% 
% 
%     if ismember('t_response', fisInputsFlat)
%         fisInputValues = [fisInputValues, m_t_response(valid_cells)];
%     end
%     if ismember('priority', fisInputsFlat)
%         fisInputValues = [fisInputValues, m_prior(valid_cells)];
%     end
%     if ismember('r_nextagent', fisInputsFlat)
%         fisInputValues = [fisInputValues, distanceMatrix(valid_cells)];
%     end
% 
%     % Ensure inputs are of correct type
%     fisInputValues = double(fisInputValues);
% 
%     % Calculate the number of valid input sets
%     numValidInputs = nnz(valid_cells); % Use nnz for counting non-zero (true) elements in valid_cells
% 
%     % Reshape fisInputValues for evalfis, ensuring integer dimensions
%     if numValidInputs > 0
%         % Only proceed if there are valid inputs to evaluate
%         fisInputMatrix = reshape(fisInputValues, numValidInputs, []); % Adjust dimensions based on actual valid inputs
% 
%         % Vectorized FIS evaluation for valid cells
%         m_att(valid_cells) = evalfis(fisInputMatrix, fis);
%     else
%         fprintf('No valid cells to evaluate.\n');
%     end
% end
% 
% function fisInputs = getFisInputs(fis)
%     % Extracts the names of inputs defined in the FIS
%     fisInputs = arrayfun(@(input) input.Name, fis.Inputs, 'UniformOutput', false);
% end

% % V2.41
% function m_att = calc_att(fis, m_t_response, m_prior, a_target, communication_enabled)
%     % Initialize the attraction map
%     m_att = NaN(size(m_t_response));
% 
%     % Create a logical matrix indicating which cells are scheduled
%     scheduled_cells = false(size(m_t_response)); % Initialize with all false
%     if communication_enabled
%         % If communication is enabled, consider all scheduled cells by all agents
%         for agentIdx = 1:size(a_target, 3)
%             for targetIdx = 1:size(a_target, 1)
%                 x = a_target(targetIdx, 1, agentIdx);
%                 y = a_target(targetIdx, 2, agentIdx);
%                 if x > 0 && y > 0
%                     scheduled_cells(x, y) = true;
%                 end
%             end
%         end
%     else
%         % If communication is disabled, consider only the agent's own scheduled cells
%         for targetIdx = 1:size(a_target, 1)
%             x = a_target(targetIdx, 1);
%             y = a_target(targetIdx, 2);
%             if x > 0 && y > 0
%                 scheduled_cells(x, y) = true;
%             end
%         end
%     end
% 
%     % Identify cells that are not scheduled and do not contain NaN in inputs
%     valid_cells = ~scheduled_cells & ~isnan(m_t_response) & ~isnan(m_prior);
% 
%     % Check if there are any valid cells to process
%     if ~any(valid_cells, 'all')
%         fprintf("No valid cells in calc_att\n")
%         return;  % Exit if no valid cells
%     end
% 
%     % Prepare the inputs for FIS for valid cells
%     t_response_valid = m_t_response(valid_cells);
%     prior_valid = m_prior(valid_cells);
% 
%     % Compress t_response_valid based on FIS input range
%     t_response_range = fis.Inputs(1).Range;
%     t_response_valid = max(t_response_range(1), min(t_response_range(2), t_response_valid));
% 
%     % Compress prior_valid based on FIS input range
%     prior_range = fis.Inputs(2).Range;
%     prior_valid = max(prior_range(1), min(prior_range(2), prior_valid));
% 
%     % Combine the compressed inputs
%     fisInputs = [t_response_valid, prior_valid];
% 
%     % Ensure inputs are of correct type
%     fisInputs = double(fisInputs);
% 
%     % Vectorized FIS evaluation for valid cells
%     m_att_temp = evalfis(fis, fisInputs);
% 
%     % Assign the evaluated FIS output to the valid cells in m_att
%     m_att(valid_cells) = m_att_temp;
% end


% V2.4
function m_att = calc_att(fis, m_t_response, m_prior, a_target, communication_enabled)
    % Initialize the attraction map
    m_att = NaN(size(m_t_response));

    % Create a logical matrix indicating which cells are scheduled
    scheduled_cells = false(size(m_t_response)); % Initialize with all false
    if communication_enabled
        % If communication is enabled, consider all scheduled cells by all agents
        for agentIdx = 1:size(a_target, 3)
            for targetIdx = 1:size(a_target, 1)
                x = a_target(targetIdx, 1, agentIdx);
                y = a_target(targetIdx, 2, agentIdx);
                if x > 0 && y > 0
                    scheduled_cells(x, y) = true;
                end
            end
        end
    else
        % If communication is disabled, consider only the agent's own scheduled cells
        for targetIdx = 1:size(a_target, 1)
            x = a_target(targetIdx, 1);
            y = a_target(targetIdx, 2);
            if x > 0 && y > 0
                scheduled_cells(x, y) = true;
            end
        end
    end

    % Identify cells that are not scheduled and do not contain NaN in inputs
    valid_cells = ~scheduled_cells & ~isnan(m_t_response) & ~isnan(m_prior);

    % Check if there are any valid cells to process
    if ~any(valid_cells, 'all')
        fprintf("No valid cells in calc_att\n")
        return;  % Exit if no valid cells
    end

    % Prepare the inputs for FIS for valid cells
    fisInputs = [m_t_response(valid_cells), m_prior(valid_cells)];

    % Ensure inputs are of correct type
    fisInputs = double(fisInputs);

    % Vectorized FIS evaluation for valid cells
    m_att(valid_cells) = evalfis(fis, fisInputs);
end
