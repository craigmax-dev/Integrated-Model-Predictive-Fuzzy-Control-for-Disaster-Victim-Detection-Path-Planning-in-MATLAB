%% Function attCalc()
% Return attraction of cell

% V2

% CHANGELOG
% removed valid_cells calculation for now
% Fixed bug with m_dw changing dimensions
% added valid_cells calculation to exclude cells in queue
% Removed t_dw and m_scan from inputs
% - Feature: FIS input rarestoredefaultpathnge - compress inputs to predefined ranges. Add warning
% message. To decide if we are to keep.

% Refactor:
% Valid cells different for each agent

% TODO
%

% % V2.5
function m_att = calc_att(fis, fisInputs, a_target, communication_enabled)

    inputSize = size(fisInputs.t_response);

    % Initialize the attraction map
    m_att = NaN(inputSize);
    
    % Create a logical matrix indicating which cells are scheduled
    scheduled_cells = false(inputSize);
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

    % Identify cells that are not scheduled and do not contain NaN in any inputs
    valid_cells = ~scheduled_cells;
    inputFields = fieldnames(fisInputs);
    for i = 1:length(inputFields)
        valid_cells = valid_cells & ~isnan(fisInputs.(inputFields{i}));
    end

    % Check if there are any valid cells to process
    if ~any(valid_cells, 'all')
        fprintf("No valid cells in calc_att\n")
        return;  % Exit if no valid cells
    end

    % Prepare the inputs for FIS for valid cells
    numInputs = numel(fieldnames(fisInputs));
    fisInputArray = zeros(sum(valid_cells(:)), numInputs);
    for i = 1:numInputs
        fieldName = inputFields{i};
        fisInputArray(:, i) = fisInputs.(fieldName)(valid_cells);
    end

    % Vectorized FIS evaluation for valid cells
    m_att(valid_cells) = evalfis(fis, fisInputArray);
end

% % V2.4
% function m_att = calc_att(fis, m_t_response, m_prior, a_target)
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
%     fisInputs = [m_t_response(valid_cells), m_prior(valid_cells)];
% 
%     % Ensure inputs are of correct type
%     fisInputs = double(fisInputs);
% 
%     % Vectorized FIS evaluation for valid cells
%     m_att(valid_cells) = evalfis(fis, fisInputs);
% end
