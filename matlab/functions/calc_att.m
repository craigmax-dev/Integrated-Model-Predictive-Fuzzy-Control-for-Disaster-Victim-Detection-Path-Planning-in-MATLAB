%% Function attCalc()
% Return attraction of cell

% V2

% WRITEUP

% CHANGELOG
% removed valid_cells calculation for now
% Fixed bug with m_dw changing dimensions
% added valid_cells calculation to exclude cells in queue
% Removed t_dw and m_scan from inputs
% - Feature: FIS input rarestoredefaultpathnge - compress inputs to predefined ranges. Add warning
% message. To decide if we are to keep.

% V2.7 - bugfix for local maps 
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
                if x > 0 && y > 0 && x <= inputSize(1) && y <= inputSize(2)
                    scheduled_cells(x, y) = true;
                end
            end
        end
    else
        % If communication is disabled, consider only the agent's own scheduled cells
        for targetIdx = 1:size(a_target, 1)
            x = a_target(targetIdx, 1);
            y = a_target(targetIdx, 2);
            if x > 0 && y > 0 && x <= inputSize(1) && y <= inputSize(2)
                scheduled_cells(x, y) = true;
            end
        end
    end

    % Identify cells that are not scheduled and do not contain NaN in any inputs
    valid_cells = ~scheduled_cells;
    inputFields = fieldnames(fisInputs);
    for i = 1:length(inputFields)
        if all(size(fisInputs.(inputFields{i})) == size(valid_cells))
            valid_cells = valid_cells & ~isnan(fisInputs.(inputFields{i}));
        else
            fprintf("Size mismatch in input fields for calc_att\n");
            return;
        end
    end

    % Check if there are any valid cells to process
    if ~any(valid_cells, 'all')
        fprintf("No valid cells in calc_att\n");
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

    % Include recharge logic using recharge map
    if isfield(fisInputs, 'm_recharge') && (sum(fisInputs.m_recharge, 'all') > 0)
        [rechargeRows, rechargeCols] = find(fisInputs.m_recharge == 1);
        rechargePoints = sub2ind(inputSize, rechargeRows, rechargeCols);
        m_att(rechargePoints) = adjustRechargeAttraction(m_att(rechargePoints), fisInputs, rechargeRows, rechargeCols);
    end    
end 

function adjustedAttraction = adjustRechargeAttraction(attractions, fisInputs, rechargeRows, rechargeCols)
    % Calculate new attraction values for recharge points based on specific logic
    % Placeholder logic: increase the attraction significantly to simulate the recharge priority
    % Here we use the battery level and distance to recharge station as inputs to a simplified function
    batteryLevel = fisInputs.battery_level(sub2ind(size(fisInputs.battery_level), rechargeRows, rechargeCols));
    distanceToRecharge = sqrt((fisInputs.current_location_x - rechargeCols).^2 + (fisInputs.current_location_y - rechargeRows).^2);
    adjustedAttraction = attractions + batteryLevel .* max(1 - distanceToRecharge / max(distanceToRecharge), 0);
end


% % V2.6 - battery recharge
% % NOTE: Battery recharge model not implemented correctly in remaining code. 
% function m_att = calc_att(fis, fisInputs, a_target, communication_enabled)
% 
%     inputSize = size(fisInputs.t_response);
% 
%     % Initialize the attraction map
%     m_att = NaN(inputSize);
% 
%     % Create a logical matrix indicating which cells are scheduled
%     scheduled_cells = false(inputSize);
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
%     % Identify cells that are not scheduled and do not contain NaN in any inputs
%     valid_cells = ~scheduled_cells;
%     inputFields = fieldnames(fisInputs);
%     for i = 1:length(inputFields)
%         valid_cells = valid_cells & ~isnan(fisInputs.(inputFields{i}));
%     end
% 
%     % Check if there are any valid cells to process
%     if ~any(valid_cells, 'all')
%         fprintf("No valid cells in calc_att\n")
%         return;  % Exit if no valid cells
%     end
% 
%     % Prepare the inputs for FIS for valid cells
%     numInputs = numel(fieldnames(fisInputs));
%     fisInputArray = zeros(sum(valid_cells(:)), numInputs);
%     for i = 1:numInputs
%         fieldName = inputFields{i};
%         fisInputArray(:, i) = fisInputs.(fieldName)(valid_cells);
%     end
% 
%     % Vectorized FIS evaluation for valid cells
%     m_att(valid_cells) = evalfis(fis, fisInputArray);
% 
%     % Include recharge logic using recharge map
%     if exist('m_recharge', 'var') && (sum(m_recharge, 'all') > 0)
%         [rechargeRows, rechargeCols] = find(m_recharge == 1);
%         rechargePoints = sub2ind(inputSize, rechargeRows, rechargeCols);
%         m_att(rechargePoints) = adjustRechargeAttraction(m_att(rechargePoints), fisInputs, rechargeRows, rechargeCols);
%     end    
% end 
% 
% 
% function adjustedAttraction = adjustRechargeAttraction(attractions, fisInputs, rechargeRows, rechargeCols)
%     % Calculate new attraction values for recharge points based on specific logic
%     % Placeholder logic: increase the attraction significantly to simulate the recharge priority
%     % Here we use the battery level and distance to recharge station as inputs to a simplified function
%     batteryLevel = fisInputs.battery_level(sub2ind(size(fisInputs.battery_level), rechargeRows, rechargeCols));
%     distanceToRecharge = sqrt((fisInputs.current_location_x - rechargeCols).^2 + (fisInputs.current_location_y - rechargeRows).^2);
%     adjustedAttraction = attractions + batteryLevel .* max(1 - distanceToRecharge / max(distanceToRecharge), 0);
% end
