% V2
function [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params)
    range = 1; % Initialize range at the beginning of parameter updates
    if strcmp(mpc_model.architecture, 'mpfc')
        if strcmp(mpc_model.optimization_target, 'output')
            [fisArray, range] = updateFISOutputParameters(fisArray, params, range);
        elseif strcmp(mpc_model.optimization_target, 'input')
            [fisArray, range] = updateFISInputParameters(fisArray, params, range);
        else
            error('Invalid optimization target. Must be "input" or "output".');
        end
    elseif strcmp(mpc_model.architecture, 'mpc')
        [agent_model, range] = updateAgentTargets(agent_model, params, range);
    else
        error('Invalid architecture value. Must be "mpfc" or "mpc".');
    end
end

% function [fisArray, range] = updateFISInputParameters(fisArray, params, range)
%     for a = 1:numel(fisArray)
%         for o = 1:numel(fisArray(a).Outputs)
%             for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
%                 numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
%                 if range + numParams - 1 <= length(params)
%                     newParams = params(range:range+numParams-1);
%                     fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
%                     range = range + numParams;
%                 else
%                     error('updateFISParameters:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
%                 end
%             end
%         end
%     end
% end


function [fisArray, range] = updateFISInputParameters(fisArray, params, range)
    for a = 1:numel(fisArray)
        for i = 1:numel(fisArray(a).Inputs)
            for mf = 1:numel(fisArray(a).Inputs(i).MembershipFunctions)
                numParams = numel(fisArray(a).Inputs(i).MembershipFunctions(mf).Parameters);
                if range + numParams - 1 <= length(params)
                    newParams = params(range:range+numParams-1);
                    % Ensure the parameters are in the correct order for 'trimf' and 'trapmf'
                    if strcmp(fisArray(a).Inputs(i).MembershipFunctions(mf).Type, 'trimf') || ...
                       strcmp(fisArray(a).Inputs(i).MembershipFunctions(mf).Type, 'trapmf')
                        newParams = sort(newParams); % Ensure ascending order
                    end
                    fisArray(a).Inputs(i).MembershipFunctions(mf).Parameters = newParams;
                    range = range + numParams;
                else
                    error('updateFISInputParameters:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
                end
            end
        end
    end
end


% function [fisArray, range] = updateFISInputParameters(fisArray, params, range)
%     % Iterates over each FIS in the array to update input MF parameters
%     for a = 1:numel(fisArray)
%         % Iterate over each input in the FIS
%         for i = 1:numel(fisArray(a).Inputs)
%             % Iterate over each membership function in the input
%             for mf = 1:numel(fisArray(a).Inputs(i).MembershipFunctions)
%                 % Extract the number of parameters for the current MF
%                 numParams = numel(fisArray(a).Inputs(i).MembershipFunctions(mf).Parameters);
% 
%                 % Check if the current range of parameters does not exceed the total number of optimization parameters
%                 if range + numParams - 1 <= length(params)
%                     % Extract the new parameters for the current MF from the optimization parameters
%                     newParams = params(range:range + numParams - 1);
% 
%                     % Update the MF parameters in the FIS
%                     fisArray(a).Inputs(i).MembershipFunctions(mf).Parameters = newParams;
% 
%                     % Update the range index for the next MF parameters
%                     range = range + numParams;
%                 else
%                     % If the range exceeds the length of params, throw an error
%                     error('updateFISInputParameters:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
%                 end
%             end
%         end
%     end
% end

function [fisArray, range] = updateFISOutputParameters(fisArray, params, range)
    % Iterates over each FIS in the array to update output MF parameters
    for a = 1:numel(fisArray)
        % Iterate over each output in the FIS
        for o = 1:numel(fisArray(a).Outputs)
            % Iterate over each membership function in the output
            for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
                % Extract the number of parameters for the current MF
                numParams = numel(fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters);
                
                % Check if the current range of parameters does not exceed the total number of optimization parameters
                if range + numParams - 1 <= length(params)
                    % Extract the new parameters for the current MF from the optimization parameters
                    newParams = params(range:range + numParams - 1);
                    
                    % Update the MF parameters in the FIS
                    fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters = newParams;
                    
                    % Update the range index for the next MF parameters
                    range = range + numParams;
                else
                    % If the range exceeds the length of params, throw an error
                    error('updateFISOutputParameters:IndexOutOfBounds', 'Attempting to access beyond the length of params.');
                end
            end
        end
    end
end


function [agent_model, range] = updateAgentTargets(agent_model, params, range)
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