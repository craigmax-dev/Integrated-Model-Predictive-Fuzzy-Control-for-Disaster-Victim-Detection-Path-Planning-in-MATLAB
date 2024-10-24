
% V2.2 - STRUCTURES
function [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params, agentIndex)
    range = 1; % Initialize range at the beginning of parameter updates
    if nargin < 5
        agentIndex = []; % Empty means update all agents
    end
    
    if strcmp(mpc_model.architecture, 'mpfc')
        if strcmp(mpc_model.optimization_target, 'output')
            [fisArray, range] = updateFISOutputParameters(fisArray, params, range, agentIndex);
        elseif strcmp(mpc_model.optimization_target, 'input')
            [fisArray, range] = updateFISInputParameters(fisArray, params, range, agentIndex);
        else
            error('Invalid optimization target. Must be "input" or "output".');
        end
    elseif strcmp(mpc_model.architecture, 'mpc')
        [agent_model, range] = updateAgentTargets(agent_model, params, range, agentIndex);
    else
        error('Invalid architecture value. Must be "mpfc" or "mpc".');
    end
end

function [fisArray, range] = updateFISOutputParameters(fisArray, params, range, agentIndex)
    targetAgents = 1:numel(fisArray);
    if ~isempty(agentIndex)
        targetAgents = agentIndex; % Update only the specified agent's FIS
    end
    
    for a = targetAgents
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

function [agent_model, range] = updateAgentTargets(agent_model, params, range, agentIndex)
    targetAgents = 1:agent_model.n_a;
    if ~isempty(agentIndex)
        targetAgents = agentIndex; % Update only the specified agent's targets
    end
    
    for a = targetAgents

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