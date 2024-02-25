% V2
function [fisArray, agent_model] = updateModel(mpc_model, fisArray, agent_model, params)
    range = 1; % Initialize range at the beginning of parameter updates
    if strcmp(mpc_model.architecture, 'mpfc')
        [fisArray, range] = updateFISParameters(fisArray, params, range);
    elseif strcmp(mpc_model.architecture, 'mpc')
        [agent_model, range] = updateAgentTargets(agent_model, params, range);
    else
        error('Invalid architecture value. Must be "mpfc" or "mpc".');
    end
    % Note: 'range' is not used after updates in this context but is crucial within each update function.
end

function [fisArray, range] = updateFISParameters(fisArray, params, range)
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