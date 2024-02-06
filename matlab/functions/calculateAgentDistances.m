% Return map of distance of each cell from an agent other than the current agent

% TO DO
% - consider whether a map or single parameter is preferable

function distanceMatrix = calculateAgentDistances(gridSize, agentPositions)
    % gridSize is a two-element vector [numRows, numCols]
    % agentPositions is an n-by-2 matrix, where n is the number of agents,
    % and each row represents the (x, y) coordinates of an agent.

    numRows = gridSize(1);
    numCols = gridSize(2);
    numAgents = size(agentPositions, 1);
    
    % Initialize the distance matrix
    distanceMatrix = cell(numAgents, 1);
    
    for a = 1:numAgents
        % Create a matrix for the current agent
        distanceMatrix{a} = inf(numRows, numCols);
        
        for row = 1:numRows
            for col = 1:numCols
                % Calculate the Euclidean distance from the cell to all other agents
                for otherAgent = 1:numAgents
                    if otherAgent ~= a
                        distToOtherAgent = sqrt((col - agentPositions(otherAgent, 1))^2 + (row - agentPositions(otherAgent, 2))^2);
                        distanceMatrix{a}(row, col) = min(distanceMatrix{a}(row, col), distToOtherAgent);
                    end
                end
            end
        end
    end
end
