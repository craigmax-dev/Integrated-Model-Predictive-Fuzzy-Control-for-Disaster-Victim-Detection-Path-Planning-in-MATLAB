% TODO:
% - Modify function for flooding given altitude of each cell
% - 

% V1
function floodMap = initializeFloodMap(gridSize, floodSources, maxTime, spreadRate)
    % gridSize: A two-element vector [numRows, numCols]
    % floodSources: An array of points where flooding starts. Each row is [x, y]
    % maxTime: The maximum time for the simulation
    % spreadRate: Rate at which flooding spreads to neighboring cells

    numRows = gridSize(1);
    numCols = gridSize(2);
    floodMap = zeros(numRows, numCols, maxTime);

    % Initialize flooding sources for t=0
    for source = 1:size(floodSources, 1)
        floodMap(floodSources(source, 2), floodSources(source, 1), 1) = 1;
    end

    % Spread the flood over time
    for t = 2:maxTime
        for row = 1:numRows
            for col = 1:numCols
                % Get neighbors' flood levels
                neighbors = getNeighbors(floodMap(:, :, t-1), row, col);
                % Calculate new flood level based on neighbors and spread rate
                floodMap(row, col, t) = floodMap(row, col, t-1) + spreadRate * mean(neighbors);
                % Ensure the flood level is between 0 and 1
                floodMap(row, col, t) = min(floodMap(row, col, t), 1);
            end
        end
    end
end

function neighbors = getNeighbors(map, row, col)
    [numRows, numCols] = size(map);
    neighbors = [];
    for r = max(row-1, 1):min(row+1, numRows)
        for c = max(col-1, 1):min(col+1, numCols)
            if ~(r == row && c == col)
                neighbors(end+1) = map(r, c);
            end
        end
    end
end
