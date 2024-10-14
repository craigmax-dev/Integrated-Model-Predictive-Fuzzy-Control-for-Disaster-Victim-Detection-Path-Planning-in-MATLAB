% V2
function victimsMap = generateVictimsMap(probMatrix, n_max)
    % Generate a map of victims based on the probability matrix
    % probMatrix: Matrix of probabilities (0 to 1)
    % n_max: Maximum number of victims in each cell
    
    % Initialize the victims map with the same size as the probability matrix
    victimsMap = zeros(size(probMatrix));
    
    % Iterate through each cell in the probability matrix
    for i = 1:size(probMatrix, 1)
        for j = 1:size(probMatrix, 2)
            % Initialize victim count for the cell
            victimCount = 0;
            
            % Check each potential victim slot in the cell
            for v = 1:n_max
                if rand() < probMatrix(i, j)
                    % Increment the victim count if the random number is less than the probability
                    victimCount = victimCount + 1;
                end
            end
            
            % Assign the total victim count to the cell
            victimsMap(i, j) = victimCount;
        end
    end
end
