% NOTE: Test function for building population grid
% TODO: remove plotting function

function brownNoiseMatrix = generateBrownNoiseMatrix(rows, cols, fluctuation, seed)
    % Generates a 2D matrix of brown noise
    % 
    % rows: number of rows in the matrix
    % cols: number of columns in the matrix
    % fluctuation: maximum change between adjacent elements
    % seed: seed for the random number generator to ensure repeatability

    % Set the seed for repeatability
    rng(seed);

    % Initialize the matrix with random values
    brownNoiseMatrix = rand(rows, cols);

    % Iteratively adjust values to create Brown noise characteristics
    for i = 1:rows
        for j = 1:cols
            % Get neighboring indices while avoiding out-of-bounds indices
            rowIndices = max(1, i - 1):min(rows, i + 1);
            colIndices = max(1, j - 1):min(cols, j + 1);
            
            % Extract the neighboring elements
            neighbors = brownNoiseMatrix(rowIndices, colIndices);

            % Calculate the average of adjacent elements
            avgNeighbors = mean(neighbors, 'all');

            % Adjust the current element based on the average and a random fluctuation
            brownNoiseMatrix(i, j) = avgNeighbors + (rand() - 0.5) * fluctuation;

            % Ensure the value stays within the 0-1 range
            brownNoiseMatrix(i, j) = min(max(brownNoiseMatrix(i, j), 0), 1);
        end
    end

    % Normalize the matrix to have values between 0 and 1
    brownNoiseMatrix = (brownNoiseMatrix - min(brownNoiseMatrix(:))) / (max(brownNoiseMatrix(:)) - min(brownNoiseMatrix(:)));

end