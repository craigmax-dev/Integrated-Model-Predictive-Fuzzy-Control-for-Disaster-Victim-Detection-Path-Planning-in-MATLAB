% EXAMPLE USAGE
% % Define dimensions
% dims = [100, 100]; % 100x100 grid
% 
% % Define the Gaussian PDFs
% wideGaussian = @(x, y) exp(-((x-0.5).^2 + (y-0.25).^2) / (2 * 0.1^2));
% narrowGaussian = @(x, y) exp(-((x-0.5).^2 + (y-0.75).^2) / (2 * 0.01^2));
% 
% % Define PDFs and their weights
% pdfs = {wideGaussian, narrowGaussian};
% weights = [0.5, 0.5]; % Equal weighting for simplicity
% 
% % Generate the matrix
% matrix = generateMatrixFromPDFs(dims, pdfs, weights, seed);

function matrix = generateMatrixFromPDFs(dims, pdfs, weights)
    % Generates a 2D matrix based on a combination of specified 2D probability distribution functions
    % 
    % dims: number of rows and columns in the matrix
    % pdfs: set of pdfs to use in the matrix
    % weights: weighting of each pdf

    % Initialize the matrix
    matrix = zeros(dims(1), dims(2));

    % Ensure weights sum to 1
    weights = weights / sum(weights);

    % Generate a grid of points
    [X, Y] = meshgrid(linspace(0, 1, dims(2)), linspace(0, 1, dims(1)));

    % Generate matrix based on the PDFs
    for i = 1:length(pdfs)
        pdf = pdfs{i};
        weight = weights(i);

        % Evaluate the PDF at each point
        pdfVals = pdf(X, Y);

        % Handle constant PDFs
        if max(pdfVals(:)) == min(pdfVals(:))
            pdfVals = ones(size(pdfVals));
        else
            % Normalize PDF values to [0, 1]
            pdfVals = (pdfVals - min(pdfVals(:))) / (max(pdfVals(:)) - min(pdfVals(:)));
        end

        % Add weighted PDF to the matrix
        matrix = matrix + weight * pdfVals;
    end

    % Check if normalization is possible
    if max(matrix(:)) == min(matrix(:))
        matrix = ones(size(matrix));
    else
        % Normalize the final matrix to have values between 0 and 1
        matrix = (matrix - min(matrix(:))) / (max(matrix(:)) - min(matrix(:)));
    end

    
end


