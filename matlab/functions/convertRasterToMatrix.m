% V2: NOT TESTED YET

function [matrixData, additionalVariables] = convertRasterToMatrix(rasterFile)
    % Function to convert raster data to a matrix
    % Inputs:
    %   rasterFile - String, path to the raster file
    % Outputs:
    %   matrixData - Matrix containing the raster data
    %   additionalVariables - Struct with other relevant variables

    % Initialize a struct to hold additional variables
    additionalVariables = struct();

    % Read the raster file
    [matrixData, rasterRef] = geotiffread(rasterFile);

    % Store relevant variables in the struct
    % Example: Save latitude and longitude limits
    additionalVariables.LatitudeLimits = rasterRef.LatitudeLimits;
    additionalVariables.LongitudeLimits = rasterRef.LongitudeLimits;

    % Include other relevant variables from rasterRef as needed
    % Example: Cell size
    additionalVariables.CellSize = [rasterRef.CellExtentInLatitude, rasterRef.CellExtentInLongitude];

    % Include any other relevant information you need
end
