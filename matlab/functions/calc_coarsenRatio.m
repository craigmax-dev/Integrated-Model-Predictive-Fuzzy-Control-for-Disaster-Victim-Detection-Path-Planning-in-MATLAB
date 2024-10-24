% %% coarsenRatio.m
% % For a given desired cell length in a tif object, return the required 
% % coarsen ratio and resulting cell lengths in both axes in order to closest
% % match that cell length.
% 
% % TEST REFACTOR - V2
% function [c_f, l_x, l_y] = calc_coarsenRatio(matrixSize, l_d)
%     % matrixSize is a 2-element vector [numRows, numCols]
%     % l_d is the desired cell length
% 
%     % Extract number of rows and columns from matrixSize
%     numRows = matrixSize(1);
%     numCols = matrixSize(2);
% 
%     % Calculate the number of cells in each dimension to match the desired cell length
%     n_rows = round(l_d);
%     n_cols = round(l_d);
% 
%     % Coarsen factor for both dimensions
%     c_f = [n_rows, n_cols];
% 
%     % Calculate the actual length of new cells in x and y directions
%     l_x = numRows / n_rows;
%     l_y = numCols / n_cols;
% 
%     % Check if the coarsen ratio is valid
%     if n_rows < 2 || n_cols < 2
%         error("Coarsen ratio not high enough");
%     end
% end