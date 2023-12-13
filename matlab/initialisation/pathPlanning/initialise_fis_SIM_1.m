%% Function initialise_pathPlanning
% Initialise path planning model

% V2
% CHANGELOG
% - refactor: correct rules, add linear output functions

% TODO
% - redefine priority map - instead 
% - check range of input parameters suitable (restrict to range 0-1)?
% - check how n_MF_out is used - may need to refactor MPC function now

function [c_prior_building, c_prior_open, m_prior, fisArray, n_MF_out] = initialise_fis_SIM_1(m_bo_s, n_a)
  
% Priority map 
  c_prior_building  = 1;    % Priority constant for building
  c_prior_open      = 0.1;  % Priority constant for open space
  
  % Calculate Priority map
  m_prior = arrayfun(@(bo_search)(c_prior_building*bo_search + c_prior_open*(1-bo_search)), m_bo_s);
                   
  
  %% V2 refactor
  
  inputs = ["t_response", "priority", "t_dw"];
  mfTypes = ["trimf", "trimf", "trimf"];  % Using triangular MFs for simplicity
  mfNames = ["low", "medium", "high"];
  mfParams = [-0.1 0.5 1.1; -0.1 0.5 1.1; -0.1 0.5 1.1];  % Example parameters
  
  %% V2 REFACTOR
  %% Generate FIS
  for a = 1:n_a
    
    fis = sugfis;
    
    for i = 1:numel(inputs)
      fis = addInput(fis, [0 1], 'Name', inputs(i));
      for j = 1:numel(mfNames)
        fis = addMF(fis, inputs(i), mfTypes(i), mfParams(j, :), 'Name', mfNames(j));
      end
    end

    outputName = "attraction";
    fis = addOutput(fis, [0 1], 'Name', outputName);

    % Output MFs for low, medium, high
    % Assuming you want these outputs to be constants, you can set the coefficients for inputs to 0
    fis = addMF(fis, outputName, 'linear', [0 0 0 0], 'Name', 'low');  % Low output
    fis = addMF(fis, outputName, 'linear', [0 0 0 0.5], 'Name', 'medium');  % Medium output
    fis = addMF(fis, outputName, 'linear', [0 0 0 1], 'Name', 'high');  % High output

    ruleList = [];
    for tResp = 1:3  % For each t_response MF
      for priority = 1:3  % For each priority MF
        for tDw = 1:3  % For each t_dw MF
          rule = [tResp priority tDw priority 1 1];  % Output is the same as priority
          ruleList = [ruleList; rule];
        end
      end
    end

    fis = addRule(fis, ruleList);

    %% Add to FIS array
    fisArray(a) = fis;
  end

end

% V1
%   % Generate FIS
%   n_in            = 3;
%   n_out           = 1;
%   n_MF_in         = [3 3 3];
%   n_MF_out        = [1];
% 
%   name_in         = ["t_response", "priority", "t_dw"];
%   name_out        = ["attraction"];
% 
%   range_in        = [ 0 1; 
%                       0 1; 
%                       0 1];
%   range_out       = [0 1];
% 
%   MF_shape_in     = ["trimf", "trimf", "trimf"];
%   
%   MF_shape_out    = ["linear"];
% %   MF_shape_out    = ["trimf", "trimf", "trimf"];
% 
%   for i=1:n_in
%     MF_range_in(:,:,i) = [-0.1 1/4 2/4; 
%                           1/4 2/4 3/4; 
%                           2/4 3/4 1.1];
%   end
%   MF_range_out    = [-1 1 -1 1];
% %   MF_range_out    = [-1 0 1; 0 1 2; 1 2 3];  % Example ranges for low, medium, high
%   
%   MF_name_in      = [ "low", "medium", "high";
%                       "low", "medium", "high";
%                       "low", "medium", "high"];
%                     
% %   MF_name_out     = ["low", "medium", "high"];
%   MF_name_out     = ["output"];
% 
% %   ruleList        = [ 1 1 1 1 1 1;
% %                       1 1 2 1 1 1;
% %                       1 1 3 1 1 1;
% %                       1 2 1 2 1 1;
% %                       1 2 2 2 1 1;
% %                       1 2 3 2 1 1;
% %                       1 3 1 3 1 1;
% %                       1 3 2 3 1 1;
% %                       1 3 3 3 1 1;
% %                       2 1 1 1 1 1;
% %                       2 1 2 1 1 1;
% %                       2 1 3 1 1 1;
% %                       2 2 1 2 1 1;
% %                       2 2 2 2 1 1;
% %                       2 2 3 2 1 1;
% %                       2 3 1 3 1 1;
% %                       2 3 2 3 1 1;
% %                       2 3 3 3 1 1;
% %                       3 1 1 1 1 1;
% %                       3 1 2 1 1 1;
% %                       3 1 3 1 1 1;
% %                       3 2 1 2 1 1;
% %                       3 2 2 2 1 1;
% %                       3 2 3 2 1 1;
% %                       3 3 1 3 1 1;
% %                       3 3 2 3 1 1;
% %                       3 3 3 3 1 1];
    
% %% Generate FIS
%   for a=1:n_a
% 
%     % Create new sugfis object
%     fis = sugfis;
% 
%     %% Inputs
%     for i_in = 1:n_in
%       fis = addInput (fis, range_in(i_in, :),'Name', name_in(i_in));
%       for i = 1:n_MF_in(i_in)
%         fis = addMF(fis, name_in(i_in), MF_shape_in(i_in), MF_range_in(i, :, i_in), 'Name', MF_name_in(i_in, i));
%       end
%     end
% 
%     %% Outputs
%     for i_out = 1:n_out
%       fis = addOutput (fis, range_out(i_out, :),'Name', name_out(i_out));
%       for i = 1:n_MF_out(i_out)
%         fis = addMF(fis, name_out(i_out), MF_shape_out(i_out), MF_range_out(i, :, i_out), 'Name', MF_name_out(i_out, i));
%       end
%     end
% 
%     %% Rule base
%     fis = addRule(fis, ruleList);
% 
%     %% Add to FIS array
%     fisArray(a) = fis;
%   end