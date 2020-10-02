%% Function initialise_pathPlanning
% Initialise path planning model

function [c_prior_building, c_prior_open, m_prior, fisArray] = initialise_pathPlanning_01(m_bo_s, n_a)
  % Priority map 
  c_prior_building  = 1;    % Priority constant for building
  c_prior_open      = 0.1;  % Priority constant for open space
  % Calculate Priority map
  m_prior = arrayfun(@(bo_search)(c_prior_building*bo_search + c_prior_open*(1-bo_search)), m_bo_s);
  % Generate FIS
  n_in            = 3;
  n_out           = 1;
  n_MF_in         = [3 3 3];
  n_MF_out        = [1];

  name_in         = ["t_response", "priority", "t_dw"];
  name_out        = ["attraction"];

  range_in        = [ 0 1; 
                      0 1; 
                      0 1];
  range_out       = [0 1];

  MF_shape_in     = ["trimf", "trimf", "trimf"];
  MF_shape_out    = ["linear"];
  for i=1:n_in
    MF_range_in(:,:,i) = [-0.1 1/4 2/4; 
                          1/4 2/4 3/4; 
                          2/4 3/4 1.1];
  end
  MF_range_out    = [-1 1 -1 1];
  MF_name_in      = [ "low", "medium", "high";
                      "low", "medium", "high";
                      "low", "medium", "high"];
                    
  MF_name_out     = ["output"];

  ruleList        = [ 1 1 1 1 1 1;
                      1 1 2 1 1 1;
                      1 1 3 1 1 1;
                      1 2 1 1 1 1;
                      1 2 2 1 1 1;
                      1 2 3 1 1 1;
                      1 3 1 1 1 1;
                      1 3 2 1 1 1;
                      1 3 3 1 1 1;
                      2 1 1 1 1 1;
                      2 1 2 1 1 1;
                      2 1 3 1 1 1;
                      2 2 1 1 1 1;
                      2 2 2 1 1 1;
                      2 2 3 1 1 1;
                      2 3 1 1 1 1;
                      2 3 2 1 1 1;
                      2 3 3 1 1 1;
                      3 1 1 1 1 1;
                      3 1 2 1 1 1;
                      3 1 3 1 1 1;
                      3 2 1 1 1 1;
                      3 2 2 1 1 1;
                      3 2 3 1 1 1;
                      3 3 1 1 1 1;
                      3 3 2 1 1 1;
                      3 3 3 1 1 1];

  %% Generate FIS
  for a=1:n_a
      % Create new sugfis object
      fis = sugfis;

      %% Inputs
      for i_in = 1:n_in
        fis = addInput (fis, range_in(i_in, :),'Name', name_in(i_in));
        for i = 1:n_MF_in(i_in)
          fis = addMF(fis, name_in(i_in), MF_shape_in(i_in), MF_range_in(i, :, i_in), 'Name', MF_name_in(i_in, i));
        end
      end

      %% Outputs
      for i_out = 1:n_out
        fis = addOutput (fis, range_out(i_out, :),'Name', name_out(i_out));
        for i = 1:n_MF_out(i_out)
          fis = addMF(fis, name_out(i_out), MF_shape_out(i_out), MF_range_out(i, :, i_out), 'Name', MF_name_out(i_out, i));
        end
      end

      %% Rule base
      fis = addRule(fis, ruleList);

      %% Add to FIS array
      fisArray(a) = fis;
  end
end