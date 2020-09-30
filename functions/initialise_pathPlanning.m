%% Function initialise_pathPlanning
% Initialise path planning model

function [c_prior_building, c_prior_open, m_prior, fisArray] = initialise_pathPlanning()
  % Priority map 
  c_prior_building  = 1;    % Priority constant for building
  c_prior_open      = 0.1;  % Priority constant for open space
  % Calculate Priority map
  m_prior = arrayfun(@(bo_search)(c_prior_building*bo_search + c_prior_open*(1-bo_search)), m_bo_s);
  % Generate FIS
  [fisArray] = initialise_FIS( n_a );
end