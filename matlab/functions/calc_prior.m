function prior = calc_prior(c_prior_building, c_prior_open, bo_search)
  prior =  c_prior_building*bo_search + c_prior_open*(1-bo_search);
end