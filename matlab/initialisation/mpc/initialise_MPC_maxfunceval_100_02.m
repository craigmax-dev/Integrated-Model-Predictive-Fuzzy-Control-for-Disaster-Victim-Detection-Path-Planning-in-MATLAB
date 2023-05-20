%% Function initialise_MPC
% Initialise MPC model

function [flag_mpc, solver, options, n_p, fis_params, ini_params, ...
  A, b, Aeq, beq, lb, ub, nonlcon, nvars] ...
  = initialise_MPC_maxfunceval_100_02(fisArray, n_a)
flag_mpc = true;
% Solver options: fmincon, ga, particleswarm, patternsearch
solver = "patternsearch";
% Prediction horizon
n_p = 2;
% Initialise optimisation parameters
fis_params = [];
for a = 1:n_a
  fis_params = [fis_params, fisArray(a).Outputs.MembershipFunctions.Parameters];
end
ini_params = [];
for i = 1:n_p
  ini_params = [ini_params, fis_params];
end
% Constraints
A       = [];
b       = [];
Aeq     = [];
beq     = [];
lb      = [];
ub      = [];
nonlcon = [];
nvars = size(ini_params, 2);
% Solver options
optTermCond       = 'MaxFunctionEvaluations';
optTermCond_value = 100;
options        = optimoptions('patternsearch','Display','iter', optTermCond, optTermCond_value);
end