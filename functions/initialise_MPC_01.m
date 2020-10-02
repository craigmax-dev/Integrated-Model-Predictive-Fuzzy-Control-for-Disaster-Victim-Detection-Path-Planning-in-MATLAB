%% Function initialise_MPC
% Initialise MPC model

function [flag_mpc, solver, ...
  n_p, fis_params, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, ...
  nvars, fminsearchOptions, gaOptions, patOptions, parOptions, t_opt] ...
  = initialise_MPC_01(fisArray, n_a)
flag_mpc = false;
% Solver options: fmincon, ga, particleswarm, patternsearch
solver = "patternsearch";
% Prediction horizon
n_p = 1;
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
optTermCond       = 'MaxTime';
t_opt             = 60;
fminsearchOptions = optimset('Display','iter','PlotFcns',@optimplotfval);
gaOptions         = optimoptions('ga','Display','iter', 'PlotFcn', @gaplotbestf);
patOptions        = optimoptions('patternsearch','Display','iter', optTermCond, t_opt);
parOptions        = optimoptions('particleswarm','Display','iter', 'PlotFcn',@pswplotbestf);
end