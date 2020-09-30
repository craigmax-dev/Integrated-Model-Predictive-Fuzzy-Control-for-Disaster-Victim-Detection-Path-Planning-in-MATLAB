%% Function initialise_MPC
% Initialise MPC model

function [n_p, fis_params, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, ...
  nvars, h_MPC, fminsearchOptions, gaOptions, patOptions, parOptions] ...
  = initialise_MPC()
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
% Function handle
h_MPC = @(params)model_MPC(params, ...
  fisArray, test_fis_sensitivity, ...
  m_f, m_s, m_bo, m_bt, m_scan, m_t_scan, ...
  dk_a, dk_c, dk_e, dk_mpc, dt_s, k, ...
  n_a, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
  a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
  l_x_s, l_y_s, c_f_s, ...
  c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
  r_bo, r_fo);
% Solver options
optTermCond       = 'MaxTime';
fminsearchOptions = optimset('Display','iter','PlotFcns',@optimplotfval);
gaOptions         = optimoptions('ga','Display','iter', 'PlotFcn', @gaplotbestf);
patOptions        = optimoptions('patternsearch','Display','iter', optTermCond, t_opt);
parOptions        = optimoptions('particleswarm','Display','iter', 'PlotFcn',@pswplotbestf);
end