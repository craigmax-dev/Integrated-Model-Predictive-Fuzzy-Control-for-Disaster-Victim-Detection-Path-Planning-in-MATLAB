from scipy.optimize import minimize
from scipy.optimize import LinearConstraint, NonlinearConstraint
from scipy.optimize import differential_evolution

def model_MPC_module(fisArray, ini_params, fis_param_hist, solver, options, n_a, n_MF_out,
                     nvars, A, b, Aeq, beq, lb, ub, nonlcon, test_fis_sensitivity, 
                     m_f, m_bo, m_bt, m_s, m_scan, m_t_scan, dk_a, dk_c, dk_e, dk_mpc, 
                     dt_s, k, seed, n_p, n_x_s, n_y_s, n_x_f, n_y_f, n_q, a_loc, a_target, 
                     a_task, a_t_trav, a_t_scan, l_x_s, l_y_s, c_f_s, c_fs_1, c_fs_2, v_as, 
                     v_w, ang_w, r_bo, r_fo, fis_data):
  
    # Function handle
    h_MPC = lambda params: func_MPC_model(params, fisArray, test_fis_sensitivity, 
                                         m_f, m_bo, m_bt, m_s, m_scan, m_t_scan, dk_a, dk_c, 
                                         dk_e, dk_mpc, dt_s, k, seed, n_a, n_MF_out, n_p, 
                                         n_x_s, n_y_s, n_x_f, n_y_f, n_q, a_loc, a_target, 
                                         a_task, a_t_trav, a_t_scan, l_x_s, l_y_s, c_f_s, 
                                         c_fs_1, c_fs_2, v_as, v_w, ang_w, r_bo, r_fo, fis_data)

    # For reproducibility
    np.random.seed(k)

    # Optimisation
    if solver == "fminsearch":
        res = minimize(h_MPC, ini_params, method='nelder-mead', options=options)
        mpc_params = res.x
    elif solver == "ga":
        linear_constraint = LinearConstraint(A, b, Aeq, beq)
        nonlinear_constraint = NonlinearConstraint(nonlcon, lb, ub)
        res = differential_evolution(h_MPC, bounds=options, constraints=(linear_constraint, nonlinear_constraint))
        mpc_params = res.x
    elif solver == "patternsearch":
        # Patternsearch is not directly available in Python. You can use alternative algorithms or
        # use third-party libraries that implement pattern search.
        pass
    elif solver == "particleswarm":
        # Particleswarm is not directly available in Python. You can use alternative algorithms or
        # use third-party libraries that implement particle swarm optimization.
        pass

    # Update FIS Parameters
    range = 0
    for a in range(n_a):
        fis_params = mpc_params[range:range+4]
        fisArray[a].outputs.membership_functions.parameters = fis_params
        range += 4

    # Update initial guess
    ini_params = mpc_params

    # Record new parameters
    fis_param_hist.append(mpc_params[0:n_a*4])

    return fisArray, ini_params, fis_param_hist
