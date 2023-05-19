from scipy import optimize
import numpy as np

def initialise_MPC_01(fis_array, n_a):
    flag_mpc = False
    # Solver options: fmincon, ga, particleswarm, patternsearch
    solver = "patternsearch"
    # Prediction horizon
    n_p = 1
    # Initialise optimisation parameters
    fis_params = []
    for a in range(n_a):
        fis_params.extend(fis_array[a]['Outputs']['MembershipFunctions']['Parameters'])
    ini_params = []
    for i in range(n_p):
        ini_params.extend(fis_params)
    # Constraints
    A       = []
    b       = []
    Aeq     = []
    beq     = []
    lb      = []
    ub      = []
    nonlcon = []
    nvars = len(ini_params)
    # Solver options
    options = {'disp': True, 'maxiter': 60}

    return flag_mpc, solver, options, n_p, fis_params, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, nvars
