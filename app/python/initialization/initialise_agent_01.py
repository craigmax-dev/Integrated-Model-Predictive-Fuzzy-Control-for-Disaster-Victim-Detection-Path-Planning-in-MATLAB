import numpy as np

def initialise_agent_01(m_bo, l_x_e, l_y_e):
    # Search map coarsen factors
    c_f_s = [5, 5]
    # Search map building occupancy
    m_bo_s = func_coarsen(m_bo, c_f_s) 
    # Search map dimensions
    n_x_s = m_bo_s.shape[0]
    n_y_s = m_bo_s.shape[1]
    # Search map cell lengths
    l_x_s = c_f_s[0]*l_x_e
    l_y_s = c_f_s[1]*l_y_e
    # Agent parameters
    n_a = 2  # Number of UAVs in simulation
    n_q = 2  # Queue length for UAV tasks
    v_as = 5  # UAV airspeed (m/s)
    a_t_trav = np.zeros((n_a, 1))  # Time left to complete travel
    t_scan_m = 0.1  # Scan time per square metre
    a_task = 2*np.ones((n_a, 1))  # Current task for UAVs
    a_loc = np.array([[1, 1], [1, 2]])  # Current locations of UAVs
    # Agent targets
    a_target = np.full((n_a, 2, n_q), np.nan)
    a_target[:, :, 0] = a_loc
    # Search map cell scan time
    t_scan_c = t_scan_m*l_x_s*l_y_s  # Scan time per cell
    m_scan = np.zeros((n_x_s, n_y_s))  # Scan map
    # Unsorted
    m_t_scan = t_scan_c*np.ones((n_x_s, n_y_s))  # Scan time map (s) - time to scan each cell
    a_t_scan = np.zeros((n_a, 1))  # Time left to complete current scanning task
    for a in range(n_a):
        a_t_scan[a] = m_bo_s[a_loc[a, 0]-1, a_loc[a, 1]-1]  # Python uses 0-based indexing

    return (n_x_s, n_y_s, l_x_s, l_y_s, n_a, n_q, v_as, a_t_trav,
            t_scan_m, t_scan_c, a_task, a_loc, a_target, a_t_scan,
            m_scan, m_t_scan, m_bo_s)
