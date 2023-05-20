import numpy as np
import skfuzzy as fuzz
import math

def calc_att(fis, t_response, p_prior, t_dw, scan_state, schedule_state):
    if scan_state or schedule_state:
        return np.nan
    else:
        return fuzz.interp_membership(fis, [t_response, p_prior, t_dw])


def calc_coarsenRatio(ref, l_d):
    # Difference in lat negligible for small scale
    lat = ref['LatitudeLimits'][0]
    m_per_deg_lat = 111132.954 - 559.822 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat)
    m_per_deg_lon = 111132.954 * math.cos(lat)
    l_r_lat = ref['CellExtentInLatitude'] * m_per_deg_lat
    l_r_lon = ref['CellExtentInLongitude'] * m_per_deg_lon

    # Coarsen to desired cell size - side to right is removed if necessary
    n_lat = round(l_d / l_r_lat)
    n_lon = round(l_d / l_r_lon)
    c_f = [n_lat, n_lon]
    # length of new cells in x direction
    l_x = n_lat * m_per_deg_lat * ref['CellExtentInLatitude']
    # length of new cells in y direction
    l_y = n_lon * m_per_deg_lon * ref['CellExtentInLongitude']

    if n_lat < 2 or n_lon < 2:
        raise ValueError("Coarsen ratio not high enough")

    return c_f, l_x, l_y

def calc_obj(m_f, m_bo, m_scan, r_bo, r_fo, dt_s, s_obj, n_x_e, n_y_e, n_x_s, n_y_s, c_f_s):
    # Generate active fire map
    m_fo = np.copy(m_f)
    m_fo[m_fo == 1] = 0
    m_fo[m_fo == 2] = 0
    m_fo[m_fo == 3] = 1
    m_fo[m_fo == 4] = 0
    
    # Convert scan map to environment map resolution
    m_scan_env = np.zeros((n_x_e, n_y_e))
    for i in range(n_x_s):
        for j in range(n_y_s):
            if m_scan[i,j] == 1:
                # Calculate range in m_scan_env
                x_min = c_f_s*(i)
                x_max = x_min + c_f_s
                y_min = c_f_s*(j)
                y_max = y_min + c_f_s
                # Set range as scanned
                m_scan_env[x_min:x_max,y_min:y_max] = 1

    # Inverse scan map
    m_scan_env_inv = np.ones((n_x_e, n_y_e)) - m_scan_env
    # Priority due to building occupancy
    m_P_bo = r_bo * m_bo
    # Priority due to fire occupancy
    m_P_fo = r_fo * m_fo    
    # Priority map
    m_P = m_P_bo + m_P_fo
    # Ignore already scanned cells
    m_P = m_P * m_scan_env_inv
    # Objective function for current timestep
    obj = np.sum(m_P) * dt_s       
    # Sum of objective over time
    s_obj = s_obj + obj

    return s_obj, obj


def calc_att(fis, t_response, p_prior, t_dw, scan_state, schedule_state):
    if scan_state or schedule_state:
        return np.nan
    else:
        return fuzz.interp_membership(fis, [t_response, p_prior, t_dw])


def calc_coarsenRatio(ref, l_d):
    # Difference in lat negligible for small scale
    lat = ref['LatitudeLimits'][0]
    m_per_deg_lat = 111132.954 - 559.822 * math.cos(2 * lat) + 1.175 * math.cos(4 * lat)
    m_per_deg_lon = 111132.954 * math.cos(lat)
    l_r_lat = ref['CellExtentInLatitude'] * m_per_deg_lat
    l_r_lon = ref['CellExtentInLongitude'] * m_per_deg_lon

    # Coarsen to desired cell size - side to right is removed if necessary
    n_lat = round(l_d / l_r_lat)
    n_lon = round(l_d / l_r_lon)
    c_f = [n_lat, n_lon]
    # length of new cells in x direction
    l_x = n_lat * m_per_deg_lat * ref['CellExtentInLatitude']
    # length of new cells in y direction
    l_y = n_lon * m_per_deg_lon * ref['CellExtentInLongitude']

    if n_lat < 2 or n_lon < 2:
        raise ValueError("Coarsen ratio not high enough")

    return c_f, l_x, l_y

def calc_obj(m_f, m_bo, m_scan, r_bo, r_fo, dt_s, s_obj, n_x_e, n_y_e, n_x_s, n_y_s, c_f_s):
    # Generate active fire map
    m_fo = np.copy(m_f)
    m_fo[m_fo == 1] = 0
    m_fo[m_fo == 2] = 0
    m_fo[m_fo == 3] = 1
    m_fo[m_fo == 4] = 0
    
    # Convert scan map to environment map resolution
    m_scan_env = np.zeros((n_x_e, n_y_e))
    for i in range(n_x_s):
        for j in range(n_y_s):
            if m_scan[i,j] == 1:
                # Calculate range in m_scan_env
                x_min = c_f_s*(i)
                x_max = x_min + c_f_s
                y_min = c_f_s*(j)
                y_max = y_min + c_f_s
                # Set range as scanned
                m_scan_env[x_min:x_max,y_min:y_max] = 1

    # Inverse scan map
    m_scan_env_inv = np.ones((n_x_e, n_y_e)) - m_scan_env
    # Priority due to building occupancy
    m_P_bo = r_bo * m_bo
    # Priority due to fire occupancy
    m_P_fo = r_fo * m_fo    
    # Priority map
    m_P = m_P_bo + m_P_fo
    # Ignore already scanned cells
    m_P = m_P * m_scan_env_inv
    # Objective function for current timestep
    obj = np.sum(m_P) * dt_s       
    # Sum of objective over time
    s_obj = s_obj + obj

    return s_obj, obj

def calc_prior(c_prior_building, c_prior_open, bo_search):
    prior = c_prior_building*bo_search + c_prior_open*(1-bo_search)
    return prior

def calc_t_response(n_x_s, n_y_s, l_x_s, l_y_s, n_a, a_t_scan, a_t_trav, a_target, q, ang_w, v_w, v_as, m_t_scan):
    # Initialise t_nextcell_mat
    m_t_response = np.full((n_x_s, n_y_s, n_a), np.nan)
    for a in range(n_a):
        # Input matrix generation
        for i in range(n_x_s):
            for j in range(n_y_s):
                # Travel time for current task
                t_response = a_t_trav[a] + a_t_scan[a]
                # Travel time for tasks in queue
                for p in range(q-1):
                    # Start location
                    loc_1 = a_target[a, :, p]
                    # End location            
                    if p != q-2:
                        loc_2 = a_target[a, :, p+1]
                    else:
                        loc_2 = [i, j]
                    # Travel time
                    t_travel = calc_t_trav(loc_1, loc_2, l_x_s, l_y_s, ang_w, v_w, v_as)
                    # Add to travel time
                    t_response = t_response + t_travel + m_t_scan[i,j]
                # Add to travel time matrix
                m_t_response[i, j, a] = t_response
    # Normalise time input in range [0 1]
    m_t_response = m_t_response/np.max(m_t_response)

    return m_t_response

def calc_t_trav(loc_1, loc_2, l_x_s, l_y_s, ang_w, v_w, v_as):
    # Distance in m
    d = np.sqrt((l_x_s*(loc_2[0] - loc_1[0])**2) + (l_y_s*(loc_2[1] - loc_1[1])**2))
    # Ground angle
    a_g = np.arctan2(loc_1[1] - loc_2[1], loc_1[0] - loc_2[0])
    # Wind to track angle
    a_wt = a_g - ang_w
    # Wind correction angle
    a_wca = np.arcsin(v_w*np.sin(a_wt)/v_as)
    # Ground speed
    v_gs = v_as*np.cos(a_wca) + v_w*np.cos(a_wt)
    # Add to travel time
    t_travel = d/v_gs
    
    return t_travel

def func_coarsen(m_p_in, c_f):
    n_lat = c_f[0]
    n_lon = c_f[1]

    # Initialise maps
    g_d = [m_p_in.shape[0]//n_lat, m_p_in.shape[1]//n_lon]
    m_occ = np.zeros((g_d[0], g_d[1]))
    m_r = np.zeros((g_d[0], g_d[1]))

    # Calculate occupancy map and raster map
    for i in range(g_d[0]):
        for j in range(g_d[1]):
            ii = i*n_lat
            jj = j*n_lon
            # Extract data from original raster
            mat = m_p_in[ii:ii+n_lat, jj:jj+n_lon]
            occ = np.sum(mat)/(n_lat*n_lon)

            # occupancy map
            m_occ[i,j] = occ

            # raster map
            if occ > 0:
                m_r[i,j] = 1

    return m_occ, m_r

def func_endCondition(endCondition, t, t_f, m_scan, n_x_search, n_y_search):
    # Time based
    if endCondition == "time":
        if t >= t_f:
            return True
        else:
            return False
    # Entire map scanned
    elif endCondition == "scan":
        if m_scan.sum() == n_x_search*n_y_search:
            return True
        else:
            return False

def func_MPC_model(params, fisArray, test_fis_sensitivity, 
                   m_f, m_bo, m_bt, m_prior, m_s, m_scan, m_t_scan, 
                   dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seed, 
                   n_a, n_MF_out, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, 
                   a_loc, a_target, a_task, a_t_trav, a_t_scan, 
                   l_x_s, l_y_s, c_f_s, 
                   c_fs_1, c_fs_2, v_as, v_w, ang_w, 
                   r_bo, r_fo, fis_data):
    flag_mpc = True

    # Variables
    k_pred  = 0
    k_a     = 0
    k_c     = 0
    k_e     = 0
    k_mpc   = 0
    s_obj_pred   = 0

    dt_a    = dk_a*dt_s
    dt_e    = dk_e*dt_s

    # Maps
    m_t_dw    = np.zeros((n_x_s, n_x_s))
    range = 1  # Range

    # Prediction
    while k_pred < dk_mpc*n_p:
        # Update FIS parameters
        if k_mpc*dk_mpc <= k_pred:
            for a in range(n_a):
                for mf in range(n_MF_out):
                    newParams = params[range:range+3]
                    fisArray[a].Outputs.MembershipFunctions[mf].Parameters = newParams
                    range += 4
            k_mpc += 1
        
        # Path planning
        if k_c*dk_c <= k_pred:
            a_target, fis_data = model_pathPlanning(
                n_a, a_target, n_q, 
                n_x_s, n_y_s, l_x_s, l_y_s, 
                m_scan, m_t_scan, m_t_dw, m_prior, 
                fisArray, 
                a_t_trav, a_t_scan, 
                ang_w, v_as, v_w, test_fis_sensitivity, fis_data)
            k_c += 1
        
        # Agent actions
        if k_a*dk_a <= k_pred:
            m_scan, _, a_loc, _, a_task, a_target, a_t_trav, a_t_scan = model_agent( 
                n_a, m_t_scan, m_scan, None, a_loc, None, a_task, a_target, 
                a_t_trav, a_t_scan, l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, flag_mpc)
            k_a += 1
        
        # Environment model
        if k_e*dk_e <= k_pred:
            m_f, _, _, _, m_bt, m_t_dw = model_environment(
                m_f, None, None, None, m_s, m_bo, m_bt, dt_e, k, seed, n_x_e, n_y_e, 
                v_w, ang_w, c_fs_1, c_fs_2, c_f_s, flag_mpc)
            k_e += 1
        
        # Objective function evaluation
        s_obj_pred, _ = calc_obj(
            m_f, m_bo, m_scan,r_bo, r_fo, dt_s, s_obj_pred, n_x_e, n_y_e, n_x_s, n_y_s, c_f_s, False)
        
        # Advance timestep
        k_pred += 1
        k += 1

    return s_obj_pred

def func_taskAssignment(a, q, a_target, m_att, m_schedule):
    if np.isnan(m_att).all():
        a_target[a, :, q] = np.nan
    else:
        # Find max attraction cell. If multiple return first one.
        i, j = np.unravel_index(np.nanargmax(m_att), m_att.shape)
        
        # Assign cell as target
        a_target[a, :, q] = [i, j]
        
        if np.all(a_target[a, :, q] == a_target[a, :, 0]):
            print("Repeat in queue")
            
        # Update scan schedule map
        m_schedule[i, j] = 1
        
    return a_target, m_schedule
