import numpy as np

def initialise_plotting(m_p_ref, n_x_e, n_y_e, n_x_s, n_y_s, m_f, m_bt, n_a, a_loc, k, fis_params):
    # Axes may not be entirely accurate as coarsening may remove some
    # rows/columns from original map.
    # Axes for dynamic environment states
    axis_x_e = np.linspace(m_p_ref['LatitudeLimits'][0],  m_p_ref['LatitudeLimits'][1],  n_x_e)
    axis_y_e = np.linspace(m_p_ref['LongitudeLimits'][0], m_p_ref['LongitudeLimits'][1], n_y_e)
    # Axes for search map
    axis_x_s = np.linspace(m_p_ref['LatitudeLimits'][0],  m_p_ref['LatitudeLimits'][1],  n_x_s)
    axis_y_s = np.linspace(m_p_ref['LongitudeLimits'][0], m_p_ref['LongitudeLimits'][1], n_y_s)
    # History plots
    obj_hist    = []
    s_obj_hist  = []
    t_hist      = []
    m_f_hist    = m_f
    m_f_hist_animate  = m_f
    m_bt_hist_animate = m_bt
    a_loc_hist    = []
    for a in range(n_a):
        a_loc_hist.append([a_loc[a, 0], a_loc[a, 1], a, k])
    m_scan_hist       = np.zeros((n_x_s, n_y_s))
    m_dw_hist_animate = np.zeros((n_x_s, n_y_s))
    fis_param_hist    = fis_params

    return (axis_x_e, axis_y_e, axis_x_s, axis_y_s, m_f_hist, m_f_hist_animate, 
            m_bt_hist_animate, m_scan_hist, a_loc_hist, t_hist, fis_param_hist, obj_hist, s_obj_hist)
