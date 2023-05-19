import numpy as np

def model_agent(n_a, m_t_scan, m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, a_t_trav, a_t_scan, l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, flag_mpc):
    for a in range(n_a):
        if a_task[a] == 1:
            # Travel
            a_t_trav[a] -= dt_a
            if a_t_trav[a] <= 0:
                # Set task to scan
                a_task[a] = 2
                # Update agent location
                a_loc[a] = a_target[a, 0]
                # Update agent location history
                a_loc_hist.append(list(a_loc[a]) + [a, k])
                # Initialise remaining scantime, using additional time from the travel
                a_t_scan[a] = m_t_scan[a_loc[a, 0], a_loc[a, 1]] + a_t_trav[a]
        elif a_task[a] == 2:
            # Scan
            a_t_scan[a] -= dt_a
            if a_t_scan[a] <= 0:
                # Set task to travel
                a_task[a] = 1
                # Get cell coordinates
                i = a_loc[a, 0]
                j = a_loc[a, 1]
                # Set cell to scanned
                m_scan[i, j] = 1
                # Update scan history
                if not flag_mpc:
                    m_scan_hist[i, j] = k
                # Shift target list
                a_target[a, 0] = np.roll(a_target[a, 0], -1)
                a_target[a, 1] = np.roll(a_target[a, 1], -1)
                a_target[a, -1] = [np.nan, np.nan]
                # Set task to idle if no target, otherwise calculate travel time
                if np.isnan(a_target[a, 0, 0]):
                    a_task[a] = 3
                else:
                    a_t_trav[a] = calc_t_trav(a_loc[a], a_target[a, 0], l_x_s, l_y_s, ang_w, v_w, v_as)
        elif a_task[a] == 3:
            return m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, a_t_trav, a_t_scan
    return m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, a_t_trav, a_t_scan
