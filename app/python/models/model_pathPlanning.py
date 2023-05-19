import numpy as np

def model_pathPlanning(n_a, a_target, n_q, n_x_s, n_y_s, l_x_s, l_y_s, 
                       m_scan, m_t_scan, m_t_dw, m_prior, fisArray, a_t_trav, 
                       a_t_scan, ang_w, v_as, v_w, test_fis_sensitivity, fis_data):
  
    # Initialise maps
    m_schedule = np.zeros((n_x_s, n_y_s))
    for a in range(n_a):
        if not np.isnan(a_target[a, :, 0]).all():
            m_schedule[a_target[a, 0, 0], a_target[a, 1, 0]] = 1
            
    # Assign tasks
    for q in range(1, n_q):
        # Calculate normalised response time for all agents
        m_t_response = calc_t_response(
            n_x_s, n_y_s, l_x_s, l_y_s, 
            n_a, a_t_scan, a_t_trav, a_target, q, 
            ang_w, v_w, v_as, m_t_scan)
        
        for a in range(n_a):
            # FIS of agent
            fis = fisArray[a]
            # Generate attraction map
            m_att = np.zeros((n_x_s, n_y_s))
            for i in range(n_x_s):
                for j in range(n_y_s):
                    m_att[i, j] = calc_att(
                        fis, 
                        m_t_response[i, j, a], m_prior[i, j], m_t_dw[i, j], 
                        m_scan[i, j], m_schedule[i, j]
                    )
                    if test_fis_sensitivity:
                        # Record fis data
                        fis_data.append([m_t_response[i, j, a], m_prior[i, j], m_t_dw[i, j], m_att[i, j]])
            # Task assignment
            a_target, m_schedule = func_taskAssignment(a, q, a_target, m_att, m_schedule)
    
    return a_target, fis_data
