import numpy as np

def model_environment(m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, m_s, m_bo, m_bt, dt_e, k, seed, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, c_f_s, flag_mpc):
    # Initialise variables
    t_i = 120              # Ignition time (s)
    t_b = 600              # Burnout time (s)
    F   = np.zeros((n_x_e,n_y_e))   # Fire spread probability map
    fire_count = 0

    # Downwind map
    W_dir_ws    = np.zeros((n_x_e, n_y_e))
    W_dis_ws    = np.zeros((n_x_e, n_y_e))
    m_t_dw_fine   = np.zeros((n_x_e, n_y_e))
    m_t_dw_temp   = np.zeros((n_x_e, n_y_e))

    # Seed random number generator using timestep fore repeatability
    np.random.seed(seed + k) 

    ## Calculate W    
    r_w     = 3
    n_f_x   = r_w*2+1
    n_f_y   = r_w*2+1
    w_dir   = np.zeros((n_f_x, n_f_y))
    w_dis   = np.zeros((n_f_x, n_f_y))
    c_wm_1  = 0.1
    c_wm_2  = 0.1
    c_wm_d  = 0.4

    for i in range(n_f_x):
        for j in range(n_f_y):
            f_d = np.arctan2((i-(r_w+1)),(j-(r_w+1))) # Fire direction [-pi, pi]
            ang = ang_w-f_d
            w_dir[i, j] = np.exp(v_w*(c_wm_1 + c_wm_2*(np.cos(ang)-1)))
            w_dis[i, j] = c_wm_d**np.sqrt((i-(r_w+1))**2 + (j-(r_w+1))**2)
            W = w_dir * w_dis

    for i in range(n_x_e):
        for j in range(n_y_e):
            if m_f[i, j] == 2:
                m_bt[i, j] += dt_e
                if m_bt[i, j] >= t_i:
                    m_f[i, j] = 3
            elif m_f[i, j] == 3:
                m_bt[i, j] += dt_e
                t_ckl = m_bt[i, j]
                if t_ckl <= (t_b-t_i)/5+t_i:
                    p = 4/(t_b-t_i)*t_ckl+(0.2*t_b-4.2*t_i)/(t_b-t_i)
                elif t_ckl <= t_b:
                    p = 5/(4*(t_b-t_i))*(-t_ckl+t_b)
                else:
                    print(f"t_ckl not in range: {t_ckl}")

                for ii in range(W.shape[0]):
                    for jj in range(W.shape[0]):
                        iii = i+ii-r_w
                        jjj = j+jj-r_w
                        if iii >= 0 and iii < n_x_e and jjj >= 0 and jjj < n_y_e:
                            F[iii, jjj] = cquote("import numpy as np\n\ndef", "m_t_dw_hist_animate.append(m_t_dw)\n\n    return m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, m_bt, m_t_dw")
                            F[iii, jjj] = c_fs_1*(m_s[iii, jjj] * m_bo[iii, jjj]) * W[ii, jj]**c_fs_2*p
                if m_bt[i, j] >= t_b:
                    m_f[i, j] = 4

    for i in range(n_x_e):
        for j in range(n_y_e):
            if m_f[i, j] == 1 and np.random.rand() <= F[i, j]:
                m_f[i, j] = 2
                if not flag_mpc:
                    m_f_hist[i, j] = k

    m_f_hist_animate.append(np.copy(m_f))

    for i in range(n_x_e):
        for j in range(n_y_e):
            if m_f[i, j] == 2 or m_f[i, j] == 3:
                fire_count += 1
                for m in range(n_x_e):
                    for n in range(n_y_e):
                        f_d             = np.arctan2((m-i),(n-j))
                        ang             = ang_w-f_d
                        W_dir_ws[m, n]  = np.exp(v_w*(c_wm_1 + c_wm_2*(np.cos(ang)-1))) 
                        W_dis_ws[m, n]  = 1 - np.sqrt((m-i)**2 + (n-j)**2)/np.sqrt(n_x_e**2 + n_y_e**2)
                W_dir_ws = W_dir_ws / np.max(W_dir_ws)
                m_t_dw_temp[:, :, fire_count] = W_dir_ws * W_dis_ws
                m_t_dw_temp[i, j, fire_count] = 1

    for i in range(n_x_e):
        for j in range(n_y_e):
            m_t_dw_fine[i, j] = np.max(m_t_dw_temp[i, j, :])

    m_t_dw_fine = np.ones((n_x_e, n_y_e)) - m_t_dw_fine

    m_t_dw, _ = func_coarsen(m_t_dw_fine, c_f_s)
    m_t_dw_hist_animate.append(np.copy(m_t_dw))

    return m_f, m_f_hist, m_f_hist_animate, m_t_dw_hist_animate, m_bt, m_t_dw
