import numpy as np
import rasterio

def initialise_environment_01(dt_e, k):
    # Import building raster
    building_raster = 'data/maps/portAuPrince/portAuPrince_campeche.tif'
    with rasterio.open(building_raster) as dataset:
        m_p_in = dataset.read(1)[50:450, 50:250]
        m_p_ref = dataset.bounds

    # Environment map cell length (m)
    l_c_e = 3

    # Building occupancy map and structure map
    c_f_e, l_x_e, l_y_e = calc_coarsenRatio(m_p_ref, l_c_e)
    m_bo, m_s = func_coarsen(m_p_in, c_f_e)

    # Environment map dimensions
    n_x_e, n_y_e = m_bo.shape

    # Wind model
    v_w = 2        # Wind speed (m/s)
    ang_w = np.pi/2     # Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis

    # Initialise fire map
    m_f = np.copy(m_s)
    m_f[80:82,1:2] = 3 * m_s[80:82,1:2]

    # Initialise burntime map
    m_bt = np.zeros((n_x_e, n_y_e))

    # Fire model parameters
    c_fs_1 = 0.2    # Wind constant 1 (for fire model)
    c_fs_2 = 0.2    # Wind constant 2 (for fire model)
    c_f_s = [5, 5]

    #  Initialise fire maps
    m_f, _, _, _, m_bt, m_dw = model_environment(
        m_f, None, None, None, m_s, m_bo, m_bt, 
        dt_e, k, 0, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, c_f_s, False
    )

    return l_x_e, l_y_e, n_x_e, n_y_e, m_bo, m_s, m_f, m_bt, m_dw, m_p_ref, c_fs_1, c_fs_2, c_f_s, v_w, ang_w
