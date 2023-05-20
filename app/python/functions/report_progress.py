def report_progress(endCondition, t, t_f, m_scan, n_x_search, n_y_search):
    # Percent of time passed
    if endCondition == "time":
        prog = t / t_f * 100
    # Percent of map scanned
    elif endCondition == "scan":
        prog = np.sum(m_scan) / (n_x_search * n_y_search) * 100

    print("Simulation progress = {:.2f} %".format(prog))

