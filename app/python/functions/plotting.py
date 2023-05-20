import matplotlib.pyplot as plt
import numpy as np
import os
import scipy.io as sio
import imageio

def plot_simulationComparisons(plots_simSet, exp_dir, simulation_set, simulation_set_name, simulation_set_names):
    # Close any open figures
    plt.close('all')

    # Data point offsets
    offset_x = 100
    offset_y = 0
    xAdjust = 0.06
    xLabAdjust = [400,0,0]
    maxsize = 1

    # Plotting
    # Iterate through each plot in set
    for i in range(len(plots_simSet)):
        # Load data information
        dataName = plots_simSet[i][0]
        dataType = plots_simSet[i][1]
        flag_plot = plots_simSet[i][2]
        # Labels
        lab_title = ''
        lab_x = ''
        lab_y = ''
        # Number of simulations
        numSimulations = len(simulation_set)
        # Retrieve desired labels from label function
        _, lab_x, lab_y, _, _, pos, ylimits = func_plot_labels(dataName, dataType)
        # Color map
        cmap, _ = func_plot_colormaps('', True, numSimulations)
        
        if flag_plot:
            for sim in range(numSimulations):
                sim_name = simulation_set[sim][0]        
                sim_route = os.path.join(exp_dir, sim_name, sim_name + ".mat")
                data_struct = sio.loadmat(sim_route)
                data_raw = data_struct[dataName]
                dataSize = data_raw.shape[1]
                
                if dataSize > maxsize:
                    t_data_raw = sio.loadmat(sim_route)["t_hist"]
                    t_data = t_data_raw
                    maxsize = dataSize

            data = np.full((maxsize, numSimulations), np.nan)
            
            # Load data into matrix
            for sim in range(numSimulations):
                sim_name = simulation_set[sim][0]        
                sim_route = os.path.join(exp_dir, sim_name, sim_name + ".mat")
                data_struct = sio.loadmat(sim_route)
                data_raw = data_struct[dataName]
                data[0:len(data_raw),sim] = data_raw
            
            # Mean data and relative data
            data_rel = np.divide(data, data[:,0][:, None])
            
            if dataType == "variable":
                plotData = data
            elif dataType == "relative":
                plotData = data_rel
            
            # Plot figure
            figName = dataName + "_" + dataType
            plt.figure(figName)
            plt.grid(True)
            
            for sim in range(numSimulations):
                if numSimulations == 2:
                    lineStyle = "-"
                    colour = sim
                elif sim <= numSimulations // 2 + 1:
                    lineStyle = "-"
                    colour = sim
                else:
                    lineStyle = "--"
                    colour = sim - (numSimulations // 2)
                
                # Plot each row
                plt.plot(t_data, plotData[:,sim], linewidth=1.5, color=cmap[colour], linestyle=lineStyle)
                if dataType == "relative":
                    # Label last value in plot
                    numNans = np.sum(np.isnan(plotData[:, sim]))
                    finalPos = len(plotData[:, sim]) - numNans
                    finalVal = plotData[finalPos, sim]
                    if not np.isinf(finalVal):
                        plt.text(t_data[finalPos]+offset_x, finalVal+offset_y, str(finalVal))
            
            # Labels
            plt.title(lab_title)
            plt.ylim(ylimits)
            plt.xlabel(lab_x)
            plt.ylabel(lab_y, rotation=0)
            plt.legend(simulation_set_names, loc=pos)
            plt.tight_layout()
            plt.show()
    
    # Save figures as .fig and .jpg files
    figs = [plt.figure(n) for n in plt.get_fignums()]
    for iFig in range(len(figs)):
        figName = figs[iFig].get_label()
        # Export directory address
        exp_folder = os.path.join(exp_dir, simulation_set_name)
        exp_fig = os.path.join(exp_folder, figName)
        # Create save directory
        if not os.path.exists(exp_folder):
            os.makedirs(exp_folder)
        figs[iFig].savefig(exp_fig + ".jpg")
        plt.close(figs[iFig])

def plot_simulationData(simulation_plots, exp_folder, axis_x_e, axis_y_e, axis_x_s, axis_y_s, t_f, n_x_s, n_y_s, n_a, ct_v, fisArray, dt_s):
    # Close any open figures
    plt.close('all')

    # Create save directory
    os.makedirs(exp_folder, exist_ok=True)
    
    # Time vectors
    axis_t_v = np.linspace(0, t_f, ct_v)
 
    # Plotting
    for i in range(len(simulation_plots)):
        # Load data
        data_name = simulation_plots[i][0]
        data = simulation_plots[i][1]
        data_type = simulation_plots[i][2]
        flag_plot = simulation_plots[i][3]
        lab_title, lab_x, lab_y, lab_legend, lab_cmap, _, _ = func_plot_labels(data_name, data_type)
        cmap, cmap_axis = func_plot_colormaps(data_name, False)
    
        if flag_plot:
            # Create figure
            fig, ax = plt.subplots()
            ax.set_title(lab_title)
            ax.set_xlabel(lab_x)
            ax.set_ylabel(lab_y)
            ax.legend([lab_legend])
            ax.set_aspect('equal', 'box')
            plt.axis('tight')
            # Animations
            if data_type == "animate":
                file_name = os.path.join(exp_folder, data_name + ".gif")
                # Create frames
                images = []
                for n in range(data.shape[2]):
                    ax.set_title(lab_title)
                    ax.set_xlabel(lab_x)
                    ax.set_ylabel(lab_y)
                    ax.legend([lab_legend])
                    ax.set_aspect('equal', 'box')
                    plt.axis('tight')
                    plt.set_cmap(cmap)
                    plt.clim(cmap_axis)
                    im = ax.imshow(data[:,:,n], aspect='auto')
                    # Capture plot as image
                    fig.canvas.draw()
                    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
                    image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
                    images.append(image)
                imageio.mimsave(file_name, images, fps=30)
                plt.close(fig)
            elif data_type == "environment_map":
                if data_name == "m_f_hist":
                    data = dt_s*data
                im = ax.imshow(data, aspect='auto')
                ax.set_xlabel(lab_x)
                ax.set_ylabel(lab_y)
                c = plt.colorbar(im)
                c.set_label(lab_cmap)
            elif data_type == "search_map":
                im = ax.imshow(data, aspect='auto')
                ax.set_xlabel(lab_x)
                ax.set_ylabel(lab_y)
                c = plt.colorbar(im)
                c.set_label(lab_cmap)
            elif data_type == "variable":
                ax.plot(axis_t_v, data)
                ax.set_xlabel(lab_x)
                ax.set_ylabel(lab_y)
            plt.savefig(os.path.join(exp_folder, data_name + ".jpg"))
