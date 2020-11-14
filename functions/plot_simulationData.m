%% Function genPlots 

%% TO DO
% Create function to load in labels and color maps
% How to determine type / category of plot? By file extension or otherwise?
% Categorise and create figures programmatically

%% Script
function [] = plot_simulationData( ...
                simulation_plots, exp_folder, ...
                axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
                t_f, n_x_s, n_y_s, n_a, ct_v, fisArray)
                      
	% Close any open figures
  close all

  % Create save directory
  if(~exist(exp_folder, 'dir'))
    mkdir(exp_folder)
  end
  
  % Time vectors
  axis_t_v = linspace(0, t_f, ct_v);
 
  % Plotting
  for i = 1:size(simulation_plots,1)
    % Load data
    data_name = simulation_plots{i,1};
    data      = simulation_plots{i,2};
    data_type = simulation_plots{i,3};
    flag_plot = simulation_plots{i,4};
    [lab_title, lab_x, lab_y, lab_legend, lab_cmap, ~, ~] = func_plot_labels(data_name, data_type);
    [cmap, cmap_axis] = func_plot_colormaps(data_name, false);
    
    if flag_plot == true
      % Create figure
      f = figure("name", data_name);
      title(lab_title);
      xlabel(lab_x);
      ylabel(lab_y);
      legend(lab_legend);
      daspect([1 1 1]);
      axis square tight manual
      % Animations
      if data_type == "animate"
        file_name = strcat(exp_folder, "\", data_name, ".gif");
        % Create frames
        for n = 1:size(data,3)
          title(lab_title);
          xlabel(lab_x);
          ylabel(lab_y);
          legend(lab_legend);
          axis square tight manual
          colormap(cmap);
          caxis(cmap_axis);
          imagesc(axis_x_e, axis_y_e, data(:,:,n));
          % Capture plot as image
          frame = getframe(f);
          im = frame2im(frame);
          [imind,cm] = rgb2ind(im,256);
          % Write to the GIF File 
          if n == 1
            imwrite(imind,cm,file_name,"gif", "Loopcount",inf); 
          else
            imwrite(imind,cm,file_name,"gif","WriteMode","append"); 
          end
         end
        close(f);
      elseif data_type == "environment_map"
        imagesc(axis_x_e, axis_y_e, data)
        xlabel(lab_x, 'Interpreter', 'latex');
        ylabel(lab_y, 'Interpreter', 'latex');
        c = colorbar;
        c.Label.String = lab_cmap;
        c.Label.Interpreter = 'latex';
        c.Label.Rotation = 0;
      elseif data_type == "search_map"
        imagesc(axis_x_s, axis_y_s, data)
        xlabel(lab_x, 'Interpreter', 'latex');
        ylabel(lab_y, 'Interpreter', 'latex');
        c = colorbar;
        c.Label.String = lab_cmap;
        c.Label.Interpreter = 'latex';
        c.Label.Rotation = 0;
      elseif data_type == "variable"
        plot(axis_t_v, data)
        xlabel(lab_x, 'Interpreter', 'latex');
        ylabel(lab_y, 'Interpreter', 'latex');
      end
    end
  end
  %% Save figures as .fig and .jpg files
  fig_list = findobj(allchild(0), "Type", "figure");
  for iFig = 1:length(fig_list)
    h_fig = fig_list(iFig); 
    fig_name   = get(h_fig, "Name");
    exp_fig = strcat(exp_folder, "\", fig_name);
    exportgraphics(h_fig, strcat(exp_fig, ".jpg"));
  end
end
  
  %% TO ORGANISE
% FIS SENSITIVITY
%   % Data
%   x = data(:,1);
%   y = data(:,2);
%   z = data(:,3);
%   c = data(:,4);
% 
%   % Plot
%   scatter3(x, y, z, 2, c);
%   c = colorbar;
% 
%   % Labels
%   xlabel('Nextcell Time')
%   ylabel('Priority')
%   zlabel('Downwind Time')
%   c.Label.String = 'Attraction';

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       elseif data_name == "UAV_loc_hist"
%         f = figure("name","UAV_loc_hist");
%         hold on;
%         grid on;
%         xlabel("Latitude");
%         ylabel("Longitude");
%         axis([0.5 n_x_s+0.5 0.5 n_y_s+0.5]);
%         daspect([1 1 1]);
%         title("UAV Paths");
%         lineWidth = 1;
% 
%         % Search cell array for m_bo
%         ind = find(cellfun(@(x)ischar(x)&&x=="m_prior",dataset));
%         m_prior = dataset{ind,2};
%         % Plot bo map
%         imagesc(m_prior);
% 
%         for UAV=1:n_a
%           % Build path
%           agentPath = zeros(1, 3);
%           count = 0;
%           pathColor = cmap_path(UAV,:);
%           for p=1:length(data)
%             if(data(p, 3) == UAV)
%               count = count + 1;
%               agentPath(count, :) = data(p,[1:2,4]);
%             end
%           end
%           % Plot path
%           for p = 1:length(agentPath)-1
%             dp = agentPath(p+1,1:2) - agentPath(p,1:2);
%             if p == 1
%               q = quiver(agentPath(p,1), agentPath(p,2), dp(1), dp(2), 0, "Color",pathColor, "linewidth", lineWidth);
%             else
%               q = quiver(agentPath(p,1), agentPath(p,2), dp(1), dp(2), 0, "Color",pathColor, "linewidth", lineWidth, "HandleVisibility","off");
%             end
% %             alpha(alp(agentPath(p,3)));
%           end
%           if UAV == 1
%             str = {strcat("Agent ", num2str(UAV))};
%           else
%             str = [str , strcat("Agent ", num2str(UAV))];
%           end
%         end
%         legend(str);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       elseif data_name == "fis"
%         for iFis=1:length(data)
%           fis = data(iFis);
%           % Plot input membership functions
%           f = figure("name",sprintf("fis_inputs_agent%d", iFis));
%           sgtitle(sprintf("FIS Inputs for Agent %d", iFis));
%           for iFisIn = 1:length(fis.Inputs)
%             subplot(length(fis.Inputs),1,iFisIn);
%             plotmf(fis,"input",iFisIn);
%             title(fis.Inputs(iFisIn).name);
%           end
%           % Plot output surface
%           f = figure("name",sprintf("fis_outputs_agent%d", iFis));
%           sgtitle(sprintf("FIS Outputs for Agent %d", iFis));
%           gensurf(fis);
%         end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       elseif data_name == "fis_var_scaling"
%         f = figure("name","fis_var_scaling");
%         % Data
%         x = data(:,1);
%         y = data(:,2);
%         z = data(:,3);
%         c = data(:,4);
%         % Plot
%         scatter3(x, y, z, 2, c);
%         c = colorbar;
%         % Labels
%         xlabel("Nextcell Time")
%         ylabel("Priority")
%         zlabel("Downwind Time")
%         c.Label.String = "Attraction";
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       elseif data_name == "obj_hist_sens"
%         % Generate plots for sensitivity test
%         dataSize = size(data);
%         data = squeeze(data);
%         
%         if dataSize(1) > 1 && dataSize(2) > 1
%           xlab = "p1";
%           ylab = "p2";
%         elseif dataSize(1) > 1 && dataSize(3) > 1
%           xlab = "p1";
%           ylab = "p3";
%         elseif dataSize(1) > 1 && dataSize(4) > 1
%           xlab = "p1";
%           ylab = "p4";
%         elseif dataSize(2) > 1 && dataSize(3) > 1
%           xlab = "p2";
%           ylab = "p3";
%         elseif dataSize(2) > 1 && dataSize(4) > 1
%           xlab = "p2";
%           ylab = "p4";
%         elseif dataSize(3) > 1 && dataSize(4) > 1
%           xlab = "p3";
%           ylab = "p4";
%         end
% 
%         for n=1:dataSize(5)
%           nameStr = strcat("obj_hist_sens_", num2str(n));
%           f = figure("name",nameStr);
%           imagesc(data(:,:,n));
%           title(nameStr)
%           xlabel(xlab);
%           ylabel(ylab);
%         end
%       elseif data_name == "fis_param_hist"
%         % Scatter
%         nameStr = "fis_param_hist_scatter";
%         f = figure("name",nameStr);
%         x = linspace(1,4,4);
%         title("FIS Parameter Scatter Plots")
%         ct_MPC = size(data,1);
%         for p=1:ct_MPC
%           for a=1:n_a
%             r = (a-1)*3+1:(a-1)*3+4;
%             subplot(ceil(ct_MPC/2),2,p);
%             grid on
%             hold on
%             scatter(x, data(p,r));
%             title(strcat("MPC Step ", num2str(p))) 
%           end
%         end
%         legendStr = strings(0,n_a);
%         for a=1:n_a
%           legendStr(a) = strcat("Agent", num2str(a));
%         end
%         fig = gcf;
%         fig.Position(3) = fig.Position(3) + 250;
%         Lgnd = legend(legendStr);
%         Lgnd.Position(1) = 0.01;
%         Lgnd.Position(2) = 0.80;
%         
%         % Surface plots - under construction!       
%         for p=1:size(data, 1)
%           nameStr = strcat("fis_param_hist_scatter_MPC_step_", num2str(p));
%           f = figure("name",nameStr);
%           set(gcf, "Units", "Normalized", "OuterPosition", [0, 0.04, 1, 0.96]);
%           sgtitle(strcat("FIS Surface Plot - MPC Step ", num2str(p)))
%           opt = gensurfOptions;
%           c = 0;
%           c_arr = [ 1, 2;
%                     1, 3;
%                     2, 3];
%           for a=1:n_a
%             r = (a-1)*3+1:(a-1)*3+4;
%             fis = fisArray(a);
%             fis.outputs.MembershipFunctions.Parameters = data(p,r);
%             for sp_fis = 1:3
%               c = c+1;
%               sp(c) = subplot(n_a,3,c);
%               opt.InputIndex = c_arr(sp_fis,:);
%               gensurf(fis,opt);
%               axis square;
%             end            
%             UAVStr = strcat("UAV ", num2str(a));
%             spPos = cat(1,sp(r(1)).Position);
%             titleSettings = {"HorizontalAlignment","center","EdgeColor","none","FontSize",18};
%             annotation("textbox","Position",[spPos(1,1:2) -0.1 0.2],"String",UAVStr,titleSettings{:})
%           end
%         end
%       end
%     end
%   end
% 
% end