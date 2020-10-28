 %% Function genPlots 

%% Script
function [] = plot_simulationData( dataSet, saveDir, folder, ...
                        ax_lon_env, ax_lat_env, ax_lat_scan, ax_lon_scan, ...
                        dt_v, t_end, n_x_search, n_y_search, n_UAV, ct_v, fisArray)
                      
	% Close any open figures
  close all
  
  % Colour maps
  cmap_m_f  = [ 1,   1,   1;    % 0
                0.5, 0.5, 0.5;  % 1
                1,   0.5, 0;    % 2
                1,   0,   0;    % 3
                0,   0,   0];   % 4  
  cmap_path = hsv(n_UAV);
  
  % Time vectors
  t_v = linspace(0, t_end, ct_v);
  % Save working directory path
  workingDir = pwd;
  % Change to save directory
  cd(saveDir); 
  % Create save directory
  mkdir(folder);
  cd(folder);
  
  for i = 1:size(dataSet,1)
    dataName  = dataSet{i,1};
    data      = dataSet{i,2};
    plotData  = dataSet{i,3};

    if plotData == true
      if dataName == "m_f_hist"
        h = figure('name','m_f_hist');
        filename = 'm_f_hist.gif';
        for n = 1:ct_v
          % Axis
          axis square tight manual
          daspect([1 1 1]);
          % Colormap
          colormap(cmap_m_f);
          caxis([0 4]);
          % Data
          imagesc(ax_lon_env, ax_lat_env, data(:,:,n));
          % Labels
          title(strcat("Fire map history t = ", int2str(n*dt_v)));
          xlabel("Latitude");
          ylabel("Longitude");
%           legend('Unburnable','Burnable','Catching','Burning','Extinguished');          
          % Capture plot as image
          frame = getframe(h);
          im = frame2im(frame);
          [imind,cm] = rgb2ind(im,256);
          % Write to the GIF File 
          if n == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
          else
            imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          end
        end
        % Prevent saving
        close(h); 
      elseif dataName == "m_dw_hist"
        h = figure('name','m_dw_hist');
        filename = 'm_dw_hist.gif';
        for n = 1:ct_v
          % Axis
          axis square tight manual
          daspect([1 1 1]);
          % Data
          imagesc(ax_lon_env, ax_lat_env, data(:,:,n));
          % Labels
          title(strcat("Downwind proximity t = ", int2str(n*dt_v)));
          xlabel("Latitude");
          ylabel("Longitude");
%           legend("Downwind proximity");
          % Capture plot as image
          frame = getframe(h);
          im = frame2im(frame); 
          [imind,cm] = rgb2ind(im,256);
          % Write to the GIF File 
          if n == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
          else
            imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          end
        end
        % Prevent saving      
        close(h); 
      elseif dataName == "m_scan_hist" %% UNDER CONSTRUCTION
        filename = 'm_scan_hist.gif';
        h = figure('name','m_scan_hist');
        xlabel("Latitude");
        ylabel("Longitude");        
        for n = 1:ct_v
          % Axis
          axis square tight manual
          daspect([1 1 1]);
          % Data
          imagesc(ax_lon_scan, ax_lat_scan, data(:,:,n));
          % Labels
          title(strcat("Scan map history t = ", int2str(n*dt_v)));
          xlabel("Latitude");
          ylabel("Longitude");
          % Capture plot as image
          frame = getframe(h);
          im = frame2im(frame);
          [imind,cm] = rgb2ind(im,256);
          % Write to the GIF File 
          if n == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
          else
            imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          end
        end
        % Prevent saving
        close(h); 
      elseif dataName == "m_prior_hist" %% UNDER CONSTRUCTION
        filename = 'm_prior_hist.gif';
        h = figure('name','m_prior_hist');
        xlabel("Latitude");
        ylabel("Longitude");        
        for n = 1:ct_v
          % Axis
          axis square tight manual
          daspect([1 1 1]);
          % Data
          imagesc(ax_lon_scan, ax_lat_scan, data(:,:,n));
          % Labels
          title(strcat("Scan map history t = ", int2str(n*dt_v)));
          xlabel("Latitude");
          ylabel("Longitude");
          % Capture plot as image
          frame = getframe(h);
          im = frame2im(frame);
          [imind,cm] = rgb2ind(im,256);
          % Write to the GIF File 
          if n == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
          else
            imwrite(imind,cm,filename,'gif','WriteMode','append'); 
          end
        end
        % Prevent saving
        close(h);
      elseif dataName == "m_f"         %% UNDER CONSTRUCTION
        h = figure('name','m_f');
        xlabel("Latitude");
        ylabel("Longitude");
        axis square tight manual
        daspect([1 1 1]);
        title("");
      elseif dataName == "m_bt" %% UNDER CONSTRUCTION
        h = figure('name','m_bt');
        xlabel("Latitude");
        ylabel("Longitude");
        axis square tight manual
        daspect([1 1 1]);
        title("");
      elseif dataName == "m_dw" %% UNDER CONSTRUCTION
        h = figure('name','m_dw');
        xlabel("Latitude");
        ylabel("Longitude");
        axis square tight manual
        daspect([1 1 1]);
        title("");
      elseif dataName == "m_bo" %% UNDER CONSTRUCTION
        h = figure('name','m_bo');
        xlabel("Latitude");
        ylabel("Longitude");
        imagesc(ax_lon_scan, ax_lat_scan, data);
        axis square
        title("Building Occupancy Map");
      elseif dataName == "UAV_loc_hist"
        h = figure('name','UAV_loc_hist');
        hold on;
        grid on;
        xlabel("Latitude");
        ylabel("Longitude");
        axis([0.5 n_x_search+0.5 0.5 n_y_search+0.5]);
        daspect([1 1 1]);
        title("UAV Paths");
        lineWidth = 1;

        % Search cell array for m_bo
        ind = find(cellfun(@(x)ischar(x)&&x=="m_prior",dataSet));
        m_prior = dataSet{ind,2};
        % Plot bo map
        imagesc(m_prior);

        for UAV=1:n_UAV
          % Build path
          agentPath = zeros(1, 3);
          count = 0;
          pathColor = cmap_path(UAV,:);
          for p=1:length(data)
            if(data(p, 3) == UAV)
              count = count + 1;
              agentPath(count, :) = data(p,[1:2,4]);
            end
          end
          % Plot path
          for p = 1:length(agentPath)-1
            dp = agentPath(p+1,1:2) - agentPath(p,1:2);
            if p == 1
              q = quiver(agentPath(p,1), agentPath(p,2), dp(1), dp(2), 0, 'Color',pathColor, 'linewidth', lineWidth);
            else
              q = quiver(agentPath(p,1), agentPath(p,2), dp(1), dp(2), 0, 'Color',pathColor, 'linewidth', lineWidth, 'HandleVisibility','off');
            end
%             alpha(alp(agentPath(p,3)));
          end
          if UAV == 1
            str = {strcat("Agent ", num2str(UAV))};
          else
            str = [str , strcat("Agent ", num2str(UAV))];
          end
        end
        legend(str);
      elseif dataName == "fis"
        for iFis=1:length(data)
          fis = data(iFis);
          % Plot input membership functions
          h = figure('name',sprintf("fis_inputs_agent%d", iFis));
          sgtitle(sprintf("FIS Inputs for Agent %d", iFis));
          for iFisIn = 1:length(fis.Inputs)
            subplot(length(fis.Inputs),1,iFisIn);
            plotmf(fis,'input',iFisIn);
            title(fis.Inputs(iFisIn).name);
          end
          % Plot output surface
          h = figure('name',sprintf("fis_outputs_agent%d", iFis));
          sgtitle(sprintf("FIS Outputs for Agent %d", iFis));
          gensurf(fis);
        end
      elseif dataName == "s_obj_hist"
        h = figure('name','s_obj_hist');
        hold on;
        grid on;
        xlabel("Time (s)"); 
        ylabel("Objective function sum");
        title("Objective function sum over time");
        plot(t_v, data)
      elseif dataName == "obj_hist"
        h = figure('name','obj_hist');
        hold on;
        grid on;
        xlabel("Time (s)");
        ylabel("Objective function");
        title("Objective function over time");
        plot(t_v, data)
      elseif dataName == "fis_var_scaling"
        h = figure('name','fis_var_scaling');
        % Data
        x = data(:,1);
        y = data(:,2);
        z = data(:,3);
        c = data(:,4);
        % Plot
        scatter3(x, y, z, 2, c);
        c = colorbar;
        % Labels
        xlabel('Nextcell Time')
        ylabel('Priority')
        zlabel('Downwind Time')
        c.Label.String = 'Attraction';
      elseif dataName == "obj_hist_sens"
        % Generate plots for sensitivity test
        dataSize = size(data);
        data = squeeze(data);
        
        if dataSize(1) > 1 && dataSize(2) > 1
          xlab = "p1";
          ylab = "p2";
        elseif dataSize(1) > 1 && dataSize(3) > 1
          xlab = "p1";
          ylab = "p3";
        elseif dataSize(1) > 1 && dataSize(4) > 1
          xlab = "p1";
          ylab = "p4";
        elseif dataSize(2) > 1 && dataSize(3) > 1
          xlab = "p2";
          ylab = "p3";
        elseif dataSize(2) > 1 && dataSize(4) > 1
          xlab = "p2";
          ylab = "p4";
        elseif dataSize(3) > 1 && dataSize(4) > 1
          xlab = "p3";
          ylab = "p4";
        end

        for n=1:dataSize(5)
          nameStr = strcat("obj_hist_sens_", num2str(n));
          h = figure('name',nameStr);
          imagesc(data(:,:,n));
          title(nameStr)
          xlabel(xlab);
          ylabel(ylab);
        end
      elseif dataName == "fis_param_hist"
        % Scatter
        nameStr = "fis_param_hist_scatter";
        h = figure('name',nameStr);
        x = linspace(1,4,4);
        title("FIS Parameter Scatter Plots")
        ct_MPC = size(data,1);
        for p=1:ct_MPC
          for a=1:n_UAV
            r = (a-1)*3+1:(a-1)*3+4;
            subplot(ceil(ct_MPC/2),2,p);
            grid on
            hold on
            scatter(x, data(p,r));
            title(strcat("MPC Step ", num2str(p))) 
          end
        end
        legendStr = strings(0,n_UAV);
        for a=1:n_UAV
          legendStr(a) = strcat("Agent", num2str(a));
        end
        fig = gcf;
        fig.Position(3) = fig.Position(3) + 250;
        Lgnd = legend(legendStr);
        Lgnd.Position(1) = 0.01;
        Lgnd.Position(2) = 0.80;
        
        % Surface plots - under construction!       
        for p=1:size(data, 1)
          nameStr = strcat("fis_param_hist_scatter_MPC_step_", num2str(p));
          h = figure('name',nameStr);
          set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
          sgtitle(strcat("FIS Surface Plot - MPC Step ", num2str(p)))
          opt = gensurfOptions;
          c = 0;
          c_arr = [ 1, 2;
                    1, 3;
                    2, 3];
          for a=1:n_UAV
            r = (a-1)*3+1:(a-1)*3+4;
            fis = fisArray(a);
            fis.outputs.MembershipFunctions.Parameters = data(p,r);
            for sp_fis = 1:3
              c = c+1;
              sp(c) = subplot(n_UAV,3,c);
              opt.InputIndex = c_arr(sp_fis,:);
              gensurf(fis,opt);
              axis square;
            end            
            UAVStr = strcat('UAV ', num2str(a));
            spPos = cat(1,sp(r(1)).Position);
            titleSettings = {'HorizontalAlignment','center','EdgeColor','none','FontSize',18};
            annotation('textbox','Position',[spPos(1,1:2) -0.1 0.2],'String',UAVStr,titleSettings{:})
          end
        end
      end
    end
  end

  %% Save figures as .fig and .mat file
  FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
  for iFig = 1:length(FigList)
    FigHandle = FigList(iFig); 
    FigName   = get(FigHandle, 'Name');
    savefig(FigHandle, strcat(FigName, '.fig'));
    saveas(FigHandle, strcat(FigName, '.jpg'));
  end
  
  %% Go back to working directory
  cd(workingDir);

end