function animateMPFCSimulation(agent_model, environment_model, config, flc_params_hist, results, output_filename)
%ANIMATEMPFCSIMULATION 
%  Create and save an animation (.gif) with subplots:
%   1) Top-left: "Environment Map" (not bold) with fire map + agents
%      - Agent 1: color (0,166,214)/255, marker 'o'
%      - Agent 2: color (237,104,66)/255, marker '^'
%      Markers have a thin black line around them.
%
%   2) Top-right: Three stacked sub-subplots (Low, Medium, High MFs)
%      - Some vertical space added between them.
%      - Only the first FLC subplot includes a legend (top-right corner).
%      - Markers also have a thin black line around them.
%
%   3) Bottom subplots (stacked):
%      (a) Objective function vs. main step k in colour (0,118,194)/255
%      (b) Optimisation time vs. MPC step in colour (224,60,49)/255
%      Dashed vertical lines appear only for steps reached.
%
%   4) "Simulation Parameters" title not in bold.
%   5) "k = X" text at bottom centre.
%
%   Extra whitespace is included around subplots for clarity.
%
% NOTE:
%   This version saves frames to a .gif file instead of .mp4.
%   The figure arrangement remains unchanged.

    %% Validate config
    if ~isfield(config, 'k') || ~isfield(config, 'dk_e') || ~isfield(config, 'dk_mpc')
        error('Config must contain k, dk_e, and dk_mpc.');
    end
    numAgents    = size(agent_model.a_loc_hist,1);
    numMainSteps = config.k;

    %% Plotting sizes
    size_marker  = 8;
    size_font    = 16;
    size_font_sub= 10;

    %% Create the figure (unchanged)
    figure('Units','pixels','Position',[100 100 1300 900],...
           'Color','w','Name','MPFC Animation','NumberTitle','off');
    colormap('hot');

    %% Agent Colors & Markers
    agentCols  = {[0,166,214]/255, [237,104,66]/255};  % Agent1/Agent2
    agentMarks = {'o','^'};

    %% FLC param storage
    oldParams = cell(numAgents,3);  % for Low, Medium, High MFs

    %% For dynamic y-limits on objective/time
    maxObjVal = 1e-12;
    maxOptVal = 1e-12;

    %% Colours for lines
    objColor = [0,118,194]/255;  % objective
    optColor = [224,60,49]/255;  % optimization

    %% Precompute maxMpcSteps
    maxMpcSteps = size(flc_params_hist,2);
    k_mpcMax    = ceil(numMainSteps / config.dk_mpc);
    if k_mpcMax>maxMpcSteps
        k_mpcMax = maxMpcSteps;
    end

    %% Set up .gif writing parameters
    delayTime = 0.5;  % seconds between frames (equivalent to ~2 fps)
    firstFrameWritten = false;  % track if we have written the first frame yet

    %% Main loop
    for k = 1:numMainSteps
        clf;

        %% SUBPLOT LAYOUT (unchanged)
        % Environment top-left
        subplot('Position',[0.05, 0.55, 0.35, 0.4]);
        k_e = min( ceil(k/config.dk_e), size(environment_model.m_f_series,3) );
        m_f = environment_model.m_f_series(:,:,k_e);
        [m_f_coarsened, ~] = func_coarsen(m_f, config.c_f_s);

        imagesc(m_f_coarsened,[0 4]);
        axis on; box on; set(gca,'YDir','normal','Layer','top');
        colorbar('Location','eastoutside');
        xlabel('$x$','Interpreter','latex');
        ylabel('$y$','Interpreter','latex');

        envTitle = title('Environment Map','FontSize',size_font,'FontWeight','normal');
        envTitle.Units='normalized';

        hold on;
        for a = 1:numAgents
            ax = agent_model.a_loc_hist(a,1,k);
            ay = agent_model.a_loc_hist(a,2,k);
            cA = agentCols{min(a,length(agentCols))};
            mA = agentMarks{min(a,length(agentMarks))};
            plot(ay, ax, mA,...
                 'MarkerSize',size_marker,...
                 'MarkerFaceColor',cA,...
                 'MarkerEdgeColor','k');
        end
        hold off;

        %% FLC subplots top-right
        totalFLCheight = 0.4;
        subGap = 0.02;
        eachH = (totalFLCheight - 2*subGap) / 3;  
        baseX=0.45; widthX=0.50; baseY=0.55;
        mfNames = {'Low','Medium','High'};
        k_mpc = min( ceil(k/config.dk_mpc), k_mpcMax);

        for mfIndex = 1:3
            yPos = baseY + (3-mfIndex)*(eachH+subGap);
            subplot('Position',[baseX, yPos, widthX, eachH]);
            hold on; box on; axis on;

            for a = 1:numAgents
                newCell = flc_params_hist{a, k_mpc}; 
                if iscell(newCell) && ~isempty(newCell)
                    paramCell = newCell{1}; 
                    if iscell(paramCell) && numel(paramCell)>=mfIndex
                        paramVec = paramCell{mfIndex};
                        if isnumeric(paramVec)
                            oldParams{a,mfIndex} = paramVec;
                        end
                    end
                end
                if ~isempty(oldParams{a,mfIndex})
                    cA = agentCols{min(a,length(agentCols))};
                    mA = agentMarks{min(a,length(agentMarks))};
                    xvals = 1:length(oldParams{a,mfIndex});
                    plot(xvals, oldParams{a,mfIndex},'-','Color',cA,'LineWidth',1.5,...
                         'Marker',mA,'MarkerSize',size_marker,...
                         'MarkerFaceColor',cA,'MarkerEdgeColor','k',...
                         'DisplayName',['Agent ',num2str(a)]);
                end
            end

            mfTitle = title([mfNames{mfIndex},' MF'],'Interpreter','latex',...
                'FontWeight','normal','FontSize',size_font_sub);
            mfTitle.Units='normalized';
            mfTitle.Position=[0.5, 0.8]; 
            xlabel('$\mathrm{Index}$','Interpreter','latex','FontSize',size_font_sub);
            ylabel('$\mathrm{Param}$','Interpreter','latex','FontSize',size_font_sub);

            if mfIndex==1
                % Only the first FLC subplot has the legend
                lg=legend('show','Location','northeast','Interpreter','latex');
                lg.FontSize = size_font;
            else
                legend off
            end
            hold off;
        end

        %% BOTTOM SUBPLOTS
        bottomHeight=0.45; 
        subH2 = bottomHeight/2;
        % (a) Objective
        subplot('Position',[0.05, 0.05+subH2, 0.90, subH2-0.05]); 
        box on; axis on; hold on;
        nObj = min(k, length(results.obj_hist));
        objData = [];
        if nObj>0
            objData = results.obj_hist(1:nObj);
            locMax = max(objData);
            if locMax>maxObjVal
                maxObjVal=locMax;
            end
        end
        if ~isempty(objData)
            plot(1:nObj, objData,'-','Color',objColor,'LineWidth',1.5);
        end
        for vlineX = 1 : config.dk_mpc : k
            xline(vlineX,'--','Color',[0,0,0],'LineWidth',1.0);
        end
        xlim([1, config.k]);
        ylim([0, max(1e-12, maxObjVal)]);
        ylabel('$J(k)$','Interpreter','latex','FontSize',size_font);
        objTitle = title('Simulation Parameters','FontSize',size_font,'FontWeight','normal');
        objTitle.Units='normalized';
        hold off;

        % (b) Optimization time
        subplot('Position',[0.05, 0.05, 0.90, subH2-0.05]);
        box on; axis on; hold on;
        optData = [];
        if k_mpc <= length(results.optimizationTimes)
            optData = results.optimizationTimes(1:k_mpc);
        elseif ~isempty(results.optimizationTimes)
            optData = results.optimizationTimes;
        end
        if ~isempty(optData)
            locMaxOpt = max(optData);
            if locMaxOpt>maxOptVal
                maxOptVal=locMaxOpt;
            end
            plot(1:length(optData), optData,'-o','Color',optColor,'LineWidth',1.5,...
                 'MarkerFaceColor',optColor,'MarkerEdgeColor','k','MarkerSize',6);
        end
        for i_line = 1 : config.dk_mpc : k
            i_mpcLine = ceil(i_line / config.dk_mpc);
            if i_mpcLine<=k_mpcMax
                xline(i_mpcLine,'--','Color',[0,0,0],'LineWidth',1.0);
            end
        end
        xlim([1, k_mpcMax]);
        ylim([0, max(1e-12, maxOptVal)]);
        ylabel('$t^{\mathrm{opt}}~\mathrm{(s)}$','Interpreter','latex','FontSize',size_font);
        hold off;

        %% "k = X" text at bottom center
        annotation('textbox',[0.45,0.0,0.1,0.04],...
                   'String',['$k = ',num2str(k),'$'],...
                   'Interpreter','latex','EdgeColor','none',...
                   'HorizontalAlignment','center','VerticalAlignment','middle',...
                   'FontSize',size_font);
        annotation('textbox',[0.55,0.95,0.3,0.04],...
                   'String',"FLC Parameters",...
                   'EdgeColor','none',...
                   'HorizontalAlignment','center','VerticalAlignment','middle',...
                   'FontSize',size_font);

        %% capture frame
        frame = getframe(gcf);

        % Convert to image and write to .gif
        im = frame2im(frame);
        [indexed,cmap] = rgb2ind(im,256);
        if k==1
            imwrite(indexed,cmap,output_filename,'gif','LoopCount',inf,'DelayTime',0.5);
        else
            imwrite(indexed,cmap,output_filename,'gif','WriteMode','append','DelayTime',0.5);
        end
    end

    close(gcf);
end
