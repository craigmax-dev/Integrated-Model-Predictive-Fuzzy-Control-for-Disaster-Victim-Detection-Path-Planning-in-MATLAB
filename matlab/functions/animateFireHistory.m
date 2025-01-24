function animateFireHistory(m_f_hist, config, output_filename_mp4, output_filename_gif)
%ANIMATEFIREHISTORY Generates and saves animations of a fire history using a custom discrete colormap.
%
% PARAMETERS:
%   m_f_hist            - 3D matrix representing fire history (rows x cols x frames)
%   config              - Configuration structure with coarsening factor (c_f_s)
%   output_filename_mp4 - Filename for the output MP4 video (e.g., 'fire_animation.mp4')
%   output_filename_gif - Filename for the output GIF animation (e.g., 'fire_animation.gif')
%
% EXAMPLE:
%   animateFireHistory(environment_model.m_f_hist, config, 'fire.mp4', 'fire.gif')

    % -------------------------------
    % Validate inputs
    % -------------------------------
    if nargin < 4
        error('Insufficient input arguments. Provide m_f_hist, config, output_filename_mp4, and output_filename_gif.');
    end

    % -------------------------------
    % Create VideoWriter for MP4
    % -------------------------------
    writerObj = VideoWriter(output_filename_mp4, 'MPEG-4');
    writerObj.FrameRate = 2;  % Adjust frame rate as needed
    open(writerObj);

    % -------------------------------
    % Configure GIF parameters
    % -------------------------------
    delay_time = 0.5; % Delay time (sec) between frames in GIF
    loop_count = inf; % Infinite loop for GIF

    % -------------------------------
    % Prepare the figure (off-screen)
    % -------------------------------
    figure('Visible','off','Color','w');

    % Define a custom discrete colormap for the fire map
    myCmap = [
        0.8,   0.8,   0.8;    % 0: non-flammable (light gray)
        0.9,   0.95,  1.0;    % 1: flammable (pale blue)
        1.0,   0.498, 0.0;    % 2: catching fire (orange)
        0.894, 0.102, 0.110;  % 3: burning (red)
        0.0,   0.0,   0.0     % 4: extinguished (black)
    ];

    % -------------------------------
    % Determine number of frames
    % -------------------------------
    numFrames = size(m_f_hist, 3);

    for k = 1:numFrames
        clf;  % Clear axes for each frame

        % Extract fire map for current frame
        m_f = m_f_hist(:, :, k);

        % Coarsen if specified
        if config.c_f_s > 1
            m_f_coarsened = func_coarsen(m_f, config.c_f_s);
        else
            m_f_coarsened = m_f;
        end

        % Plot the coarsened fire map
        imagesc(m_f_coarsened, [0 4]); 
        axis equal tight;
        set(gca,'YDir','normal');
        colormap(myCmap);
        c = colorbar('Location','eastoutside');
        c.Ticks = 0:4;
        xlabel('$x$ cell index','Interpreter','latex');
        ylabel('$y$ cell index','Interpreter','latex');
        title(['$k = ',num2str(k),'$'],'Interpreter','latex', 'FontSize', 16);
        
        drawnow;

        % -------------------------------
        % Capture frame for MP4
        % -------------------------------
        frame = getframe(gcf);
        writeVideo(writerObj, frame);

        % -------------------------------
        % Convert frame to image for GIF
        % -------------------------------
        img = frame2im(frame);
        [imind, cm] = rgb2ind(img, 256);

        % Write the GIF
        if k == 1
            imwrite(imind, cm, output_filename_gif, 'gif', ...
                    'Loopcount', loop_count, 'DelayTime', delay_time);
        else
            imwrite(imind, cm, output_filename_gif, 'gif', ...
                    'WriteMode','append','DelayTime',delay_time);
        end
    end

    % -------------------------------
    % Close the video file and figure
    % -------------------------------
    close(writerObj);
    close(gcf);

    fprintf('Animation successfully saved as "%s" and "%s".\n', ...
            output_filename_mp4, output_filename_gif);
end
