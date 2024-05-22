function salp_no_fluids_animation(p,t,X,exportVideo,playbackRate)
% Load-Spring-Damper-Actuator Animation 
% Input
%   p: Simulation constants
%   t: Simulation time vector
%   X: Simulation state vector
%   exportVideo: Should the video be exported? (True/False)
% Output
%   An animation/File
% By Kevin Green 2021

% FPS for playback and video export
FPS = 30; % If your computer cannot plot in realtime lower this.

% For SE3
addpath(fullfile(pwd,'..', 'groupTheory'))
% For CubeClass and SpringClass
addpath(fullfile(pwd,'..', 'visualization'))

% Create objects
front_cap_height = 0.002;
end_cap_height = 0.002;
capFrontObj = CubeClass([front_cap_height, front_cap_height*3]);
capEndObj = CubeClass([end_cap_height, end_cap_height*3]);
% springObj = SpringClass(SE3, p.srl);

% Create a figure handle
h.figure = figure;
%This sets figure dimension which will dictate video dimensions
h.figure.Position(3:4) = [1280 720];
movegui(h.figure)

% Put the shapes into a plot
capFrontObj.plot
capEndObj.plot
% springObj.plot

% Figure properties
view(2)
title('Simulation')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')
% These commands set the aspect ratio of the figure so x scale = y scale
% "Children(1)" selects the axes that contains the animation objects
h.figure.Children(1).DataAspectRatioMode = 'manual';
h.figure.Children(1).DataAspectRatio = [1 1 1];

% Setup videowriter object
if exportVideo
   v = VideoWriter('ME542_HW1_Animation.mp4', 'MPEG-4');
   v.FrameRate = FPS;
   open(v)
end

% Iterate over state data
tic;
for t_plt = t(1):playbackRate*1.0/FPS:t(end)
    
    % Set axis limits (These will respect the aspect ratio set above)
    h.figure.Children(1).XLim = [-0.1, 0.1];
    h.figure.Children(1).YLim = [-0.05, 0.05];
    h.figure.Children(1).ZLim = [-1.0, 1.0];

    % Interpolate and set the endcap position
    cap_end_state = interp1(t',X(1,:)',t_plt);
    cap_end_pos = cap_end_state(1);
    capEndObj.resetFrame
    capEndObj.globalMove(SE3([cap_end_pos,0, 0]));
    
    % Interpolate and set the frontcap position
    cap_front_state = interp1(t',X(2,:)',t_plt);
    cap_front_pos = cap_front_state(1);
    disp(cap_front_pos)
    capFrontObj.resetFrame
    capFrontObj.globalMove(SE3([cap_front_pos, 0, 0]))

    % Spring Position
    % Under compression
    % if cap_front_pos < (cap_end_pos + p.srl)
    %     spring_compression = p.srl - (cap_front_pos - cap_end_pos);
    %     springObj.updateState(SE3([cap_end_pos, 0, 0,0,0,pi/2]),...
    %                           p.srl-spring_compression)
    % % At spring resting length
    % else
    %     springObj.updateState(SE3([cap_end_pos, 0,0,0,0,pi/2]),p.srl)
    % end
    
    % Update data
    capFrontObj.updatePlotData
    capEndObj.updatePlotData
    % springObj.updatePlotData
    
    if exportVideo %Draw as fast as possible for video export
        drawnow
        frame = getframe(h.figure);
        writeVideo(v,frame);
    else % pause until 1/FPS of a second has passed then draw
        while( toc < 1.0/FPS)
            pause(0.002)
        end
        drawnow
        tic;
    end % if exportvideo
end % t_plt it = ...

end % paddleAnimation