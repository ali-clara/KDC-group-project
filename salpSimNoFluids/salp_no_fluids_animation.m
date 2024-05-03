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
FPS = 60; % If your computer cannot plot in realtime lower this.

% For SE3
addpath(fullfile(pwd,'..', 'groupTheory'))
% For CubeClass and SpringClass
addpath(fullfile(pwd,'..', 'visualization'))

% Create objects
load_h = 0.25;
actuator_h = 0.35;
springObj = SpringClass(SE3, p.srl);
loadObj = CubeClass([2, load_h]);
actuatorObj = CubeClass([3, actuator_h]);

% Create a figure handle
h.figure = figure;
%This sets figure dimension which will dictate video dimensions
h.figure.Position(3:4) = [1280 720];
movegui(h.figure)

% Put the shapes into a plot
loadObj.plot
actuatorObj.plot
springObj.plot

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
    
    load_state = interp1(t',X(1,:)',t_plt);
    load_pos = load_state(1);

    actuator_state = interp1(t',X(2,:)',t_plt);
    actuator_pos = actuator_state(1);
    
    % Set axis limits (These will respect the aspect ratio set above)
    h.figure.Children(1).XLim = [-4, 4];
    h.figure.Children(1).YLim = [-15, 30];
    h.figure.Children(1).ZLim = [-1.0, 1.0];

    % Set the load position
    loadObj.resetFrame
    loadObj.globalMove(SE3([0, load_pos, 0]));
    
    % Set the actuator position
    actuatorObj.resetFrame
    actuatorObj.globalMove(SE3([0, actuator_pos, 0]))

    % Spring Position
    if load_pos < (actuator_pos + p.srl)
        spring_compression = p.srl - (load_pos - actuator_pos);
        springObj.updateState(SE3([0, actuator_pos, 0,0,0,pi/2]),...
                              p.srl-spring_compression)
    else
        springObj.updateState(SE3([0, actuator_pos, 0,0,0,pi/2]),p.srl)
    end
    
    % Update data
    loadObj.updatePlotData
    actuatorObj.updatePlotData
    springObj.updatePlotData
    
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