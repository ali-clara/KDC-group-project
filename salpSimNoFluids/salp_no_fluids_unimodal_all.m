% Name: Aiden Shaevitz
% Date: 5/7/2024
% Description: Test version of dynamics with all setup, sim functions,
% controller functions, and animation functions baked into one script

clc;
clearvars;

%% Setup
% Initial conditions  X0 = [x1; v1; x2; v2]
X0 = [0;  % nozzle position
      0;  % nozzle velocity
      0.1; % base position (should be relaxed length of origami)
      0]; % base velocity

% masses and constant force (gravity)
p.m_nozzle = 1; %kg
p.m_base = 1; % kg
p.g = 9.81; % not really necessary (m/s^2)
p.b_fluid  = 0;  % temp fluid damping (N*s/m)
p.k_origami  = 5; % spring (N/m)
p.srl = 0.1;  % Spring rest length, should match base position (relaxed length of origami spring) (m)

% control input function (Not really going to have any feedback, but this
% is where we could assign the relevant TCA actuation parameters
c.TCA_stuff = "something";
% c.kp = 200;   
% c.kd = 10;
% c.setpoint = 2;
c.forceLim = 100;
% c.use_setpoint = true; % If true, use setpoint instead of trajectory
% Set an arbitrary trajectory
% trajectory_handle = @(t) sin(t); 
% c.trajectory = trajHandles(trajectory_handle);


%% Run the simulation
[t_vec,X_vec] = ROB542_HW1_sim_Shaevitz(X0,p,c);

%% Dynamics
function dX = contractionDynamics(t,X,p,controller)
    dX = zeros(4,1);

    % Load mass
    dX(1) = X(2); % Vel
    dX(2) = "add in first equation here" % Accel

    % Actuator
    dX(3) = X(4); % Vel
    dX(4) = "add in second equation here"
end