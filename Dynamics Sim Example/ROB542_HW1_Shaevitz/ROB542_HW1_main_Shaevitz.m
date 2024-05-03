%% ROB 542: Actuator Dynamics
% Assignment 1
% Name: Aiden Shaevitz
% Date: April 6, 2021

% Description: Simulate hybrid dynamics of actuator and inertial mass in a
% mass spring damper system
clc

%% setup
% Initial conditions  X0 = [x1; v1; x2; v2]
X0 = [2;  %load position
      0;  %load velocity
      -2;  %actuator position
      0]; %actuator velocity

% masses and constant force (gravity)
p.ml = 1; %kg
p.ma = 5; % kg
p.g = 9.8; %m/s^2
p.b  = 5;  % damper (N*s/m)
p.k  = 40; % spring (N/m)
p.srl = 2;  % Spring rest length (m)

% control input function
c.kp = 200;   
c.kd = 10;
c.setpoint = 2;
c.forceLim = 100;
c.use_setpoint = true; % If true, use setpoint instead of trajectory
% Set an arbitrary trajectory
trajectory_handle = @(t) sin(t); 
c.trajectory = trajHandles(trajectory_handle);


%% Run the simulation
[t_vec,X_vec] = ROB542_HW1_sim_Shaevitz(X0,p,c);


%% Plotting
% Add spring length for visual
X_vec(1,:) = X_vec(1,:) + p.srl;

% Plot load and actuator mass positions
plot(t_vec, X_vec(1,:)); hold on;
plot(t_vec, X_vec(3,:));


%% Animation
X = [X_vec(1,:); X_vec(3,:)];
ROB542_HW1_Animation_Shaevitz(p,t_vec,X,false,1);


%% Functions
% Get equation for derivative of trajectory for PD controller
function [funcs] = trajHandles(pos_func)
pos = sym(pos_func);
vel = diff(pos);
vel_func = matlabFunction(vel);
funcs.pf = pos_func;
funcs.vf = vel_func;
end