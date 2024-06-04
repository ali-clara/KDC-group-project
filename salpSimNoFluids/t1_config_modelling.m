clear all; close all; clc

%% Inputs 
close all; clc

% load input force ("force") and time ("time)
load experimental-data\3W_air_force.mat;
input_force = force / 1000; % N
input_t = time - time(1); % sec
data_freq = time(3) - time(2);

% slice the data into heating and cooling
power_on_t = 4.3;
power_off_t = 14.3;
% power_off_t = 6;
power_on_idx = cast(power_on_t/data_freq, "uint8");
power_off_idx = cast(power_off_t/data_freq, "uint8");

heating_t = input_t(power_on_idx:power_off_idx);
heating_t = heating_t - heating_t(1);
heating_force = input_force(power_on_idx:power_off_idx);

cooling_t = input_t(power_off_idx:end);
cooling_t = cooling_t - cooling_t(1);
cooling_force = input_force(power_off_idx:end);

plot(heating_t, heating_force)
hold on
plot(cooling_t, cooling_force)


%% T1 Thruster Configuration
close all; clc

tca_num = 4; % number of TCAs in the current configuration

b_origami = 300; % damper (N*s/m) - found from system identification in air
k_origami = 98.1; % spring (N/m) - 0.004" plastic
% k_origami = 18.25 % spring (N/m) - 0.002" plastic

m_cap = 0.015; % mass (kg)
% m_added = 0.065; % added water term
m_added = 0;
m_total = m_cap + m_added;

k_tca = tca_num*25.4; % (N/m) - spring constant of TCAs between 0-4mm, becomes more nonlinear after that
b_tca = k_tca; % In water, cooling time constant = 1 (tau = B/K)

%% Solve with STEP
close all; clc

% Set up the state matrices
A = [-b_origami/m_cap, -1/m_cap; k_origami, 0];
B = [1/m_cap; 0];
C = [0, 1/k_origami; 1, 0];
D = [0; 0];

sys = ss(A, B, C, D);
step_tfinal = 20;
[y_step, t_step] = step(sys, tfinal);
plot(t_step, y_step)

%% Solve with ODE45 
% (should be the same as above if x0=[0,0] and f=[1])
close all; clc

% Heating response
x0 = [0; 0];
tfinal = 5.5;
tspan = [0, tfinal];

% find the state variables (x)
% tca_num = 4;
[t_heating,x_heating] = ode45(@(t,x) odefcn_heating(t,x,m_total,b_origami,k_origami,heating_force,heating_t,tca_num), tspan, x0);

% use the state variables (x) to find the output variables (y)
sz = size(x_heating); 
y_heating = zeros(sz); 
y_heating(:,1) = (1/k_origami)*x_heating(:,2); % displacement of the mass
y_heating(:,2) = x_heating(:,1); % velocity of the mass

% Cooling response
x0 = [x_heating(end,1); -x_heating(end,2); x_heating(end,2)]; % assuming F_tca = -F_origami at t=0
tfinal = 15;
tspan = [0, tfinal];

% find the state variables (x)
[t_cooling, x_cooling] = ode45(@(t,x) odefcn_cooling(t, x, m_total, b_origami, k_origami, b_tca, k_tca), tspan, x0);

% use the state variables (x) to find the output variables (y)
sz = size(x_cooling); 
y_cooling = zeros(sz); 
y_cooling(:,1) = (1/k_origami)*x_cooling(:,3); % displacement of the mass
y_cooling(:,2) = x_cooling(:,1); % velocity of the mass

%% Plots
close all; clc

% load real data
load experimental-data\3W_air_length.mat
y = cast(Displacement, "double"); % mm
t = Time; % sec
data_freq = Time(3) - Time(2);

% some trimming to the area of interest
y = y(15:end);
t = t(15:end);
t = t - t(1);

% % slice to area of interest and rescale
% y = y(240:600);
% t = t(240:600) - t(240);
% y = (y-y(1))*1.33+y(1); % adjust for water magnification
% y = y / y(1) * 51; % rescale from px to mm

% figure
plot(t, y(1)-y)
hold on
plot(t_heating, y_heating(:,1)*1000)
plot(t_cooling+5.5, y_cooling(:,1)*1000)
% plot(t_cooling+8, y_cooling(:,1)*1000)
xlabel("Time (sec)")
ylabel("X_{mass} (mm)")
title("Displacement of Cap")
legend("Data", "Contraction Model", "Expansion Model")

figure
subplot(1,2,1)
plot(t_heating, y_heating(:,2)*1000)
xlabel("Time (sec)")
ylabel("V_{mass} (mm/s)")
title("Velocity of Cap: Contraction")

% subplot(1,2,2)
% plot(t_cooling, y_cooling(:,2)*1000)
% xlabel("Time (sec)")
% ylabel("V_{mass} (mm/s)")
% title("Velocity of Cap: Expansion")

%% Animation
p.srl = 0.04;
% Position of end cap is just at zero for now
fixed_cap_pos = [zeros(size(y_heating(:,1))); zeros(size(y_cooling(:,1)));];

% Concatenation of heating and cooling positions
moving_cap_pos = [-y_heating(:,1)+p.srl ; -y_cooling(:,1)+p.srl];
t_all = [t_heating; t_cooling+t_heating(end)+0.0000001]; % Need to add the 0.0000001 to make the time values all unique for the interpolation in the animation

% Horizontally concatenation fixed end cap and front cap positions
X = [fixed_cap_pos moving_cap_pos];

% Animate
salp_no_fluids_animation(p,t_all,X,true,1);

%% Functions

function dxdt = odefcn_heating(t, x, m, b, k, input_force, input_t, tca_num)
    f_t = interp1(input_t, input_force, t, 'linear', 'extrap');
    dxdt = zeros(2,1);
    dxdt(1) = (-b/m)*x(1) + (-1/m)*x(2) + (1/m)*tca_num*f_t;
    dxdt(2) = k*x(1);
end

function dxdt = odefcn_cooling(t, x, m, b_origami, k_origami, b_tca, k_tca)
    dxdt = zeros(3,1);
    dxdt(1) = (-b_origami/m)*x(1) + (-1/m)*x(2) + (-1/m)*x(3);
    dxdt(2) = k_tca*x(1) + (-k_tca/b_tca)*x(2);
    dxdt(3) = k_origami*x(1);
end

function input_force = ft(f_in, t_in, t_desired)
    tol = 0.2;
    idx = find_index_near(t_in, t_desired, tol);
    input_force = f_in(idx);
end

function idx = find_index_near(vector, value, tol)
    idx = -100;
    for i = 1:length(vector)
        if abs(vector(i) - value) < tol
            idx = i;
            break
        end
    end
    if idx == -100
        disp("Could not find an index within the given tolerance")
    end
end