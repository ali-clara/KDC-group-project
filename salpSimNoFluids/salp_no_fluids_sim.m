function [t_vec,X_vec,sol_set] = salp_no_fluids_sim(X0,p,c)

% Simulation timespan
t_start = 0;
t_end = 20;
dt = 0.01;

% Bind dynamics and control functions
controlForce = @(t,X) max(-c.forceLim, min(c.forceLim, ROB542_HW1_controller_Shaevitz(t,X,c)));
untDynamics = @(t,X) untetheredDynamics(t,X,p,controlForce);
tethDynamics = @(t,X) tetheredDynamics(t,X,p,controlForce);
contact_fun = @(t,X) contactEvent(t,X,p);
liftoff_fun = @(t,X) liftoffEvent(t,X,p);

% Simulation tolerances
options1 = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events', contact_fun);

options2 = odeset(...
    'RelTol', 1e-9, ...
    'AbsTol', 1e-9, ...
    'Events', liftoff_fun);

% Setup data structures
t_vec = t_start:dt:t_end;
X_vec = zeros(length(X0), length(t_vec));
sol_set = {};
mode = 1;

% Loop simulation until we reach t_end
while t_start < t_end
    % Run the simulation until t_end or a contact event
    if mode == 1 % Untethered - Load is not in contact
        sol = ode45(untDynamics, [t_start,t_end], X0, options1);
        mode = 2;
        X0 = sol.y(:,end);
    else % Tethered - Load is in contact with the spring
        sol = ode45(tethDynamics, [t_start,t_end], X0, options2);
        mode = 1;
        X0 = sol.y(:,end);
    end
    
    % Concatenate the last ode45 result onto the sol_set cell array
    sol_set = [sol_set, {sol}]; %#ok<AGROW>
    
    % Setup t_start for the next ode45 call
    t_start = sol.x(end);
    
end

for idx = 1:length(sol_set)
    % This sets up a logical vector so we can perform logical indexing
    t_sample_mask = t_vec >= sol_set{idx}.x(1) & t_vec <= sol_set{idx}.x(end);
    % Evaluate the idx solution structure only at the applicable times
    X_eval = deval(sol_set{idx}, t_vec(t_sample_mask));
    % Assign the result to the correct indicies of the return state array
    X_vec(:,t_sample_mask) = X_eval;
end

end

function dX = untetheredDynamics(t,X,p,controller)
    dX = zeros(4,1);

    % Load mass
    dX(1) = X(2); % Vel
    dX(2) = -p.g; % Accel

    % Actuator
    dX(3) = X(4); % Vel
    dX(4) = -p.g + controller(t,[X(3) X(4)])/p.ma; % Accel
end

function dX = tetheredDynamics(t,X,p,controller)
    dX = zeros(4,1);

    % Load mass
    dX(1) = X(2); % Vel
    dX(2) = p.k/p.ml*(X(3)-X(1)) + p.b/p.ml*(X(4)-X(2)) - p.g; % Accel

    % Actuator
    dX(3) = X(4); % Vel
    dX(4) = p.k/p.ma*(X(1)-X(3)) + p.b/p.ma*(X(2)-X(4)) ... % Accel
           -p.g + controller(t,[X(3) X(4)])/p.ma;
end

function  [eventVal,isterminal,direction] = contactEvent(t,X,p)
    val = X(1) - X(3);

    eventVal = val;

    isterminal = 1;
    direction = -1;
end

function  [eventVal,isterminal,direction] = liftoffEvent(t,X,p)
    accel_on_load = p.k*(X(3)- X(1)) + p.b*(X(4)-X(2));
    eventVal = accel_on_load;
    isterminal = 1;
    direction = 0;
end

