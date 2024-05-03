function [force] = salp_no_fluids_controller(t,X,c)
assert( isfield(c,'kp'));
assert( isfield(c,'kd'));
assert( isfield(c,'setpoint'));
if c.use_setpoint
    force = c.kp*(c.setpoint - X(1)) + c.kd*(0 - X(2));
else
    x_des = c.trajectory.pf(t);
    v_des = c.trajectory.vf(t);
    
    % PD force calculation
    force = c.kp*(x_des - X(1)) + c.kd*(v_des - X(2));
end


end
