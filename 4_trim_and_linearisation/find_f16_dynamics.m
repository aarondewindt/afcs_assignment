function [model] = find_f16_dynamics(fidelity, altitude, velocity, trim_iterations)
    global fi_flag_Simulink

    %% Initial guess for trim
    thrust = 5000;          % thrust, lbs
    elevator = -0.09;       % elevator, degrees
    alpha = 8.49;              % AOA, degrees
    rudder = -0.01;             % rudder angle, degrees
    aileron = 0.01;            % aileron, degrees
    
    %% Trim ad desired altitude and velocity
    fi_flag_Simulink = fidelity;
    [trim_state, trim_thrust, trim_control, dLEF, xu, cost] = ... 
        trim_f16_modified(thrust, elevator, alpha, aileron, ...
                          rudder, velocity, altitude, trim_iterations, 1);
    
    %% Find state space model
    trim_state_lin = trim_state; 
    trim_thrust_lin = trim_thrust; 
    trim_control_lin = trim_control;
    
    assignin('base','trim_state_lin', trim_state)
    assignin('base','trim_thrust_lin', trim_thrust)
    assignin('base','trim_control_lin', trim_control)
    assignin('base','dLEF', dLEF)
    assignin('base','fi_flag_Simulink', fidelity)    
    [A, B, C, D] = linmod(...
        'LIN_F16Block', ...
        [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); ...
         trim_control_lin(3); dLEF; -trim_state_lin(8)*180/pi], ...
        [trim_thrust_lin; trim_control_lin(1); ...
         trim_control_lin(2); trim_control_lin(3)]);

    %% Make systems
    ss_lon = longitudunal(A, B, C, D);
    ss_lat = lateral(A, B, C, D);
       
    model = struct();    
    model.ss_lon = ss_lon;
    model.ss_lat = ss_lat;
    model.cost = cost;
    model.thrust = xu(13);
    model.delta_e = xu(14);
    model.delta_a = xu(15);
    model.delta_r = xu(16);
    model.delta_lef = xu(17);
    model.alpha = xu(8)*180/pi    ;
    model.altitude = altitude;
    model.velocity = velocity;
end

function [SS] = longitudunal(A, B, C, D)
    mat = [A B; C D];    
    A_lon = mat([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
    B_lon = mat([3 5 7 8 11 13 14], [19 20]);
    C_lon = mat([21 23 25 26 29], [3 5 7 8 11 13 14]);
    D_lon = mat([21 23 25 26 29], [19 20]);
    SS = ss(A_lon, B_lon, C_lon, D_lon);
end


function [SS] = lateral(A, B, C, D)
    mat = [A B; C D];    
    A_lat = mat([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);
    B_lat = mat([3 5 7 8 11 13 14], [19 20]);
    C_lat = mat([21 23 25 26 29], [3 5 7 8 11 13 14]);
    D_lat = mat([21 23 25 26 29], [19 20]);
    SS = ss(A_lat, B_lat, C_lat, D_lat);
end