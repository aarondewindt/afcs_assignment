function [model] = find_f16_dynamics(system_name, fidelity, altitude, velocity, trim_iterations)
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
    
    % Write variables to the workspace to the simulink model can read them
    assignin('base','trim_state_lin', trim_state)
    assignin('base','trim_thrust_lin', trim_thrust)
    assignin('base','trim_control_lin', trim_control)
    assignin('base','dLEF', dLEF)
    assignin('base','fi_flag_Simulink', fidelity)
    
    [A, B, C, D] = linmod(...
        system_name, ...
        [trim_state_lin; trim_thrust_lin; trim_control_lin(1); trim_control_lin(2); ...
         trim_control_lin(3); dLEF; -trim_state_lin(8)*180/pi], ...
        [trim_thrust_lin; trim_control_lin(1); ...
         trim_control_lin(2); trim_control_lin(3)]);
         
    %% Create systems
    ss_total = ss(A, B, C, D);
    
    [ss_lon, ss_lon_ac] = longitudunal(A, B, C, D);
    [ss_lat, ss_lat_ac] = lateral(A, B, C, D);
    
    %% Structure results
    model = struct();
    model.ss = ss_total;
    model.ss_lon = ss_lon;
    model.ss_lon_ac = ss_lon_ac;
    model.ss_lat = ss_lat;
    model.ss_lat_ac = ss_lat_ac;
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

function [ss_lon, ss_lon_ac] = longitudunal(A, B, C, D)  
    mat = [A B; C D]; 
    states_idxs = [7 8 5 11 14];
    inputs_idxs = 20;
       
    A_lon = mat(states_idxs, states_idxs);
    B_lon = mat(states_idxs, inputs_idxs);
    C_lon = eye(4, 5);
    D_lon = zeros(4, 1);
    ss_lon = ss(A_lon, B_lon, C_lon, D_lon, ...
                'StateName', {'v', 'alpha', 'theta', 'q', 'delta_e'},...
                'InputName', {'delta_e'},...
                'OutputName', {'v', 'alpha', 'theta', 'q'});
    
            
    A_lon_ac = A_lon(1:4, 1:4);
    B_lon_ac = A_lon(1:4, end);
    C_lon_ac = eye(4);
    D_lon_ac = zeros(4, 1);
    ss_lon_ac = ss(A_lon_ac, B_lon_ac, C_lon_ac, D_lon_ac, ...
                   'StateName', {'v', 'alpha', 'theta', 'q'},...
                   'InputName', {'delta_e'},...
                   'OutputName', {'v', 'alpha', 'theta', 'q'});
end

function [ss_lat, ss_lat_ac] = lateral(A, B, C, D)
    mat = [A B; C D]; 
    states_idxs = [9 4 10 12 15 16];
    inputs_idxs = [21 22];
    
    A_lat = mat(states_idxs, states_idxs);
    B_lat = mat(states_idxs, inputs_idxs);
    C_lat = eye(4, 6);
    D_lat = zeros(4, 2);
    ss_lat = ss(A_lat, B_lat, C_lat, D_lat, ...
                'StateName', {'beta', 'phi', 'p', 'r', ...
                              'delta_a', 'delta_r'},...
                'OutputName', {'beta', 'phi', 'p', 'r'});
            
    A_lat_ac = A_lat(1:4, 1:4);
    B_lat_ac = A_lat(1:4, 5:6);
    C_lat_ac = eye(4);
    D_lat_ac = zeros(4, 2);
    ss_lat_ac = ss(A_lat_ac, B_lat_ac, C_lat_ac, D_lat_ac, ...
                'StateName', {'beta', 'phi', 'p', 'r'},...
                'InputName', {'delta_a', 'delta_r'},...
                'OutputName', {'beta', 'phi', 'p', 'r'});
end
















