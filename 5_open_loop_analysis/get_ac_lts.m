global x_a;
altitude = 40000; 
velocity = 300;
x_a = 0;
simulink_block = 'LIN_F16Block_trim';

% Linearize the system for the conditions given above.
find_f16_dynamics_lofi;

% Create the longitudinal aircraft LTI
% States: velocity, alpha, theta, q
% Input: Elevator
SS_lo
ss = modred(SS_lo, [1 2 3 4 6 9 10 12 13 15 16 17 18], "Truncate")
% xperm(ss, [3 2 4 1 5])
A_lon = A_lo([7 8 5 11 14], [7 8 5 11 14])
A_lon_ac = A_lon(1:4, 1:4)
B_lon_ac = A_lon(1:4, 5)
eig(A_lon_ac)

% Create the lateral aircraft LTI
% States: beta, phi, p, r
% Inputs: Aileron, rudder
A_lat = A_lo([9 4 10 12, 15, 16], [9 4 10 12, 15, 16])

A_lat_ac = A_lat(1:4, 1:4)
B_lat_ac = A_lat(1:4, 5:6)



