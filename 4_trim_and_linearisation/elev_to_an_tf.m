% Get the transfer function s-power coefficients for all outputs w.r.t.
% elevator input (input-2)
[tf_num, tf_den] = ss2tf(A_lo, B_lo, C_lo, D_lo, 2);

% Get tranfer function numerator coefficients for the accelerometer.
tf_num = tf_num(19, :);

% Create and display the tranfer function
tf_elev_an = tf(tf_num, tf_den)
zero(tf_elev_an)

