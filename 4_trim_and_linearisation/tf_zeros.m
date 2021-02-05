global x_a;
altitude = 15000; 
velocity = 500;
simulink_block = 'LIN_F16Block_trim';

styles = ["-", "--", "-.", "-", "--", "-.";
            1,    1,   1,    2,   2,    2;
          "o",  "+", "x",  "s", "d",  "p";    
];

x_a_options = [0, 5, 5.9, 6, 7, 15];
time_vector = 0:0.001:0.1;

figure(1)
clf;
grid on
hold on;

figure(2)
clf;
grid on
hold on;

figure(3)
clf;
grid on
hold on;

for idx = 1:length(x_a_options)
    x_a = x_a_options(idx) * 0.3048;
    
    fprintf('Running x_a=%f\n', x_a)
    % Linearize system
    find_f16_dynamics_lofi
    
    % Get the transfer function s-power coefficients for all outputs w.r.t.
    % elevator input (input-2)
    [tf_num, tf_den] = ss2tf(A_lo, B_lo, C_lo, D_lo, 2);

    % Get tranfer function numerator coefficients for the accelerometer.
    tf_num = tf_num(19, :);

    % Create and display the tranfer function
    tf_elev_an = tf(tf_num, tf_den);
    
    % Plot zeros
    figure(1)
    zs = zero(tf_elev_an);
    plot(zs, char(styles(3, idx)));
    hold on
    
    figure(3)
    plot(zs, char(styles(3, idx)));
    hold on
    
    % Plot step response
    figure(2)
    u_elev = -ones(1, length(time_vector));
    [y, t] = lsim(tf_elev_an, u_elev, time_vector);
    plot(t, y, char(styles(1, idx)), 'LineWidth', double(styles(2, idx)))
    
end

lbs = [];
for idx=1:length(x_a_options)
    lbs = [lbs sprintf("x_a=%f [ft]", x_a_options(idx))];
end

figure(1)
xlabel('real')
ylabel('imagenary')
title('Elevator to normal acceleration tranfer function zeros.')
set(gcf,'units','points','position',[0,0, 600, 400])
legend(lbs, 'Location','northwest')
ylim([-10, 10])

figure(3)
xlabel('real')
ylabel('imagenary')
title('Elevator to normal acceleration tranfer function zeros zoomed in.')
set(gcf,'units','points','position',[0,0, 600, 400])
legend(lbs, 'Location','northwest')
xlim([-20, 45])
ylim([-10, 10])


figure(2)
xlabel('time [s]')
ylabel('a_n [g units]')
title('Normal acceleration elevator step response for different x_a values.')
set(gcf,'units','points','position',[0,0, 600, 400])
legend(lbs)

