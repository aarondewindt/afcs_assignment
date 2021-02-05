time_vector = 0:0.001:0.5;
u_elev = -ones(1, length(time_vector));

[y, t] = lsim(tf_elev_an, u_elev, time_vector);

plot(t, y)
xlabel('time [s]')
ylabel('a_n [g units]')
title('Normal acceleration due to elevtor step response.')
set(gcf,'units','points','position',[0,0, 600, 400])