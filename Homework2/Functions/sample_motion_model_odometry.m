function x_t = sample_motion_model_odometry(u_t, x_t_1, alpha)
% u(t) = (x_bar_t-1, x_bar_t)T
% where x_bar_t-1 = (x, y, theta BAR) and x_bar_t = x, y, theta BAR PRIME
% x(t) = (x, y, theta)T

x_bar_t_1 = u_t(1, :);
x_bar_t   = u_t(2, :);

x  = x_t_1(1);
y  = x_t_1(2);
th = x_t_1(3); 

x_bar  = x_bar_t_1(1);
y_bar  = x_bar_t_1(2);
th_bar = x_bar_t_1(3);
x_bar_prime  = x_bar_t(1);
y_bar_prime  = x_bar_t(2);
th_bar_prime = x_bar_t(3);

rot1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - th_bar;
tran = sqrt((x_bar - x_bar_prime)^2 + (y_bar - y_bar_prime)^2);
rot2 = th_bar_prime - th_bar - rot1;

%Standard deviations used for sampling
std_dev1 = sqrt((alpha(1)*rot1^2) + (alpha(2)*tran^2));
std_dev2 = sqrt((alpha(3)*tran^2) + (alpha(4)*rot1^2) + (alpha(4)*rot2^2));
std_dev3 = sqrt((alpha(1)*rot2^2) + (alpha(2)*tran^2));

rot1_hat = rot1 - normrnd(0,std_dev1);
tran_hat = tran - normrnd(0,std_dev2);
rot2_hat = rot2 - normrnd(0,std_dev3);

x_prime  = x + (tran_hat*cos(th + rot1_hat));
y_prime  = y + (tran_hat*sin(th + rot1_hat));
th_prime = th + rot1_hat + rot2_hat;

x_t = [x_prime, y_prime, th_prime]';
end

