function p = motion_model_odometry(x_t,u_t, x_t_1, alpha)
% u(t) = (x_bar_t-1, x_bar_t)T
% where x_bar_t-1 = (x, y, theta BAR) and x_bar_t = x, y, theta BAR PRIME
% x(t) = (x, y, theta)T
x_bar_t_1 = u_t(1, :);
x_bar_t   = u_t(2, :);

x  = x_t_1(1);
y  = x_t_1(2);
th = x_t_1(3);
x_prime  = x_t(1);
y_prime  = x_t(2);
th_prime = x_t(3);

x_bar  = x_bar_t_1(1);
y_bar  = x_bar_t_1(2);
th_bar = x_bar_t_1(3);
x_bar_prime  = x_bar_t(1);
y_bar_prime  = x_bar_t(2);
th_bar_prime = x_bar_t(3);

rot1 = atan2(y_bar_prime - y_bar, x_bar_prime - x_bar) - th_bar;
tran = sqrt((x_bar - x_bar_prime)^2 + (y_bar - y_bar_prime)^2);
rot2 = th_bar_prime - th_bar - rot1;

rot1_hat = atan2(y_prime - y, x_prime - x) - th;
tran_hat = sqrt((x - x_prime)^2 + (y - y_prime)^2);
rot2_hat = th_prime - th - rot1_hat;

%Standard deviations used for sampling
std_dev1 = sqrt((alpha(1)*rot1_hat^2) + (alpha(2)*tran_hat^2));
std_dev2 = sqrt((alpha(3)*tran_hat^2) + (alpha(4)*rot1_hat^2) + (alpha(4)*rot2_hat^2));
std_dev3 = sqrt((alpha(1)*rot2_hat^2) + (alpha(2)*tran_hat^2));

p =  prob(rot1 - rot1_hat, std_dev1^2) * prob(tran - tran_hat, std_dev2^2) * prob(rot2 - rot2_hat, std_dev3^2);
end

% Probability of a value 'a' occuring with a zero-centered normal distribution
% with variance as an input.
function p = prob(a, var)
    p = (1 / sqrt(2 * pi * var)) * exp(-a^2 / (2*var));
end