function p = motion_model_velocity(x_t,u_t, x_t_1, alpha, deltaT)
% u(t) = (v, w)T
% x(t) = (x, y, theta)T
x = x_t_1(1);
y = x_t_1(2);
th = x_t_1(3);
v = u_t(1);
w = u_t(2);
x_prime = x_t(1);
y_prime = x_t(2);
th_prime = x_t(3);

std_dev1 = sqrt((alpha(1)*v^2) + (alpha(2)*w^2));
std_dev2 = sqrt((alpha(3)*v^2) + (alpha(4)*w^2));
std_dev3 = sqrt((alpha(5)*v^2) + (alpha(6)*w^2));

mu = 0.5 * (((x - x_prime)*cos(th)) + ((y - y_prime)*sin(th))) / (((y - y_prime)*cos(th)) - ((x - x_prime)*sin(th)));
x_star = (0.5*(x + x_prime)) + mu*(y - y_prime);
y_star = (0.5*(y + y_prime)) + mu*(x_prime - x);
r_star = sqrt((x - x_star)^2 + (y - y_star)^2);

delta_theta = atan2(y_prime - y_star, x_prime - x_star) - atan2(y - y_star, x - x_star);

v_hat = (delta_theta / deltaT) * r_star;
w_hat = delta_theta / deltaT;
gamma_hat = ((th_prime - th) / deltaT) - w_hat;
p =  prob(v - v_hat, std_dev1^2) * prob(w - w_hat, std_dev2^2) * prob(gamma_hat, std_dev3^2);
end

% Probability of a value 'a' occuring with a zero-centered normal distribution
% with variance as an input.
function p = prob(a, var)
    p = (1 / sqrt(2 * pi * var)) * exp(-0.5 * (a^2 / var));
end