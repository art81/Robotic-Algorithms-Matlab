function x_t = sample_motion_model_velocity(u_t, x_t_1, alpha, deltaT)
% u(t) = (v, w)T
% x(t) = (x, y, theta)T
v     = u_t(1);
w     = u_t(2);
x     = x_t_1(1);
y     = x_t_1(2);
theta = x_t_1(3);

std_dev1 = sqrt((alpha(1)*v^2) + (alpha(2)*w^2));
std_dev2 = sqrt((alpha(3)*v^2) + (alpha(4)*w^2));
std_dev3 = sqrt((alpha(5)*v^2) + (alpha(6)*w^2));

v_hat = v + normrnd(0,std_dev1);
w_hat = w + normrnd(0,std_dev2);
gamma_hat = normrnd(0,std_dev3);

theta_prime = theta + deltaT*(w_hat + gamma_hat);

%Wish I could deal with this better :(
if (w_hat == 0)
    x_prime = x + v_hat*deltaT*cos(theta);
    y_prime = y + v_hat*deltaT*sin(theta);
else
    x_prime = x - ((v_hat / w_hat)*sin(theta)) + ((v_hat / w_hat)*sin(theta + w_hat*deltaT));
    y_prime = y + ((v_hat / w_hat)*cos(theta)) - ((v_hat / w_hat)*cos(theta + w_hat*deltaT));
end

x_t = [x_prime, y_prime, theta_prime]';
end

