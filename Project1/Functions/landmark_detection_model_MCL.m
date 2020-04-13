function p = landmark_detection_model_MCL(z_t, x_t, m, sigma_r, sigma_phi)
% Gives a probability based on the current map, current x value, and the
% sensor output

% Parameters of the Model
var_r   = sigma_r^2;
var_phi = sigma_phi^2;

robotPos = x_t(1:2);
robotPhi = x_t(3);

currR   = z_t(1);
currPhi = z_t(2) + robotPhi;
currC   = z_t(3);

% We have a valid measurement
landmarkPos = m(currC, :);

diff = landmarkPos - robotPos';
desR   = norm(diff);
desPhi = atan2(diff(2), diff(1));

% Deal with angle wraparound
phiDiff = desPhi - currPhi;
phiDiff = mod(phiDiff, 2 * pi);

if phiDiff < -pi
    phiDiff = phiDiff + (2 * pi);
elseif phiDiff >= pi
    phiDiff = phiDiff - (2 * pi);
end

p = prob(desR - currR, var_r)*prob(phiDiff, var_phi);

end

% Probability of a value 'a' occuring with a zero-centered normal distribution
% with variance as an input.
function p = prob(a, var)
    p = (1 / sqrt(2 * pi * var)) * exp(-0.5 * (a^2 / var));
end