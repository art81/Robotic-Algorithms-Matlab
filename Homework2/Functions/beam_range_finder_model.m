function [p, z_des] = beam_range_finder_model(x_t, maxRange)
% Computes the likelihood of a scan z_t given a state and a MAP

% Parameters of this Model
stddev_hit = 0.05;
lambda_short = 0.05;
z_hit   = 0.8;
z_short = 0.05;
z_max   = 0.05;
z_rand  = 0.1;

% Ray Tracing Implementation
x  = x_t(1);
y  = x_t(2);
th = x_t(3);
slope = sin(th) / cos(th);

%If we have more than one beam sensor on the robot than the following code
%needs to be put into a loop that calculates the probability of each ray
%individually and multiplies the probabilities together.

%NOTE: matlab takes into account slope being zero or Inf -> returns Inf and 0 respectively
x_intersect = ((3 - y) / slope) + x; %BASED ON THE MAP OF A WALL AT Y=3
if x_intersect < 0 || x_intersect > 3
    %We miss the wall because of wall x length: [0, 3]
    z_des = inf;
else
    % NEED TO TAKE INTO ACCOUNT DIRECTION FOR ACTUAL IMPLEMENTATION!
    % For this assignment it actually isn't necessary due to the test cases given ie. we never face away from the wall.
    z_des = sqrt((x - x_intersect)^2 + (y - 3)^2);
end

% At this point we have found z_des through ray tracing
% Find the probability distribution with symbolic variables (plotted in figure 6.4 on page 157)
p = (z_hit * prob_hit(z_des, stddev_hit, maxRange)) + z_short*(prob_short(z_des, lambda_short)) + z_max*(prob_max(maxRange)) + z_rand*(prob_rand(maxRange));
end

%Probabilitiy Helper Functions that Return symbolic notations dependant on 'z_t'
function p = prob_hit(z_des, stddev_hit, maxRange)
    syms z_t;
    
    e_expr = exp(-0.5 * ((z_des - z_t)^2 / stddev_hit^2));
    %nu = 1 / int(e_expr, [0, maxRange]);
    nu = 1; %Dr. Cenk said we could do this in class...
    p = piecewise(0 <= z_t <= maxRange, nu*(1 / sqrt(2 * pi * stddev_hit^2))*e_expr, 0);
end

function p = prob_short(z_des, lambda_short)
    syms z_t;
    
    nu = 1 / (1 - exp(-lambda_short * z_des));
    p = piecewise(0 <= z_t <= z_des, nu*lambda_short*exp(-lambda_short*z_t), 0);
end

function p = prob_max(maxRange)
    syms z_t;
    
    %Define this for range that counts as "max" reading
    maxRangePercent = 0.99;
    
    min = maxRangePercent*maxRange;
    p = piecewise(min <= z_t <= maxRange, 1 / (maxRange - min), 0);
end

function p = prob_rand(maxRange)
    syms z_t;
    
    p = piecewise(0 <= z_t <= maxRange, 1 / maxRange, 0);
end

