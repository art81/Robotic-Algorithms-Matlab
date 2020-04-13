function [mu_t, sigma_t, mu_t_motion, sigma_t_motion] = EKF_localization_known_correspondence(mu_t_1, sigma_t_1, ut, zt, m, sigma_r, sigma_phi, alpha)
% mu_t_1    : Means from previous time step
% sigma_t_1 : Covariance matrix from previous time step
% ut        : Motion Command
% zt        : Measurement
% m         : Map
% sigma_r   : Std Dev of Measurement Model for R
% sigma_phi : Std Dev of Measurement Model for Phi

% Implementation of the Extended kalman Filter with Known Correspondence in
% a feature based map
deltaT = 1; %second

%prevMu_x  = mu_t_1(1);
%prevMu_y  = mu_t_1(2);
prevMu_th = mu_t_1(3);

vt = ut(1);
wt = ut(2);

if wt == 0
    wt = 0.0001;
end

%meas_R   = zt(1);
%meas_Phi = zt(2);
meas_C   = zt(3);

% Reference for this part is Table 7.2 in PR

%% Motion Step
th = prevMu_th;
th_prime = th + (wt * deltaT);

temp1 = (-1 * vt * cos(th) / wt) + (vt / wt * cos(th_prime));
temp2 = (-1 * vt * sin(th) / wt) + (vt / wt * sin(th_prime));
G = [[1, 0, temp1];...
     [0, 1, temp2];...
     [0, 0, 1]];

temp1 = (-sin(th) + sin(th_prime)) / (wt);
temp2 = (cos(th) - cos(th_prime)) / (wt);
temp3 = (temp1 * -1 * vt / wt) + (vt * cos(th_prime) * deltaT / wt);
temp4 = (temp2 * -1 * vt / wt) + (vt * sin(th_prime) * deltaT / wt);
V = [[temp1, temp3];...
     [temp2, temp4];...
     [0, deltaT]];
   
M = [[(alpha(1) * vt^2) + (alpha(2) * wt^2), 0];...
     [0, (alpha(3) * vt^2) + (alpha(4) * wt^2)]];
   
mu_t_bar = mu_t_1 + [(-vt * sin(th) / wt) + (vt * sin(th_prime) / wt);...
                     (vt * cos(th) / wt) - (vt * cos(th_prime) / wt);...
                     (wt * deltaT)];
        
sigma_t_bar = (G * sigma_t_1 * G') + (V * M * V');

mu_t_motion = mu_t_bar;
sigma_t_motion = sigma_t_bar;
 
%% Observation Step 
Q = [[sigma_r^2, 0];...
     [0, sigma_phi^2]];

% For all observed features (only one for our scenario) do:
j = meas_C;
diffX = m(j, 1) - mu_t_bar(1);
diffY = m(j, 2) - mu_t_bar(2);
q = (diffX)^2 + (diffY)^2;
z_hat = [(sqrt(q));...
         (atan2(diffY, diffX) - mu_t_bar(3));...
         (j)];
H = [[-diffX/sqrt(q), -diffY/sqrt(q), 0];...
     [diffY/q, -diffX/q, -1];...
     [0, 0, 0]];

H = H(1:2, :); %Remove last row because Q is 2x2
S = (H * sigma_t_bar * H') + Q;
K = sigma_t_bar * H' / S; %Division instead of inverse because matlab complains

% Deal with angle wraparound
zDiff = zt(1:2) - z_hat(1:2);
zDiff(2) = mod(zDiff(2), 2 * pi);

if zDiff(2) < -pi
    zDiff(2) = zDiff(2) + (2 * pi);
elseif zDiff(2) >= pi
    zDiff(2) = zDiff(2) - (2 * pi);
end

mu_t_bar = mu_t_bar + (K * zDiff);

sigma_t_bar = (eye(3,3) - (K * H)) * sigma_t_bar;

%% Output
mu_t    = mu_t_bar;
sigma_t = sigma_t_bar;
     
     
end
