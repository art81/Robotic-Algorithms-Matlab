close all; clear; clc;
addpath('./Functions');
deltaT = 1; %second

%% **********PROBLEM 3 (Beam-Based Measurement Model)**********
maxRange = 5;
x0 = [2, 0, pi/2]';
u1 = [0.5, 0];
u2 = [0.2, 0.76];

x1_noiseless = sample_motion_model_velocity(u1,x0, zeros(6, 1), deltaT);
x2_noiseless = sample_motion_model_velocity(u2,x1_noiseless, zeros(6, 1), deltaT);

[p1, z_des1] = beam_range_finder_model(x1_noiseless, maxRange);
[p2, z_des2] = beam_range_finder_model(x2_noiseless, maxRange);

%plotting the PDF's with line for z_des
figure();
fplot(p1, [0, 1.2*maxRange]); hold on;
plot([z_des1, z_des1], [0, 5]);
legend('PDF', strcat('z_1^* = ', num2str(z_des1)));
title('PDF For P(z1 | x1, m)');

figure();
fplot(p2, [0, 1.2*maxRange]); hold on;
plot([z_des2, z_des2], [0, 5]);
legend('PDF', strcat('z_2^* = ', num2str(z_des2)));
title('PDF For P(z2 | x2, m)');

%% **********PROBLEM 4 (Landmark-Based Measurement Model)**********
x0 = [2, 0, pi/2]';
z0 = [[1.6213, 2.430, 1]; [1.1270, 1.4140, 2]];
z1 = [[0.5100, -2.6801, 2]; [1.0270, 1.405, 3]];
u1 = [1.2, -1.0];

% Landmark Map Definition
m1 = [1, 1];
m2 = [2, 1];
m3 = [3, 2];
m4 = [2, 4];
mL = [m1; m2; m3; m4];

p1 = landmark_detection_model(z0(1, :), x0, mL, 0.3, 0.5) * landmark_detection_model(z0(2, :), x0, mL, 0.3, 0.5);
x1_noiseless = sample_motion_model_velocity(u1,x0, zeros(6, 1), deltaT);
p2 = landmark_detection_model(z1(1, :), x1_noiseless, mL, 0.3, 0.5) * landmark_detection_model(z1(2, :), x1_noiseless, mL, 0.3, 0.5);

disp('');
p1
p2