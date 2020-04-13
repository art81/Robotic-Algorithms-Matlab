close all; clear; clc;
addpath('./Functions');
addpath(genpath('../Homework2')); % for velocity motion model

xMin = 0;
xMax = 8;
yMin = 0;
yMax = 6;
deltaT = 1; %second
alpha = [0.0001; 0.0001; 0.01; 0.0001; 0.0001; 0.0001];

%% Velocity Model Information (Nominal Values and Trajectory Vectors)
x0 = [2, 2, 0]';
ut = [[1, 0]; [1, 0]; [1, 0]; [pi/2, pi/2];...
      [pi/2, pi/2]; [1, 0]; [1, 0]; [1, 0]]';
zt = [[2.276, 5.249, 2]; [4.321, 5.834, 3]; [3.418, 5.869, 3]; [3.774, 5.911, 4];...
      [2.631, 5.140, 5]; [4.770, 5.791, 6]; [3.828, 5.742, 6]; [3.153, 5.739, 6]]';
xt_noiseless = zeros(3, 9);

xt_noiseless(:, 1) = x0;
xt_noiseless(:, 2) = sample_motion_model_velocity(ut(:, 1),xt_noiseless(:, 1), zeros(6, 1), deltaT);
xt_noiseless(:, 3) = sample_motion_model_velocity(ut(:, 2),xt_noiseless(:, 2), zeros(6, 1), deltaT);
xt_noiseless(:, 4) = sample_motion_model_velocity(ut(:, 3),xt_noiseless(:, 3), zeros(6, 1), deltaT);
xt_noiseless(:, 5) = sample_motion_model_velocity(ut(:, 4),xt_noiseless(:, 4), zeros(6, 1), deltaT);
xt_noiseless(:, 6) = sample_motion_model_velocity(ut(:, 5),xt_noiseless(:, 5), zeros(6, 1), deltaT);
xt_noiseless(:, 7) = sample_motion_model_velocity(ut(:, 6),xt_noiseless(:, 6), zeros(6, 1), deltaT);
xt_noiseless(:, 8) = sample_motion_model_velocity(ut(:, 7),xt_noiseless(:, 7), zeros(6, 1), deltaT);
xt_noiseless(:, 9) = sample_motion_model_velocity(ut(:, 8),xt_noiseless(:, 8), zeros(6, 1), deltaT);

% Generate Noiseless Trajectories
numSamplesInTraj = 200;
x1_noiseless_vec = zeros(numSamplesInTraj, 3);
x2_noiseless_vec = zeros(numSamplesInTraj, 3);
x3_noiseless_vec = zeros(numSamplesInTraj, 3);
x4_noiseless_vec = zeros(numSamplesInTraj, 3);
x5_noiseless_vec = zeros(numSamplesInTraj, 3);
x6_noiseless_vec = zeros(numSamplesInTraj, 3);
x7_noiseless_vec = zeros(numSamplesInTraj, 3);
x8_noiseless_vec = zeros(numSamplesInTraj, 3);
x1_noiseless_vec(1, :) = xt_noiseless(:, 1);
x2_noiseless_vec(1, :) = xt_noiseless(:, 2);
x3_noiseless_vec(1, :) = xt_noiseless(:, 3);
x4_noiseless_vec(1, :) = xt_noiseless(:, 4);
x5_noiseless_vec(1, :) = xt_noiseless(:, 5);
x6_noiseless_vec(1, :) = xt_noiseless(:, 6);
x7_noiseless_vec(1, :) = xt_noiseless(:, 7);
x8_noiseless_vec(1, :) = xt_noiseless(:, 8);

for i = 2:numSamplesInTraj
    x1_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 1),x1_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x2_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 2),x2_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x3_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 3),x3_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x4_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 4),x4_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x5_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 5),x5_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x6_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 6),x6_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x7_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 7),x7_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x8_noiseless_vec(i, :) = sample_motion_model_velocity(ut(:, 8),x8_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
end

%% Landmark Map and Sigma Definition
m1 = [0, 0];
m2 = [4, 0];
m3 = [8, 0];
m4 = [8, 6];
m5 = [4, 6];
m6 = [0, 6];
mL = [m1; m2; m3; m4; m5; m6];

sigma_r = 0.1;
sigma_phi = 0.09;

%% Running EKF
mu_0 = xt_noiseless(:, 1);
sigma_0 = zeros(3, 3);
[mu_1, sigma_1, mu_1_motion, sigma_1_motion] = EKF_localization_known_correspondence(mu_0, sigma_0, ut(:, 1), zt(:, 1), mL, sigma_r, sigma_phi, alpha);
[mu_2, sigma_2, mu_2_motion, sigma_2_motion] = EKF_localization_known_correspondence(mu_1, sigma_1, ut(:, 2), zt(:, 2), mL, sigma_r, sigma_phi, alpha);
[mu_3, sigma_3, mu_3_motion, sigma_3_motion] = EKF_localization_known_correspondence(mu_2, sigma_2, ut(:, 3), zt(:, 3), mL, sigma_r, sigma_phi, alpha);
[mu_4, sigma_4, mu_4_motion, sigma_4_motion] = EKF_localization_known_correspondence(mu_3, sigma_3, ut(:, 4), zt(:, 4), mL, sigma_r, sigma_phi, alpha);
[mu_5, sigma_5, mu_5_motion, sigma_5_motion] = EKF_localization_known_correspondence(mu_4, sigma_4, ut(:, 5), zt(:, 5), mL, sigma_r, sigma_phi, alpha);
[mu_6, sigma_6, mu_6_motion, sigma_6_motion] = EKF_localization_known_correspondence(mu_5, sigma_5, ut(:, 6), zt(:, 6), mL, sigma_r, sigma_phi, alpha);
[mu_7, sigma_7, mu_7_motion, sigma_7_motion] = EKF_localization_known_correspondence(mu_6, sigma_6, ut(:, 7), zt(:, 7), mL, sigma_r, sigma_phi, alpha);
[mu_8, sigma_8, mu_8_motion, sigma_8_motion] = EKF_localization_known_correspondence(mu_7, sigma_7, ut(:, 8), zt(:, 8), mL, sigma_r, sigma_phi, alpha);

%% Running Monte-Carlo Localization (Particle Filter)
numParticles = 1000;

x0 = [2, 2, 0];
p0 = repmat(x0, numParticles, 1);

[p1, p1_motion] = MCL_localization_known_correspondence(p0, ut(:, 1), zt(:, 1), mL, sigma_r, sigma_phi, alpha);
[p2, p2_motion] = MCL_localization_known_correspondence(p1, ut(:, 2), zt(:, 2), mL, sigma_r, sigma_phi, alpha);
[p3, p3_motion] = MCL_localization_known_correspondence(p2, ut(:, 3), zt(:, 3), mL, sigma_r, sigma_phi, alpha);
[p4, p4_motion] = MCL_localization_known_correspondence(p3, ut(:, 4), zt(:, 4), mL, sigma_r, sigma_phi, alpha);
[p5, p5_motion] = MCL_localization_known_correspondence(p4, ut(:, 5), zt(:, 5), mL, sigma_r, sigma_phi, alpha);
[p6, p6_motion] = MCL_localization_known_correspondence(p5, ut(:, 6), zt(:, 6), mL, sigma_r, sigma_phi, alpha);
[p7, p7_motion] = MCL_localization_known_correspondence(p6, ut(:, 7), zt(:, 7), mL, sigma_r, sigma_phi, alpha);
[p8, p8_motion] = MCL_localization_known_correspondence(p7, ut(:, 8), zt(:, 8), mL, sigma_r, sigma_phi, alpha);

%% Final Visualizations (EKF)

% Plotting all Ellipses and Formatting
figure(); hold on;
lGray = [0.5, 0.5, 0.5];    % Color Definition
dGray = [0.25, 0.25, 0.25]; % Color Definition
lineWidth = 5;
multiplier = 1;

h1_m = plot_gaussian_ellipsoid(mu_1_motion(1:2), multiplier * sigma_1_motion(1:2, 1:2));
h2_m = plot_gaussian_ellipsoid(mu_2_motion(1:2), multiplier * sigma_2_motion(1:2, 1:2));
h3_m = plot_gaussian_ellipsoid(mu_3_motion(1:2), multiplier * sigma_3_motion(1:2, 1:2));
h4_m = plot_gaussian_ellipsoid(mu_4_motion(1:2), multiplier * sigma_4_motion(1:2, 1:2));
h5_m = plot_gaussian_ellipsoid(mu_5_motion(1:2), multiplier * sigma_5_motion(1:2, 1:2));
h6_m = plot_gaussian_ellipsoid(mu_6_motion(1:2), multiplier * sigma_6_motion(1:2, 1:2));
h7_m = plot_gaussian_ellipsoid(mu_7_motion(1:2), multiplier * sigma_7_motion(1:2, 1:2));
h8_m = plot_gaussian_ellipsoid(mu_8_motion(1:2), multiplier * sigma_8_motion(1:2, 1:2));

h1   = plot_gaussian_ellipsoid(mu_1(1:2), multiplier * sigma_1(1:2, 1:2));
h2   = plot_gaussian_ellipsoid(mu_2(1:2), multiplier * sigma_2(1:2, 1:2));
h3   = plot_gaussian_ellipsoid(mu_3(1:2), multiplier * sigma_3(1:2, 1:2));
h4   = plot_gaussian_ellipsoid(mu_4(1:2), multiplier * sigma_4(1:2, 1:2));
h5   = plot_gaussian_ellipsoid(mu_5(1:2), multiplier * sigma_5(1:2, 1:2));
h6   = plot_gaussian_ellipsoid(mu_6(1:2), multiplier * sigma_6(1:2, 1:2));
h7   = plot_gaussian_ellipsoid(mu_7(1:2), multiplier * sigma_7(1:2, 1:2));
h8   = plot_gaussian_ellipsoid(mu_8(1:2), multiplier * sigma_8(1:2, 1:2));

set(h1, 'color', dGray, 'LineWidth', lineWidth);
set(h1_m, 'color', lGray, 'LineWidth', lineWidth);
set(h2, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h2_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h3, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h3_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h4, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h4_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h5, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h5_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h6, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h6_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h7, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h7_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h8, 'color', dGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');
set(h8_m, 'color', lGray, 'LineWidth', lineWidth, 'HandleVisibility', 'off');

% Plotting Nominal Points and Trajectories
scatter(x1_noiseless_vec(:, 1), x1_noiseless_vec(:, 2),5, 'filled', 'b');
scatter(x2_noiseless_vec(:, 1), x2_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x3_noiseless_vec(:, 1), x3_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x4_noiseless_vec(:, 1), x4_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x5_noiseless_vec(:, 1), x5_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x6_noiseless_vec(:, 1), x6_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x7_noiseless_vec(:, 1), x7_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x8_noiseless_vec(:, 1), x8_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 1), xt_noiseless(2, 1), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 2), xt_noiseless(2, 2), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 3), xt_noiseless(2, 3), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 4), xt_noiseless(2, 4), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 5), xt_noiseless(2, 5), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 6), xt_noiseless(2, 6), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 7), xt_noiseless(2, 7), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 8), xt_noiseless(2, 8), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 9), xt_noiseless(2, 9), 'filled', 'b', 'HandleVisibility', 'off');

% Plotting Actual Trajectory
plot([mu_1(1), mu_2(1)], [mu_1(2), mu_2(2)], 'g');
plot([mu_2(1), mu_3(1)], [mu_2(2), mu_3(2)], 'g', 'HandleVisibility', 'off');
plot([mu_3(1), mu_4(1)], [mu_3(2), mu_4(2)], 'g', 'HandleVisibility', 'off');
plot([mu_4(1), mu_5(1)], [mu_4(2), mu_5(2)], 'g', 'HandleVisibility', 'off');
plot([mu_5(1), mu_6(1)], [mu_5(2), mu_6(2)], 'g', 'HandleVisibility', 'off');
plot([mu_6(1), mu_7(1)], [mu_6(2), mu_7(2)], 'g', 'HandleVisibility', 'off');
plot([mu_7(1), mu_8(1)], [mu_7(2), mu_8(2)], 'g', 'HandleVisibility', 'off');

legend({'Motion Step', 'MeasurementStep', 'Nominal Trajectory', 'Actual Trajectory'});
title('Extended Kalman Filter Localization');
axis([xMin xMax yMin yMax]);

%% Final Visualizations (PF)

%Visualization
scatterSize = 10;
figure(); hold on;
red = [1, 0, 0];

scatter(p1_motion(:, 1), p1_motion(:, 2), scatterSize, lGray, 'filled');
scatter(p2_motion(:, 1), p2_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');
scatter(p3_motion(:, 1), p3_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');
scatter(p4_motion(:, 1), p4_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');
scatter(p5_motion(:, 1), p5_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');
scatter(p6_motion(:, 1), p6_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');
scatter(p7_motion(:, 1), p7_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');
scatter(p8_motion(:, 1), p8_motion(:, 2), scatterSize, lGray, 'filled', 'HandleVisibility', 'off');

scatter(p0(:, 1), p0(:, 2), scatterSize, red, 'filled');
scatter(p1(:, 1), p1(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p2(:, 1), p2(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p3(:, 1), p3(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p4(:, 1), p4(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p5(:, 1), p5(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p6(:, 1), p6(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p7(:, 1), p7(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');
scatter(p8(:, 1), p8(:, 2), scatterSize, red, 'filled', 'HandleVisibility', 'off');

% Plotting Nominal Points and Trajectories
scatter(x1_noiseless_vec(:, 1), x1_noiseless_vec(:, 2),5, 'filled', 'b');
scatter(x2_noiseless_vec(:, 1), x2_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x3_noiseless_vec(:, 1), x3_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x4_noiseless_vec(:, 1), x4_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x5_noiseless_vec(:, 1), x5_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x6_noiseless_vec(:, 1), x6_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x7_noiseless_vec(:, 1), x7_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(x8_noiseless_vec(:, 1), x8_noiseless_vec(:, 2),5, 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 1), xt_noiseless(2, 1), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 2), xt_noiseless(2, 2), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 3), xt_noiseless(2, 3), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 4), xt_noiseless(2, 4), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 5), xt_noiseless(2, 5), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 6), xt_noiseless(2, 6), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 7), xt_noiseless(2, 7), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 8), xt_noiseless(2, 8), 'filled', 'b', 'HandleVisibility', 'off');
scatter(xt_noiseless(1, 9), xt_noiseless(2, 9), 'filled', 'b', 'HandleVisibility', 'off');

% Plotting Actual Trajectory
plot([mean(p0(:, 1)), mean(p1(:, 1))], [mean(p0(:, 2)), mean(p1(:, 2))], 'g');
plot([mean(p1(:, 1)), mean(p2(:, 1))], [mean(p1(:, 2)), mean(p2(:, 2))], 'g', 'HandleVisibility', 'off');
plot([mean(p2(:, 1)), mean(p3(:, 1))], [mean(p2(:, 2)), mean(p3(:, 2))], 'g', 'HandleVisibility', 'off');
plot([mean(p3(:, 1)), mean(p4(:, 1))], [mean(p3(:, 2)), mean(p4(:, 2))], 'g', 'HandleVisibility', 'off');
plot([mean(p4(:, 1)), mean(p5(:, 1))], [mean(p4(:, 2)), mean(p5(:, 2))], 'g', 'HandleVisibility', 'off');
plot([mean(p5(:, 1)), mean(p6(:, 1))], [mean(p5(:, 2)), mean(p6(:, 2))], 'g', 'HandleVisibility', 'off');
plot([mean(p6(:, 1)), mean(p7(:, 1))], [mean(p6(:, 2)), mean(p7(:, 2))], 'g', 'HandleVisibility', 'off');
plot([mean(p7(:, 1)), mean(p8(:, 1))], [mean(p7(:, 2)), mean(p8(:, 2))], 'g', 'HandleVisibility', 'off');

legend('Motion Step', 'Measurement (Resample) Step', 'Nominal Trajectory', 'Actual Trajectory (Based on Resampled Particle Means)');
title('Particle Filter (Monte-Carlo) Localization');
axis([xMin xMax yMin yMax]);


