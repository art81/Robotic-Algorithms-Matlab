close all; clear; clc;
addpath('./Functions');
deltaT = 1; %second

%% Parameters for discretized state space
xMin = 0;
xMax = 4;
yMin = 0;
yMax = 3;
thMin = 0;
thMax = 2*pi;

xSamples  = 200;
ySamples  = 200;
thSamples = 90;

x_vec = linspace(xMin,xMax,xSamples);
y_vec = linspace(yMin,yMax,ySamples);
th_vec = linspace(thMin,thMax,thSamples);

%% **********PROBLEM 1 (Velocity Model)**********
%1a: For this one since we want no error, we make alpha all zeros
x0 = [2, 0, pi/2]';
u1 = [pi/2, pi/2]';
u2 = [pi/2, -pi/2]';

x1_noiseless = sample_motion_model_velocity(u1,x0, zeros(6, 1), deltaT);
x2_noiseless = sample_motion_model_velocity(u2,x1_noiseless, zeros(6, 1), deltaT);

numSamplesInTraj = 200;
x1_noiseless_vec = zeros(numSamplesInTraj, 3);
x2_noiseless_vec = zeros(numSamplesInTraj, 3);
x1_noiseless_vec(1, :) = x0;
x2_noiseless_vec(1, :) = x1_noiseless;

for i = 2:numSamplesInTraj
    x1_noiseless_vec(i, :) = sample_motion_model_velocity(u1,x1_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
    x2_noiseless_vec(i, :) = sample_motion_model_velocity(u2,x2_noiseless_vec(i - 1, :), zeros(6, 1), deltaT / numSamplesInTraj);
end

%1b: This one is similar but we have actual values for alpha
numSamples = 1000;
alpha1 = [0.0001, 0.0001, 0.01, 0.0001, 0.0001, 0.0001]; %More rotational error
alpha2 = [0.005, 0.005, 0.0001, 0.0001, 0.0001, 0.0001]; %More translational error
x1_vec_1 = zeros(numSamples, 3);
x2_vec_1 = zeros(numSamples, 3);
x1_vec_2 = zeros(numSamples, 3);
x2_vec_2 = zeros(numSamples, 3);

for i = 1:numSamples
    x1_vec_1(i, :) = sample_motion_model_velocity(u1,x0, alpha1, deltaT);
    x2_vec_1(i, :) = sample_motion_model_velocity(u2,x1_vec_1(i, :), alpha1, deltaT);
    x1_vec_2(i, :) = sample_motion_model_velocity(u1,x0, alpha2, deltaT);
    x2_vec_2(i, :) = sample_motion_model_velocity(u2,x1_vec_2(i, :), alpha2, deltaT);
end

%Visualization
figure();
scatter(x0(1), x0(2), 'filled'); hold on;
scatter(x1_vec_1(:, 1), x1_vec_1(:, 2));
scatter(x1_vec_2(:, 1), x1_vec_2(:, 2));
scatter(x1_noiseless_vec(:, 1), x1_noiseless_vec(:, 2),5, 'filled'); %Noiseless sample!
scatter(x2_vec_1(:, 1), x2_vec_1(:, 2));
scatter(x2_vec_2(:, 1), x2_vec_2(:, 2));
scatter(x2_noiseless_vec(:, 1), x2_noiseless_vec(:, 2),5, 'filled'); %Noiseless sample!
scatter(x1_noiseless(1), x1_noiseless(2), 'filled');
scatter(x2_noiseless(1), x2_noiseless(2), 'filled');
legend('Starting Location', 'X1 with Noise (alpha1 - angular error)', 'X1 with Noise (alpha2 - translational error)', 'X1 Noiseless Traj', 'X2 with Noise (alpha1 - angular error)', 'X2 with Noise (alpha2 - translational error)', 'X2 Noiseless Traj'); hold off;
title('Velocity Model with two different "alphas"');

%1c: We must use the motion_model_velocity function to generate a prob
%distribution across the state space

pdfVelocity = zeros(xSamples*ySamples, 3);
pdfVelocity2 = zeros(xSamples*ySamples, 3);
c = 1;
for x = x_vec
    for y = y_vec
        p = 0;
        p2 = 0;
        for th = th_vec
            currX = [x, y, th]';
            p = p + motion_model_velocity(currX, u1, x0, alpha1, deltaT);
            p2 = p2 + motion_model_velocity(currX, u1, x0, alpha2, deltaT);
        end
        pdfVelocity(c, :) = [x, y, p];
        pdfVelocity2(c, :) = [x, y, p2];
        c = c + 1;
    end
end

% Plot results using scatter3 and an inverted gray colormap
figure;
dotsize=10;
scatter3(pdfVelocity(:, 1), pdfVelocity(:, 2), pdfVelocity(:, 3), dotsize, pdfVelocity(:, 3), 'filled'); hold on;
scatter3(x1_noiseless(1), x1_noiseless(2), max(pdfVelocity(:, 3)) + 1, 'filled');
legend('Noiseless Value', 'PDF');
title('PDF for Velocity Model with alpha1 (higher angular error)');
colormap(flipud(gray));
view(2);

figure;
scatter(x0(1), x0(2), 'filled'); hold on;
scatter3(pdfVelocity2(:, 1), pdfVelocity2(:, 2), pdfVelocity2(:, 3), dotsize, pdfVelocity2(:, 3), 'filled'); hold on;
scatter3(x1_noiseless(1), x1_noiseless(2), max(pdfVelocity2(:, 3)) + 1, 'filled');
legend('Noiseless Value', 'PDF');
title('PDF for Velocity Model with alpha2 (higher translational error)');
colormap(flipud(gray));
view(2);

%% **********PROBLEM 2 (Odometry Model)**********
%2a: For this one since we want no error, we make alpha all zeros
x0 = [2, 0, pi/2]';
x0_bar = [1, 0, 0]';
x1_bar = [3, -1, -1.571]';
x2_bar = [3, -2, 0]';
u1 = [x0_bar, x1_bar]';
u2 = [x1_bar, x2_bar]';

x1_noiseless_od = sample_motion_model_odometry(u1,x0, zeros(4, 1));
x2_noiseless_od = sample_motion_model_odometry(u2,x1_noiseless_od, zeros(4, 1));

%2b: This one is similar but we have actual values for alpha
alpha1_od = [0.01, 0.002, 0.0001, 0.0001]; %More rotational error
alpha2_od = [0.0001, 0.0002, 0.01, 0.0001]; %More translational error
x1_vec_1_od = zeros(numSamples, 3);
x2_vec_1_od = zeros(numSamples, 3);
x1_vec_2_od = zeros(numSamples, 3);
x2_vec_2_od = zeros(numSamples, 3);

for i = 1:numSamples
    x1_vec_1_od(i, :) = sample_motion_model_odometry(u1,x0, alpha1_od);
    x2_vec_1_od(i, :) = sample_motion_model_odometry(u2,x1_vec_1_od(i, :), alpha1_od);
    x1_vec_2_od(i, :) = sample_motion_model_odometry(u1,x0, alpha2_od);
    x2_vec_2_od(i, :) = sample_motion_model_odometry(u2,x1_vec_2_od(i, :), alpha2_od);
end

%Visualization
figure();
scatter(x0(1), x0(2), 'filled'); hold on;
scatter(x1_vec_1_od(:, 1), x1_vec_1_od(:, 2));
scatter(x1_vec_2_od(:, 1), x1_vec_2_od(:, 2));
plot([x0(1), x1_noiseless_od(1)], [x0(2), x1_noiseless_od(2)]); %Noiseless sample!
scatter(x2_vec_1_od(:, 1), x2_vec_1_od(:, 2));
scatter(x2_vec_2_od(:, 1), x2_vec_2_od(:, 2));
plot([x1_noiseless_od(1), x2_noiseless_od(1)], [x1_noiseless_od(2), x2_noiseless_od(2)]); %Noiseless sample!
scatter(x1_noiseless_od(1), x1_noiseless_od(2), 'filled');
scatter(x2_noiseless_od(1), x2_noiseless_od(2), 'filled');
legend('Starting Location', 'X1 with Noise (alpha1 - angular error)', 'X1 with Noise (alpha2 - translational error)', 'X1 Noiseless Traj', 'X2 with Noise (alpha1 - angular error)', 'X2 with Noise (alpha2 - translational error)', 'X2 Noiseless Traj'); hold off;
title('Odometry Model with two different "alphas"');

%2c: We must use the motion_model_odometry function to generate a prob
%distribution across the state space
pdfOdometry = zeros(xSamples*ySamples, 3);
pdfOdometry2 = zeros(xSamples*ySamples, 3);
c = 1;
for x = x_vec
    
    for y = y_vec
        p = 0;
        p2 = 0;
        for th = th_vec
            currX = [x, y, th]';
            p = p + motion_model_odometry(currX, u1, x0, alpha1_od);
            p2 = p2 + motion_model_odometry(currX, u1, x0, alpha2_od);
        end
        pdfOdometry(c, :) = [x, y, p];
        pdfOdometry2(c, :) = [x, y, p2];
        c = c + 1;
    end
end

% Plot results using scatter3 and an inverted gray colormap
figure;
dotsize=10;
scatter3(pdfOdometry(:, 1), pdfOdometry(:, 2), pdfOdometry(:, 3), dotsize, pdfOdometry(:, 3)); hold on;
scatter3(x1_noiseless_od(1), x1_noiseless_od(2), max(pdfOdometry(:, 3)) + 1, 'filled');
title('PDF for Odometry Model with alpha1 (higher angular error)');
colormap(flipud(gray));
view(2);

figure;
scatter(x0(1), x0(2), 'filled'); hold on;
scatter3(pdfOdometry2(:, 1), pdfOdometry2(:, 2), pdfOdometry2(:, 3), dotsize, pdfOdometry2(:, 3), 'filled');
scatter3(x1_noiseless_od(1), x1_noiseless_od(2), max(pdfOdometry2(:, 3)) + 1, 'filled');
title('PDF for Odometry Model with alpha2 (higher translational error)');
colormap(flipud(gray));
view(2);