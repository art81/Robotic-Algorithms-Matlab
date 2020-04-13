close all; clear; clc;
addpath('./Functions');

%% P1
disp("**********PROBLEM 1**********" + newline);
N = [100 1000 100000];
struct1 = struct();
pdf1 = gaussian_pdf(0, 1);

figure('Name', 'Problem 1');
for i = 1:numel(N)
    fieldName = strcat('Distribution_',num2str(i));
    currDist = normrnd(0, 1,[1,N(i)]);
    struct1.(fieldName) = currDist;
    
    subplot(numel(N),1,i);
    struct1.(strcat(fieldName,'_histogram')) = histogram(currDist,'Normalization','pdf');
    hold on;
    fplot(pdf1)
    title(strcat('N = ', {' '},num2str(N(i))));
end

%% P2
disp("**********PROBLEM 2**********" + newline);
struct2 = struct();
pdf2 = gaussian_pdf(0, 1.5);

figure('Name', 'Problem 2');
for i = 1:numel(N)
    fieldName = strcat('Distribution_',num2str(i));
    currDist = normrnd(0, 1.5,[1,N(i)]);
    struct2.(fieldName) = currDist;
    
    subplot(numel(N),1,i);
    struct2.(strcat(fieldName,'_histogram')) = histogram(currDist, 'Normalization','pdf');
    hold on;
    fplot(pdf2)
    title(strcat('N = ', {' '},num2str(N(i))));
end

%% P3
disp("**********PROBLEM 3**********" + newline);
xDist = rejection_sample_p3(100000);
syms x;
assume(-2<=x<=2)
pdf3 = piecewise(-1<=x<=1, abs(x));

figure('Name', 'Problem 3');
h3 = histogram(xDist, 'Normalization','pdf');
hold on
fplot(pdf3, [-2, 2])
title('N = 100,000');

%% P4
disp("**********PROBLEM 4**********" + newline);
n = 100000;
fx = zeros(1,n);

for i = 1:n
    rand = uniform_random(0, 1);
    
    if rand <= 0.3
        fx(i) = normrnd(0, 1);
    else
        fx(i) = normrnd(2, 0.5);
    end
end

pdf4 = (0.3 * gaussian_pdf(0, 1)) + (0.7 * gaussian_pdf(2, 0.5));

figure('Name', 'Problem 4');
h4 = histogram(fx, 'Normalization','pdf');
hold on
fplot(pdf4, [-5, 5])


%% P5
disp("**********PROBLEM 5**********" + newline);
n = 100000;
x5 = normrnd(1, 0.5, [1,n]);
y5 = x5 .^ 2;
x5_2 = normrnd(1, 0.5, [1,1000]);
y5_2 = x5_2 .^ 2;

% figure('Name', 'Problem 5 (N = 100,000)');
% subplot(2, 2, 1); hold on;
% histogram(x5, 'Normalization','pdf');
% xlabel('X-Histogram');
% subplot(2, 2, 2);
% histogram(y5, 'Normalization','pdf');
% xlabel('Y-Histogram');
% subplot(2, 2, 3); hold on;
% makeDensityPlot(x5);
% xlabel('X-Density Distribution');
% subplot(2, 2, 4); hold on;
% makeDensityPlot(y5);
% xlabel('Y-Density Distribution');
% 
% figure('Name', 'Problem 5 (N = 1,000)');
% subplot(2, 2, 1); hold on;
% histogram(x5_2, 'Normalization','pdf');
% xlabel('X-Histogram');
% subplot(2, 2, 2);
% histogram(y5_2, 'Normalization','pdf');
% xlabel('Y-Histogram');
% subplot(2, 2, 3); hold on;
% makeDensityPlot(x5_2);
% xlabel('X-Density Distribution');
% subplot(2, 2, 4); hold on;
% makeDensityPlot(y5_2);
% xlabel('Y-Density Distribution');
