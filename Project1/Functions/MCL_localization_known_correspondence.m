function [p, p_motion] = MCL_localization_known_correspondence(p_t_1, ut, zt, mL, sigma_r, sigma_phi, alpha)
% p-t_1     : Particles from previous time step
% ut        : Motion Command
% zt        : Measurement
% m         : Map
% Implementation of the Particle Filter (Monte-Carlo Localization) with Known Correspondence in
% a feature based map
M = numel(p_t_1(:, 1));
sizeState = numel(p_t_1(1, :));
deltaT = 1; %second

% Reference for this part is Table 8.2 in PR
% each particle has its own row.
p_bar = zeros(M, sizeState + 1); % increase size of columns to fit probability

for m = 1:M
    currX = p_t_1(m, :);
    xt = sample_motion_model_velocity(ut, currX, alpha, deltaT);
    
    wt = landmark_detection_model_MCL(zt', xt, mL, sigma_r, sigma_phi); %would need to put this in a loop for multiple zt
    p_bar(m, :) = [xt', wt];
end


weights = p_bar(:, sizeState + 1);
% This implementation works but probably better to use matlab function :(
% cumSumScore = cumsum(weights);
% 
% for m = 1:M
%     randNum = rand(); % since range (0, 1) --> (a + (b-a)*rand());
%     currVal = cumSumScore(1);
%     c = 1;
%     while (randNum > currVal)
%         c = c + 1;
%         currVal = cumSumScore(c);
%     end
%     % here c holds the index that passed the probability test
%     p(m, :) = p_bar(c, 1:sizeState);
% end

indexes = randsample(1:M, M, true, weights);

%% Output
p = p_bar(indexes, 1:3);
p_motion = p_bar(:, 1:3);
     
end