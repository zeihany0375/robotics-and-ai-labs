function [first, strictFirst, Q, fractionCorrect] = toyQLearningLab(epsilon, gamma, p)
% The report's learned table averages the final ten thousand episodes.
[first,strictFirst,Q,fractionCorrect] = trainToyQ(epsilon,gamma,p,50000,40001);
end
