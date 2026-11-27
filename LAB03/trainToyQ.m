function [first, strictFirst, Q, fractionCorrect] = trainToyQ(epsilon,gamma,p,budget,averageFrom)
% Preserve the starter's action order, reward, start rule, and learning rate.
Q = zeros(11, 2);
alpha = 0.5;
maxSteps = 20;
first = NaN;
strictFirst = NaN;
sweepOnly = isempty(averageFrom);
positions = [-5:-1 1:5]';
rows = positions + 6;
expected = 1 + (positions < 0);
Qsum = zeros(11, 2);
correctCount = 0;
for ep = 1:budget
    s = randi([-5, 5]);
    if s == 0
        s = 5;
    end
    for step = 1:maxSteps
        row = s + 6;
        if rand < epsilon
            action = randi(2);
        else
            [~, action] = max(Q(row, :));
        end
        move = 2 * action - 3;
        if rand < p
            move = -move;
        end
        next = max(-5, min(5, s + move));
        done = next == 0;
        reward = -1 + 11 * done;
        target = reward + ~done * gamma * max(Q(next + 6, :));
        Q(row, action) = Q(row, action) + alpha * (target - Q(row, action));
        s = next;
        if done
            break
        end
    end
    [~, greedy] = max(Q(rows, :), [], 2);
    correct = all(greedy == expected);
    if correct && isnan(first)
        first = ep;
    end
    if correct && all(Q(rows, 1) ~= Q(rows, 2)) && isnan(strictFirst)
        strictFirst = ep;
    end
    if ~sweepOnly && ep >= averageFrom
        Qsum = Qsum + Q;
        correctCount = correctCount + correct;
    end
    if sweepOnly && ~isnan(strictFirst)
        break
    end
end
fractionCorrect = NaN;
if ~sweepOnly
    Q = Qsum / (budget - averageFrom + 1);
    fractionCorrect = correctCount / (budget - averageFrom + 1);
end
end
