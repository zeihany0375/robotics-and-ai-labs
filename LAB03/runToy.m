function runToy
if ~isfolder('evidence')
    mkdir('evidence');
end
a = 7;
b = 6;
s = 76;
p = .05 + .02*b;
results = [];
for slip = [p, 0]
    for e = [0, .1, .3]
        for g = [.5, .9]
            for seed = s:s+19
                rng(seed, 'twister');
                [ep, strictEp] = trainToyQ(e,g,slip,5000,[]);
                results(end+1,:) = [slip, e, g, seed, ep, strictEp]; %#ok<AGROW>
            end
        end
    end
end
sweep = array2table(results, 'VariableNames', ...
    {'Slip','Epsilon','Gamma','Seed','FirstEpisode','FirstStrictEpisode'});
writetable(sweep, 'evidence/toy_sweep.csv');

rows = [];
stability = [];
for slip = [0, p]
    for g = [.5, .9]
        rng(s, 'twister');
        [~, ~, Q, fraction] = toyQLearningLab(.3, g, slip);
        [~, policy] = max(Q, [], 2);
        policy(6) = 0;
        rows = [rows; repmat([slip,g],11,1), (-5:5)', Q, ...
            abs(diff(Q,1,2)), policy]; %#ok<AGROW>
        stability(end+1,:) = [slip,g,fraction]; %#ok<AGROW>
    end
end
writetable(array2table(rows, 'VariableNames', ...
    {'Slip','Gamma','Position','QLeft','QRight','Margin','Policy'}), ...
    'evidence/toy_values.csv');
writetable(array2table(stability, 'VariableNames', {'Slip','Gamma','CorrectFraction'}), ...
    'evidence/toy_stability.csv');
fprintf('Task 1 complete, ID digits %d%d, seed %04d\n', a, b, s);
end
