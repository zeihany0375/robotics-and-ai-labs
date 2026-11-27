function [summary, replay] = replayStarts(agent, env, model, starts, params, mass, noisy, noiseSeed)
% A serial reset closure assigns each row to its simulation exactly once.
next = 0;
env.ResetFcn = @reset;
opt = rlSimulationOptions('MaxSteps',120,'NumSimulations',size(starts,1), ...
    'UseParallel',false,'StopOnError','on');
output = sim(agent, env, opt);
assert(numel(output) == size(starts,1), 'Unexpected number of episodes.');
[collision, clearance] = startClearance(starts, params);
number = numel(output);
outcome = strings(number,1);
elapsed = zeros(number,1);
timeGoal = NaN(number,1);
terminalY = zeros(number,1);
steps = zeros(number,1);
episodeReturn = zeros(number,1);
empty = struct('t',[],'obs',[],'tAction',[],'action',[], ...
    'trueY',[],'trueTime',[],'trueX',[]);
replay = repmat(empty,number,1);
for k = 1:number
    ep = output(k);
    obs = ep.Observation.obs1;
    state = reshape(obs.Data,6,[]);
    actions = ep.Action.act1;
    t = obs.Time(:);
    row = starts(k,:);
    expected = [row(1:2), sin(row(3)),cos(row(3)),row(4:5)]';
    if noisy
        storage = ep.SimulationInfo;
        if storage.NumSimulations == 1
            simData = storage(1);
        else
            simData = storage(k);
        end
        truth = simData.logsout.getElement('trueState').Values;
        trueTime = truth.Time(:);
        trueState = reshape(permuteToChannels(truth.Data, numel(trueTime)),6,[]);
        state0 = trueState(:,1);
    else
        trueTime = t;
        trueState = state;
        state0 = state(:,1);
    end
    assert(max(abs(state0-expected)) < 1e-5, ...
        'Start row %d does not match the simulated initial state.',k);
    elapsed(k) = t(end);
    terminalY(k) = trueState(2,end);
    steps(k) = numel(ep.Reward.Data);
    episodeReturn(k) = sum(ep.Reward.Data);
    if terminalY(k) < -1.5
        outcome(k) = "Success";
        timeGoal(k) = elapsed(k);
    elseif any(ep.IsDone.Data(end))
        outcome(k) = "Crash";
    else
        outcome(k) = "Timeout";
    end
    replay(k) = struct('t', t, 'obs', state, 'tAction', actions.Time(:), ...
        'action', reshape(actions.Data,2,[]), 'trueY',trueState(2,:)', ...
        'trueTime',trueTime,'trueX',trueState(1,:)');
end
summary = table((1:number)',starts(:,1),starts(:,2),rad2deg(starts(:,3)), ...
    starts(:,4),starts(:,5),collision,clearance,outcome,elapsed,timeGoal, ...
    terminalY,steps,episodeReturn,'VariableNames', ...
    {'Run','X','Y','HeadingDeg','V','Omega','InitialCollision','Clearance', ...
    'Outcome','Elapsed','TimeGoal','TerminalY','Steps','Return'});

    function input = reset(input)
        next = next + 1;
        assert(next <= size(starts,1), 'The simulation requested extra reset calls.');
        row = starts(next,:);
        theta = row(3);
        if abs(theta + pi) < 1e-12
            theta = pi;
        end
        values = [row(1:2), theta, row(4:5), mass];
        names = {'x0','y0','theta0','v0','w0','m'};
        for j = 1:numel(names)
            input = setVariable(input,names{j},values(j),'Workspace',model);
        end
        if noisy
            input = setVariable(input,'noiseSeed',noiseSeed + 2*next, ...
                'Workspace',model);
        end
    end
end

function array = permuteToChannels(array, times)
if ismatrix(array) && size(array,1) == times
    array = array.';
end
end
