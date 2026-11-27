function runRobot
if ~isfolder('evidence')
    mkdir('evidence');
end
set(groot,'defaultFigureVisible','off');
load robotmodel agent env
load_system('whrobot');
model = 'whrobot';
set_param([model '/visualization'],'Commented','on');
agent.UseExplorationPolicy = false;
env.UseFastRestart = 'on';
params = robotParameters(model);
seed = 76;
actor = getActor(agent);
fprintf('Robotics experiments using MATLAB %s\n',version);

if ~isfile('evidence/grid.mat')
    [X,T] = ndgrid([-4.5:.5:-2.5, 2.5:.5:4.5], -180:15:165);
    starts = [X(:), 3.12*ones(numel(X),1), deg2rad(T(:)), zeros(numel(X),2)];
    assert(~any(startClearance(starts,params)), 'Initial collision in required grid.');
    [gridRuns,gridReplay] = replayStarts(agent,env,model,starts,params,params.m,false,0);
    writetable(gridRuns,'evidence/grid.csv');
    save('evidence/grid.mat','gridRuns','gridReplay','params');
end
fprintf('The 240-start grid is complete.\n');

if ~isfile('evidence/baseline.csv')
    [baseline,~] = replayStarts(agent,env,model,randomStarts(50,7,false),params,params.m,false,0);
    writetable(baseline,'evidence/baseline.csv');
end
if ~isfile('evidence/mirror_actor_grid.csv')
    measureMirrorActor(actor);
end
if ~isfile('evidence/mirror.mat')
    left = randomStarts(24,seed,true);
    right = left;
    right(:,1) = -left(:,1);
    right(:,3) = atan2(sin(left(:,3)),-cos(left(:,3)));
    right(:,5) = -left(:,5);
    [leftRuns,leftReplay] = replayStarts(agent,env,model,left,params,params.m,false,0);
    [rightRuns,rightReplay] = replayStarts(agent,env,model,right,params,params.m,false,0);
    mirrorPairs = table(leftRuns.Run,leftRuns.Outcome,rightRuns.Outcome, ...
        leftRuns.TimeGoal,rightRuns.TimeGoal,abs(leftRuns.TimeGoal-rightRuns.TimeGoal), ...
        'VariableNames',{'Pair','LeftOutcome','RightOutcome','LeftTime','RightTime','AbsDelta'});
    writetable(mirrorPairs,'evidence/mirror_pairs.csv');
    save('evidence/mirror.mat','leftRuns','rightRuns','leftReplay','rightReplay','mirrorPairs');
end
fprintf('Random baseline and symmetry experiments are complete.\n');

robStarts = randomStarts(30,seed,false);
massFactors = [.5,1,2,3,4];
limits = [.7,.5,.3];
cases = [massFactors,limits];
kind = [repmat("Mass",1,numel(massFactors)),repmat("Limit",1,numel(limits))];
allResults = table();
allReplay = cell(1,numel(cases));
for k = 1:numel(cases)
    factor = cases(k);
    mass = params.m;
    cap = 1;
    if kind(k) == "Mass"
        mass = mass*factor;
    else
        cap = factor;
    end
    label = sprintf('%s_%03d',kind(k),round(100*factor));
    checkpoint = fullfile('evidence',['case_' label '.mat']);
    if isfile(checkpoint)
        checkpointState = load(checkpoint,'results','replay');
        results = checkpointState.results;
        replay = checkpointState.replay;
    else
        set_param([model '/Saturation'],'UpperLimit',num2str(cap),'LowerLimit',num2str(-cap));
        [results,replay] = replayStarts(agent,env,model,robStarts,params,mass,false,0);
        results.Kind = repmat(kind(k),height(results),1);
        results.Setting = repmat(factor,height(results),1);
        save(checkpoint,'results','replay');
    end
    allResults = [allResults;results]; %#ok<AGROW>
    allReplay{k} = replay;
    fprintf('%s %.2f complete: %d/%d successful.\n',kind(k),factor, ...
        sum(results.Outcome=="Success"),height(results));
end
writetable(allResults,'evidence/robustness.csv');
save('evidence/robustness.mat','allResults','allReplay','cases','kind');
close_system(model,0);
end
