function runChecks
load robotmodel agent env
load_system('whrobot');
set_param('whrobot/visualization','Commented','on');
agent.UseExplorationPolicy = false;
params = robotParameters('whrobot');
start = [3.5,1.5,0,0,0];
assert(startClearance(start,params),'The deliberate collision control is clear.');
tag = "";
try
    replayStarts(agent,env,'whrobot',start,params,params.m,false,0);
catch err
    tag = string(err.identifier);
end
inputs = Simulink.SimulationInput('whrobot');
keys = {'x0','y0','theta0','v0','w0'};
for k = 1:numel(keys)
    inputs = setVariable(inputs,keys{k},start(k),'Workspace','whrobot');
end
inputs = setVariable(inputs,'agent',agent);
inputs = setModelParameter(inputs,'SignalLogging','on','SignalLoggingName','logsout');
cornerPorts = get_param('whrobot/robot corners','PortHandles');
rewardPorts = get_param('whrobot/reward function','PortHandles');
set_param(cornerPorts.Outport(3),'DataLogging','on','DataLoggingNameMode','Custom', ...
    'DataLoggingName','collisionTruth');
set_param(rewardPorts.Outport(2),'DataLogging','on','DataLoggingNameMode','Custom', ...
    'DataLoggingName','doneTruth');
raw = sim(inputs);
collisionSignal = raw.logsout.getElement('collisionTruth').Values;
doneSignal = raw.logsout.getElement('doneTruth').Values;
assert(collisionSignal.Data(1) && doneSignal.Data(1));
writetable(table(start(1),start(2),collisionSignal.Time(1), ...
    doneSignal.Time(1),tag,'VariableNames', ...
    {'X','Y','CollisionTime','DoneTime','WrapperError'}),'evidence/collision_probe.csv');
close_system('whrobot',0);

load('evidence/noise.mat','allTraces','sigma');
maxDelta = 0;
maxOtherChannel = 0;
for k = 1:30
    small = allTraces{2}(k);
    middle = allTraces{3}(k);
    a = normalizedNoise(small,sigma(2));
    b = normalizedNoise(middle,sigma(3));
    n = min(numel(a),numel(b))-1;
    maxDelta = max(maxDelta,max(abs(a(1:n)-b(1:n))));
    maxOtherChannel = max(maxOtherChannel, ...
        max(abs(small.obs(3:6,1)-middle.obs(3:6,1))));
end
assert(maxDelta < 1e-8,'Noise runs did not replay the same y disturbance samples.');
assert(maxOtherChannel < 1e-12,'The other four initial inputs differ across noise levels.');
writetable(table(maxDelta,maxOtherChannel),'evidence/noise_checks.csv');
end

function noise = normalizedNoise(trace,sigma)
[~,a,b] = intersect(round(trace.t,9),round(trace.trueTime,9),'stable');
noise = (trace.obs(2,a)'-trace.trueY(b))/sigma;
end
