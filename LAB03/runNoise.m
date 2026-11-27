function runNoise
if ~isfolder('evidence')
    mkdir('evidence');
end
model = 'whrobot_noise';
set(groot,'defaultFigureVisible','off');
load robotmodel agent
agent.UseExplorationPolicy = false;
buildNoiseModel(model);
set_param([model '/visualization'],'Commented','on');
params = robotParameters(model);
env = rlSimulinkEnv(model,[model '/controller'], ...
    getObservationInfo(agent),getActionInfo(agent));
env.UseFastRestart = 'on';
starts = randomStarts(30,76,false);
sigma = [0,.1,.3,.6,1];
allNoise = table();
allTraces = cell(size(sigma));
for k = 1:numel(sigma)
    checkpoint = fullfile('evidence',sprintf('noise_%03d.mat',round(100*sigma(k))));
    if isfile(checkpoint)
        saved = load(checkpoint,'results','traces');
        results = saved.results;
        traces = saved.traces;
    else
        workspace = get_param(model,'ModelWorkspace');
        assignin(workspace,'noiseSigma',sigma(k));
        [results,traces] = replayStarts(agent,env,model,starts,params,params.m,true,76000);
        results.Sigma = repmat(sigma(k),height(results),1);
        save(checkpoint,'results','traces');
    end
    allNoise = [allNoise;results]; %#ok<AGROW>
    allTraces{k} = traces;
    fprintf('Noise sigma %.2f complete: %d/%d successful.\n',sigma(k), ...
        sum(results.Outcome=="Success"),height(results));
end
writetable(allNoise,'evidence/noise.csv');
save('evidence/noise.mat','allNoise','allTraces','sigma');
try
    print(['-s' model],'-dpng','-r165','evidence/model_noise.png');
catch issue
    fprintf('Model snapshot failed: %s\n',issue.message);
end
close_system(model,0);
end

function buildNoiseModel(model)
copyfile('whrobot.slx',[model '.slx'],'f');
load_system(model);
ws = get_param(model,'ModelWorkspace');
assignin(ws,'noiseSigma',0);
assignin(ws,'noiseSeed',76002);
set_param(model,'SignalLogging','on','SignalLoggingName','logsout');
ports = get_param([model '/Mux1'],'PortHandles');
set_param(ports.Outport(1),'DataLogging','on','DataLoggingNameMode','Custom', ...
    'DataLoggingName','trueState');

delete_line(model,'Mux1/1','controller/1');
add_block('simulink/Sources/Random Number',[model '/Gaussian noise'], ...
    'Mean','[0 0 0 0 0 0]','Variance','[1 1 1 1 1 1]', ...
    'Seed','noiseSeed+(0:5)','SampleTime','dt', ...
    'Position',[-285,55,-223,95]);
add_block('simulink/Math Operations/Gain',[model '/Position channels only'], ...
    'Gain','noiseSigma*[1 1 0 0 0 0]','Multiplication','Element-wise(K.*u)', ...
    'Position',[-178,52,-108,98]);
add_block('simulink/Math Operations/Sum',[model '/Noisy observation'], ...
    'Inputs','++','IconShape','rectangular','Orientation','left', ...
    'Position',[-311,161,-280,196]);
add_line(model,'Gaussian noise/1','Position channels only/1','autorouting','on');
add_line(model,'Position channels only/1','Noisy observation/1','autorouting','on');
add_line(model,'Mux1/1','Noisy observation/2','autorouting','on');
add_line(model,'Noisy observation/1','controller/1','autorouting','on');
assert(sourceBlock([model '/controller'],1) == string([model '/Noisy observation']));
assert(sourceBlock([model '/reward function'],1) == string([model '/Mux1']));
assert(sourceBlock([model '/robot corners'],1) == string([model '/Mux1']));
save_system(model);
end

function name = sourceBlock(destination, index)
ports = get_param(destination,'PortHandles');
line = get_param(ports.Inport(index),'Line');
name = string(getfullname(get_param(line,'SrcBlockHandle')));
end
