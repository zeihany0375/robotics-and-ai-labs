function plotEvidence
set(groot,'defaultFigureVisible','off','defaultAxesFontName','Arial', ...
    'defaultAxesFontSize',11,'defaultLineLineWidth',1.5);
gridData = load('evidence/grid.mat');
mirror = load('evidence/mirror.mat');
rob = load('evidence/robustness.mat');
sensor = load('evidence/noise.mat');
load robotmodel agent
p = gridData.params;
data = gridData.gridRuns;
paths = gridData.gridReplay;
levels = ["Success","Crash","Timeout"];
x = unique(data.X);
headings = unique(data.HeadingDeg);
[~,outcomeCode] = ismember(string(data.Outcome),levels);

fig = newFigure(1000,400);
tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
ax = nexttile;
imageMap(ax,x,headings,reshape(outcomeCode,[],numel(headings))');
colormap(ax,[.10 .54 .44;.74 .18 .19;.65 .49 .16]);
clim(ax,[.5 3.5]);
bar = colorbar(ax,'Ticks',1:3,'TickLabels',cellstr(levels));
bar.TickLength = 0;
title(ax,'Outcome');
ax = nexttile;
times = reshape(data.TimeGoal,[],numel(headings))';
imageMap(ax,x,headings,times);
colormap(ax,parula);
bar = colorbar(ax);
bar.Label.String = 'Seconds';
title(ax,'Time to goal');
drawFile(fig,'grid_maps');

fig = newFigure(1000,440);
tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
failed = find(data.Outcome ~= "Success");
success = find(data.Outcome == "Success");
[~,order] = sort(data.TimeGoal(success),'descend');
chosen = [failed(1);success(order(1:2))];
colors = [.76 .16 .18;.20 .33 .69;.10 .54 .44];
ax = nexttile;
shelves(ax,p);
legends = strings(size(chosen));
traces = gobjects(size(chosen));
for j = 1:numel(chosen)
    k = chosen(j);
    trace = paths(k);
    traces(j) = drawTrail(ax,trace,colors(j,:));
    footprint(ax,trace.obs(:,1),p,colors(j,:));
    if data.Outcome(k) == "Crash"
        footprint(ax,trace.obs(:,end),p,colors(j,:));
    end
    legends(j) = sprintf('x = %g m, %g° (%s)',data.X(k),data.HeadingDeg(k),data.Outcome(k));
end
legend(ax,traces,legends,'Location','northoutside','FontSize',9);
title(ax,'Crash and two slowest arrivals');
ax = nexttile;
shelves(ax,p);
theta = data.HeadingDeg(chosen(1));
[xg,yg] = meshgrid(-5.5:.35:4.5,1.5:.35:4.25);
obs = [xg(:),yg(:),repmat([sind(theta),cosd(theta),0,0],numel(xg),1)]';
act = getAction(getActor(agent),{reshape(obs,6,1,[])});
act = reshape(act{1},2,[]);
quiver(ax,xg,yg,reshape(act(1,:),size(xg))*cosd(theta), ...
    reshape(act(1,:),size(xg))*sind(theta),.45,'Color',[.21 .21 .22],'LineWidth',.7);
scatter(ax,xg(:),yg(:),16,act(2,:),'filled');
scatter(ax,data.X(chosen),data.Y(chosen),53,'w','filled','MarkerEdgeColor','k');
clim(ax,[-1 1]);
colormap(ax,turbo);
bar = colorbar(ax);
bar.Label.String = 'Torque command';
title(ax,sprintf('Actor field at %g° and zero speed',theta));
ylim(ax,[.8 5.0]);
drawFile(fig,'grid_paths');

[~,pair] = max(mirror.mirrorPairs.AbsDelta);
fig = newFigure(1000,405);
tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
ax = nexttile;
shelves(ax,p);
leftPath = mirror.leftReplay(pair);
rightPath = mirror.rightReplay(pair);
drawTrail(ax,leftPath,colors(2,:));
drawTrail(ax,rightPath,colors(3,:));
footprint(ax,leftPath.obs(:,1),p,colors(2,:));
footprint(ax,rightPath.obs(:,1),p,colors(3,:));
title(ax,sprintf('Pair %d paths at largest time gap',pair));
ax = nexttile;
hold(ax,'on'); grid(ax,'on');
scatter(ax,mirror.mirrorPairs.LeftTime,mirror.mirrorPairs.RightTime,29,colors(2,:),'filled');
scatter(ax,mirror.mirrorPairs.LeftTime(pair),mirror.mirrorPairs.RightTime(pair),50,colors(1,:),'filled');
bounds = [min([mirror.mirrorPairs.LeftTime;mirror.mirrorPairs.RightTime])-.25,...
    max([mirror.mirrorPairs.LeftTime;mirror.mirrorPairs.RightTime])+.25];
plot(ax,bounds,bounds,'k:');
xlim(ax,bounds); ylim(ax,bounds); axis(ax,'square');
xlabel(ax,'Left time to goal (s)'); ylabel(ax,'Right time to goal (s)');
title(ax,'Times for all successful pairs');
drawFile(fig,'mirror');

settingSummary = groupsummary(rob.allResults,["Kind","Setting"], ...
    {'sum','mean'},["TimeGoal"]);
settingSummary.Successes = zeros(height(settingSummary),1);
settingSummary.Crashes = zeros(height(settingSummary),1);
for k = 1:height(settingSummary)
    rows = rob.allResults.Kind == settingSummary.Kind(k) & ...
        rob.allResults.Setting == settingSummary.Setting(k);
    settingSummary.Successes(k) = nnz(rob.allResults.Outcome(rows)=="Success");
    settingSummary.Crashes(k) = nnz(rob.allResults.Outcome(rows)=="Crash");
end
settingSummary = sortrows(settingSummary,{'Successes','Crashes','mean_TimeGoal'}, ...
    {'ascend','descend','descend'});
worst = settingSummary(1,:);
sampleCase = settingSummary(find(settingSummary.Crashes>0,1),:);
k = find(rob.kind == sampleCase.Kind & rob.cases == sampleCase.Setting);
rows = rob.allResults.Kind == sampleCase.Kind & rob.allResults.Setting == sampleCase.Setting;
caseData = rob.allResults(rows,:);
crashed = find(caseData.Outcome == "Crash");
[~,near] = min(caseData.TerminalY(crashed));
sample = crashed(near);
episode = rob.allReplay{k}(sample);
nominalIndex = find(rob.kind == "Mass" & rob.cases == 1);
reference = rob.allReplay{nominalIndex}(sample);
capped = min(max(episode.action,-1),1);
if sampleCase.Kind == "Limit"
    cap = sampleCase.Setting;
    capped = min(max(episode.action,-cap),cap);
end

fig = newFigure(1000,408);
tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
ax = nexttile;
shelves(ax,p);
l1 = drawTrail(ax,reference,colors(3,:));
l2 = drawTrail(ax,episode,colors(1,:));
footprint(ax,episode.obs(:,1),p,colors(1,:));
footprint(ax,episode.obs(:,end),p,colors(1,:));
legend(ax,[l1 l2],{'Nominal','Crash'},'Location','northoutside','Orientation','horizontal');
title(ax,sprintf('%s %.2g, start %d',sampleCase.Kind,sampleCase.Setting,sample));
ax = nexttile;
hold(ax,'on'); grid(ax,'on');
stairs(ax,episode.tAction,capped(1,:),'--','Color',colors(2,:));
stairs(ax,episode.tAction,capped(2,:),'Color',colors(1,:));
yline(ax,0,':');
ylim(ax,1.1*[-1,1]);
legend(ax,{'Applied translation','Applied rotation'},'Location','southoutside', ...
    'Orientation','horizontal','FontSize',9);
xlabel(ax,'Time (s)'); ylabel(ax,'Normalised action');
title(ax,'Commands applied to the plant');
drawFile(fig,'worst_case');

sigmaIndex = find(sensor.sigma == .6);
noiseRows = sensor.allNoise(sensor.allNoise.Sigma == sensor.sigma(sigmaIndex),:);
noiseSample = find(noiseRows.Outcome == "Success",1);
noisy = sensor.allTraces{sigmaIndex}(noiseSample);
nominalNoise = sensor.allTraces{1}(noiseSample);
obsY = noisy.obs(2,:)';
[sharedObsTime,ia,ib] = intersect(round(noisy.t,9),round(noisy.trueTime,9),'stable');
assert(numel(sharedObsTime) > 2,'True and observed logs do not share the control instants.');
positionError = obsY(ia)-noisy.trueY(ib);
fig = newFigure(930,365);
ax = axes(fig);
hold(ax,'on'); grid(ax,'on');
plot(ax,noisy.trueTime,noisy.trueY,'Color',colors(2,:),'LineWidth',2.0);
plot(ax,noisy.t,obsY,'.-','Color',colors(1,:));
yline(ax,-1.5,'k:','Goal line');
legend(ax,{'True y','Observed y'},'Location','southwest');
xlabel(ax,'Time (s)'); ylabel(ax,'Vertical position (m)');
title(ax,sprintf('Position measurements, \\sigma = %.1f m, start %d', ...
    sensor.sigma(sigmaIndex),noiseSample));
drawFile(fig,'noise_trace');

nominal = rob.allResults(rob.allResults.Kind == "Mass" & rob.allResults.Setting == 1,:);
noiseZero = sensor.allNoise(sensor.allNoise.Sigma == 0,:);
assert(isequal(nominal.Outcome,noiseZero.Outcome) && ...
    max(abs(nominal.Elapsed-noiseZero.Elapsed)) < 1e-8 && ...
    max(abs(nominal.TerminalY-noiseZero.TerminalY)) < 1e-8, ...
    'The zero-noise model differs from the nominal model.');

diag.gridPaths = struct([]);
for j = 1:length(chosen)
    k = chosen(j);
    trace = paths(k);
    diag.gridPaths(j).run = k;
    diag.gridPaths(j).initialAction = trace.action(:,1)';
    diag.gridPaths(j).lateAction = trace.action(:,end)';
    diag.gridPaths(j).terminal = trace.obs(:,end)';
    diag.gridPaths(j).xRange = [min(trace.obs(1,:)), max(trace.obs(1,:))];
    diag.gridPaths(j).yRange = [min(trace.obs(2,:)), max(trace.obs(2,:))];
    diag.gridPaths(j).samplePath = [trace.t,trace.obs([1,2,5,6],:)'];
    diag.gridPaths(j).sampleActions = [trace.tAction,trace.action'];
end
diag.actorGridCount = 5*3*24;
diag.symmetryPair = pair;
diag.worstKind = worst.Kind;
diag.worstSetting = worst.Setting;
diag.crashKind = sampleCase.Kind;
diag.crashSetting = sampleCase.Setting;
diag.crashSample = sample;
diag.crashTerminal = episode.obs(:,end)';
diag.crashInitialAction = capped(:,1)';
diag.crashApplied = [episode.tAction capped'];
diag.crashPath = [episode.t,episode.obs([1,2,5,6],:)'];
diag.crashRefPath = [reference.t,reference.obs([1,2,5,6],:)'];
diag.noiseSample = noiseSample;
diag.noiseSigma = sensor.sigma(sigmaIndex);
diag.noiseNominalTime = nominalNoise.t(end);
diag.noiseRmsError = sqrt(mean(positionError.^2));
diag.noisePeakError = max(abs(positionError));
diag.noiseTime = noisy.t(end);
diag.noiseTrueTime = noisy.trueTime(end);
diag.noiseTrueYEnd = noisy.trueY(end);
diag.noiseObservedYEnd = obsY(end);
diag.noiseStarts = noiseRows(noiseSample,:);
fid = fopen('evidence/diagnostics.json','w');
fprintf(fid,'%s\n',jsonencode(diag,'PrettyPrint',true));
fclose(fid);
end

function fig = newFigure(width,height)
fig = figure('Visible','off','Color','white','Position',[20,20,width,height]);
end

function drawFile(fig,name)
exportgraphics(fig,fullfile('evidence',[name '.png']),'Resolution',230);
close(fig);
end

function imageMap(ax,x,y,values)
im = imagesc(ax,1:numel(x),y,values);
im.AlphaData = ~isnan(values);
ax.Color = [.94 .94 .94];
set(ax,'YDir','normal','XTick',1:numel(x),'XTickLabel',compose('%g',x), ...
    'YTick',-180:45:165,'TickDir','out','Layer','top');
xline(ax,5.5,'k-');
xlabel(ax,'Start x (m)'); ylabel(ax,'Heading (°)');
end

function shelves(ax,p)
hold(ax,'on');
fill(ax,[p.L,7,7,p.L],[-p.D,-p.D,p.D,p.D],[.82 .83 .84],'EdgeColor','none');
fill(ax,[-7,-p.L,-p.L,-7],[-p.D,-p.D,p.D,p.D],[.82 .83 .84],'EdgeColor','none');
fill(ax,[-p.L,p.L,p.L,-p.L],[3*p.D,3*p.D,7,7],[.82 .83 .84],'EdgeColor','none');
yline(ax,-1.5,'k:');
axis(ax,'equal');
xlim(ax,[-5.7 5.7]); ylim(ax,[-2.3 5.4]);
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); grid(ax,'on');
end

function line = drawTrail(ax,trace,color)
line = plot(ax,trace.obs(1,:),trace.obs(2,:),'-','Color',color,'LineWidth',1.75);
scatter(ax,trace.obs(1,1),trace.obs(2,1),37,color,'o','LineWidth',1.2);
scatter(ax,trace.obs(1,end),trace.obs(2,end),29,color,'filled');
end

function footprint(ax,state,p,color)
corners = [p.h,-p.h,-p.h,p.h;p.w,p.w,-p.w,-p.w]/2;
rotated = [state(4),-state(3);state(3),state(4)]*corners + state(1:2);
patch(ax,rotated(1,:),rotated(2,:),color,'FaceAlpha',.12,'EdgeColor',color);
end
