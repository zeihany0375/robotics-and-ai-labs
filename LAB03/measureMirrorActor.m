function measureMirrorActor(actor)
[x,y,t] = ndgrid(-4.5:.5:-2.5,2.7:.3:3.3,deg2rad(-180:15:165));
actorStates = [x(:),y(:),sin(t(:)),cos(t(:)),zeros(numel(x),2)];
mapping = [-1,1,1,-1,1,-1];
commands = queryActor(actor,actorStates');
pairedCommands = queryActor(actor,(actorStates.*mapping)');
delta = abs(pairedCommands-commands.*[1;-1]);
metrics = array2table([mean(delta,2),max(delta,[],2)], ...
    'VariableNames',{'MeanAbs','MaxAbs'});
metrics.Action = ["Translation";"Rotation"];
writetable(metrics,'evidence/mirror_actor.csv');
raw = array2table([x(:),y(:),rad2deg(t(:)),commands',pairedCommands',delta'], ...
    'VariableNames',{'X','Y','HeadingDeg','FTrans','FRot','MirroredFTrans', ...
    'MirroredFRot','AbsTranslationError','AbsRotationError'});
writetable(raw,'evidence/mirror_actor_grid.csv');
end

function commands = queryActor(actor, state)
raw = getAction(actor,{reshape(state,6,1,[])});
commands = reshape(raw{1},2,[]);
end
