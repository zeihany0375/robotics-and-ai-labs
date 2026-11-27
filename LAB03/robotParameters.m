function params = robotParameters(model)
workspace = get_param(model, 'ModelWorkspace');
for key = ["D","L","h","w","m","dt"]
    params.(key) = getVariable(workspace, char(key));
end
end
