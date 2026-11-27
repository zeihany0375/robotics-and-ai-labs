function [collision, clearance] = startClearance(starts, p)
% Repeat the model's obstacle predicates for the initial true footprint.
corners = [p.h,-p.h,-p.h,p.h; p.w,p.w,-p.w,-p.w]/2;
collision = false(size(starts,1),1);
clearance = zeros(size(starts,1),1);
for k = 1:size(starts,1)
    pose = starts(k,:);
    c = cos(pose(3));
    s = sin(pose(3));
    points = [c,-s;s,c]*corners + pose(1:2)';
    x = points(1,:);
    y = points(2,:);
    collision(k) = any(inpolygon([-p.L;p.L], [p.D;p.D], x, y)) || ...
        any(y < p.D & abs(x) > p.L) || ...
        any(inpolygon([-p.L;p.L], 3*[p.D;p.D], x, y)) || ...
        any(y > 3*p.D & abs(x) < p.L);
    clearance(k) = min(min(y)-p.D, 3*p.D-max(y));
end
end
