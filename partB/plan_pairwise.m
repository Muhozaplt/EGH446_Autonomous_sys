function [D, paths] = plan_pairwise(planMap, nodes)
n = size(nodes,1);
D = zeros(n); paths = cell(n);
planner = plannerAStarGrid(planMap);
rc = world2grid(planMap, nodes);
step = 0.05;

for i = 1:n-1
    for j = i+1:n
        g = plan(planner, rc(i,:), rc(j,:));
        assert(~isempty(g), 'No path %d->%d', i, j);
        w = grid2world(planMap, g);
        w(1,:) = nodes(i,:); w(end,:) = nodes(j,:);
        w = shortcut(w, planMap, step);
        L = sum(vecnorm(diff(w),2,2));
        D(i,j) = L; D(j,i) = L;
        paths{i,j} = w; paths{j,i} = flipud(w);
    end
end
end

function out = shortcut(pts, planMap, step)
n = size(pts,1);
if n < 3, out = pts; return, end
out = pts(1,:); i = 1;
while i < n
    j = i + 1;
    while j+1 <= n && los(pts(i,:), pts(j+1,:), planMap, step)
        j = j + 1;
    end
    out(end+1,:) = pts(j,:);
    i = j;
end
end

function tf = los(a, b, planMap, step)
k = max(2, ceil(norm(b-a)/step) + 1);
t = linspace(0,1,k).';
tf = ~any(getOccupancy(planMap, a + t.*(b-a)) > 0.5);
end