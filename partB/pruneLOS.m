function wp = pruneLOS(p, map, keep)
% PRUNELOS reduces a dense path to the points needed to avoid obstacles
%
% p     - dense path [N x 2]
% map   - inflated occupancy map used for planning
% keep  - [N x 1] logical, true for points that must stay (targets)
%
% wp    - pruned path [M x 2]

N = size(p,1);
wp = p(1,:);
i = 1;

while i < N
    % never skip past the next point that must be kept
    nextKeep = find(keep(i+1:end), 1) + i;
    if isempty(nextKeep)
        jmax = N;
    else
        jmax = nextKeep;
    end

    % furthest point visible from p(i) without crossing an occupied cell
    j = jmax;
    while j > i+1 && ~losFree(map, p(i,:), p(j,:))
        j = j - 1;
    end

    wp(end+1,:) = p(j,:); %#ok<AGROW>
    i = j;
end
end

function free = losFree(map, a, b)
% check the straight line from a to b at half-cell spacing
n = ceil(norm(b-a)/(0.5/map.Resolution)) + 1;
pts = [linspace(a(1),b(1),n)' linspace(a(2),b(2),n)'];
free = ~any(checkOccupancy(map, pts) == 1);
end