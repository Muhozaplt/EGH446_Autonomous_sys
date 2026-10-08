function [newP, newT, newN, status, r_used] = replan_local(pos, P, T, N, m, obs, MAXN)
% REPLAN_LOCAL plans a local detour around detected obstacles and splices
% it onto the remaining path. It is called from Guidance_Control through
% coder.extrinsic when a detected obstacle lies close to the path ahead.
%
% The room between the detected obstacle and the walls is measured from
% the wall map, perpendicular to the direction of travel, and the wall
% clearance and obstacle radius used for planning are sized to fit that
% room, so a single A* search is normally enough. A* is limited to a
% window around the vehicle and the rejoin point, so a blocked passage
% fails quickly and cannot produce a long detour back the way the vehicle
% came.
%
% Inputs:
%   pos   - current estimated position [x y]
%   P     - current path [2 x MAXN], first N columns valid
%   T     - target waypoint flags for the path [1 x MAXN]
%   N     - number of valid path points
%   m     - path segment that passes closest to the obstacle
%   obs   - remembered (detected) obstacle positions [12 x 2], NaN when unused
%   MAXN  - size of the path arrays
%
% Outputs:
%   newP   - new path [2 x MAXN], first newN columns valid
%   newT   - target waypoint flags for the new path [1 x MAXN]
%   newN   - number of valid points in the new path
%   status - 1 if a new path was found, 0 otherwise

newP = zeros(2, MAXN);
newT = false(1, MAXN);
newN = 0;
status = 0;
r_used = 0;

% requirements and planning parameters
r_req = 0.5;                % obstacle requirement, centre to centre [m]
r_max = 1.3;                % largest obstacle radius used for planning [m]
r_min = 0.8;                % smallest preferred obstacle radius [m]
grid_margin = 0.1;          % one grid cell, so the passage is never zero width [m]
r_clear = 0.4;              % area cleared around the start and goal [m]
win_margin = 4.0;           % margin around the start and goal for the A* window [m]
room_max = 6.0;             % furthest distance checked when measuring room [m]

% maps prepared by setupPartB, from largest to smallest wall clearance
planMap  = evalin('base', 'planMap');
baseMap  = evalin('base', 'baseMap');
avoidMap = evalin('base', 'avoidMap');
rawMap   = binaryOccupancyMap(evalin('base', 'logical_map'), 10);

wallMaps  = {planMap, baseMap, avoidMap};
wallClear = [evalin('base', 'plan_clearance'), evalin('base', 'clearance'), 0.6];

obs = obs(~isnan(obs(:,1)), :);
pos = pos(:)';

% detected obstacle that conflicts with the path at segment m
a = P(:,m)';
b = P(:,m+1)';
dseg = zeros(size(obs,1), 1);
for i = 1:size(obs,1)
    dseg(i) = pointSegDist(obs(i,:), a, b);
end
[~, io] = min(dseg);
o = obs(io,:);

fprintf('Obstacle selected for replanning: (%.2f, %.2f)\n', ...
    o(1), o(2));

% rejoin point: first path point after the conflicting segment that is
% clear of the obstacles, without going past a target waypoint
k = m + 1;
while k < N && ~T(k) && ...
        min(hypot(obs(:,1) - P(1,k), obs(:,2) - P(2,k))) < r_max + 0.3
    k = k + 1;
end
goal = P(:,k)';

% widest passage past the obstacle, measured across the direction of travel
t_dir = (goal - pos)/max(norm(goal - pos), 1e-6);
room = sideRoom(rawMap, o, t_dir, room_max);

% largest wall clearance that still leaves room for an r_min disc,
% then the largest obstacle radius that fits alongside it
w_sel = 0;
r_sel = 0;
for w = 1:numel(wallMaps)
    r = min(r_max, room - wallClear(w) - grid_margin);
    if r >= r_min
        w_sel = w;
        r_sel = r;
        break
    end
end

% very tight passage: smallest wall clearance and whatever radius fits
if w_sel == 0
    w_sel = numel(wallMaps);
    r_sel = max(room - wallClear(w_sel) - grid_margin, r_req + 0.05);
end

[best_path, ok] = planDetour(wallMaps{w_sel}, avoidMap, obs, r_sel, ...
                             pos, goal, r_clear, win_margin);

% fallback if the sized plan does not connect within the window
if ~ok
    for w = 1:numel(wallMaps)
        for r = [1.0, 0.8, 0.6]
            [best_path, ok] = planDetour(wallMaps{w}, avoidMap, obs, r, ...
                                         pos, goal, r_clear, win_margin);
            if ok
                w_sel = w;
                r_sel = r;
                break
            end
        end
        if ok
            break
        end
    end

    if ok
        fprintf('Sized plan did not connect, used fallback\n');
    end
end

if ~ok
    fprintf('Replanning failed at (%.1f, %.1f), keeping current path\n', pos(1), pos(2));
    return
end

% splice: current position, detour corners, rest of the path
seg = [pos; best_path(2:end-1,:); P(:,k:N)'];
tg  = [false; false(size(best_path,1) - 2, 1); T(k:N)'];

if size(seg,1) > MAXN
    fprintf('Replanning produced too many points, keeping current path\n');
    return
end

newN = size(seg,1);
newP(:,1:newN) = seg';
newT(1:newN) = tg';
status = 1;
r_used = r_sel;

% keep every replanned path in the workspace for plotting
try
    hist = evalin('base', 'replan_history');
catch
    hist = {};
end
hist{end+1} = seg';
assignin('base', 'replan_history', hist);

% keep the latest path in the workspace for plotting
assignin('base', 'current_path', seg');

fprintf(['Replanned at (%.1f, %.1f): room %.1f m, obstacle radius %.1f m, ' ...
         'wall clearance %.1f m, %d path points\n'], ...
        pos(1), pos(2), room, r_sel, wallClear(w_sel), newN);

end


function [sparse, ok] = planDetour(wallMap, avoidMap, obs, r, pos, goal, r_clear, win_margin)
% PLANDETOUR runs A* around the detected obstacles within a local window
% and prunes the result to its corners

sparse = [];
ok = false;

% Walls plus detected obstacles
map = copy(wallMap);
addDiscs(map, obs, r);

% Only clear start/goal cells if they are outside
% the required obstacle clearance region.
if all(vecnorm(obs - pos, 2, 2) >= r)
    clearDisc(map, avoidMap, pos, r_clear);
end

if all(vecnorm(obs - goal, 2, 2) >= r)
    clearDisc(map, avoidMap, goal, r_clear);
end

% Restore obstacle inflation so clearing cannot
% remove any part of an obstacle's safety region.
addDiscs(map, obs, r);

% only allow the search inside a window around the start and goal
map = limitToWindow(map, pos, goal, win_margin);

try
    planner = plannerAStarGrid(map);
    g = plan(planner, world2grid(map, pos), world2grid(map, goal));
catch
    return
end

if isempty(g)
    return
end

dense = grid2world(map, g);
sparse = pruneLOS(dense, map, false(size(dense,1), 1));
ok = true;

end


function room = sideRoom(map, o, t, gmax)
% SIDEROOM measures the widest passage past an obstacle, perpendicular to
% the direction of travel t, taking the narrowest of three cross-sections
% on each side

n = [-t(2), t(1)];
roomL = inf;
roomR = inf;

for s = [-0.5, 0, 0.5]
    c = o + s*t;
    roomL = min(roomL, rayGap(map, c,  n, gmax));
    roomR = min(roomR, rayGap(map, c, -n, gmax));
end

room = max(roomL, roomR);

end


function g = rayGap(map, o, u, gmax)
% RAYGAP returns the free distance from a point along a direction
d = (0.1:0.1:gmax)';
pts = o + d*u;
occ = checkOccupancy(map, pts);
idx = find(occ ~= 0, 1);

if isempty(idx)
    g = gmax;
else
    g = d(idx);
end
end


function map = limitToWindow(map, pos, goal, margin)
% LIMITTOWINDOW marks everything outside a box around pos and goal as occupied

M = occupancyMatrix(map);
res = map.Resolution;
[nr, nc] = size(M);

xmin = min(pos(1), goal(1)) - margin;
xmax = max(pos(1), goal(1)) + margin;
ymin = min(pos(2), goal(2)) - margin;
ymax = max(pos(2), goal(2)) + margin;

cmin = max(1,  floor(xmin*res) + 1);
cmax = min(nc, ceil(xmax*res));
rmin = max(1,  nr - ceil(ymax*res) + 1);
rmax = min(nr, nr - floor(ymin*res));

outside = true(nr, nc);
outside(rmin:rmax, cmin:cmax) = false;
M(outside) = true;

map = binaryOccupancyMap(M, res);

end


function d = pointSegDist(p, a, b)
% shortest distance from point p to segment ab
ab = b - a;
L2 = dot(ab, ab);
if L2 < 1e-12
    t = 0;
else
    t = max(0, min(1, dot(p - a, ab)/L2));
end
d = norm(p - (a + t*ab));
end


function addDiscs(map, obs, r)
% mark a disc of radius r around each obstacle as occupied
[gx, gy] = meshgrid(-r:0.05:r);
offsets = [gx(:) gy(:)];
offsets = offsets(vecnorm(offsets, 2, 2) <= r, :);

for i = 1:size(obs,1)
    pts = obs(i,:) + offsets;
    setOccupancy(map, pts, 1);
end
end


function clearDisc(map, wallMap, centre, r)
% free a small disc around a point, except cells too close to walls
[gx, gy] = meshgrid(-r:0.05:r);
offsets = [gx(:) gy(:)];
offsets = offsets(vecnorm(offsets, 2, 2) <= r, :);

pts = centre + offsets;
isFree = checkOccupancy(wallMap, pts) == 0;
setOccupancy(map, pts(isFree,:), 0);
end