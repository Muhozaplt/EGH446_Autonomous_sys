here = fileparts(mfilename('fullpath'));
root = fileparts(here);

mdl_quadrotor
quad.init_pos = [2 2 -0.5];
start_xy = quad.init_pos(1:2);
x0 = [start_xy'; 0; 0; 0; 0];

load(fullfile(root, 'complexMap_air_ground.mat'));
load(fullfile(root, 'obstacles_air_ground.mat'));

clearance = 0.9;          % used by wp_gen for waypoint placement [m]
plan_clearance = 1.2;     % used for path planning around walls [m]
prune_margin = 0.4;       % extra clearance for pruned path segments [m]

% generate random waypoints that do not overlap walls or obstacles
wp_list = wp_gen(52, 41, 5, start_xy, logical_map, obstacles, 100, clearance);
nodes = [start_xy; wp_list];

% planning map: walls only, inflated for clearance (no obstacle information)
planMap = binaryOccupancyMap(logical_map, 10);
inflate(planMap, plan_clearance);

% nodes closer than plan_clearance to a wall sit inside the inflated area,
% so clear a small area around each node using the base clearance map
baseMap = binaryOccupancyMap(logical_map, 10);
inflate(baseMap, clearance);

r_free = (plan_clearance - clearance) + 0.2;      % radius cleared around each node [m]
[gx, gy] = meshgrid(-r_free:0.1:r_free);
offsets = [gx(:) gy(:)];
offsets = offsets(vecnorm(offsets, 2, 2) <= r_free, :);

for i = 1:size(nodes,1)
    pts = nodes(i,:) + offsets;
    isFree = checkOccupancy(baseMap, pts) == 0;
    setOccupancy(planMap, pts(isFree,:), 0);
end

% plan between every pair of nodes and find the shortest visiting order
[D, paths] = plan_pairwise(planMap, nodes);
[order, routeCost] = order_wp(D);
wp_ordered = wp_list(order,:);
target_path = build_path(paths, order);

% prune the dense path to its corners, keeping every waypoint,
% against a map with extra margin so straight segments keep away from walls
pruneMap = binaryOccupancyMap(logical_map, 10);
inflate(pruneMap, plan_clearance + prune_margin);

P = target_path';
keep = ismembertol(P, wp_ordered, 1e-3, 'ByRows', true, 'DataScale', 1);
target_path = pruneLOS(P, pruneMap, keep)';