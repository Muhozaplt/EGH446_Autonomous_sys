here = fileparts(mfilename('fullpath'));
root = fileparts(here);

mdl_quadrotor
quad.init_pos = [2 2 -0.5];
start_xy = quad.init_pos(1:2);
x0 = [start_xy'; 0; 0; 0; 0];

load(fullfile(root, 'complexMap_air_ground.mat'));
load(fullfile(root, 'obstacles_air_ground.mat'));

clearance = 0.9;
wp_list = wp_gen(52, 41, 5, start_xy, logical_map, obstacles, 100, clearance);

planMap = binaryOccupancyMap(logical_map, 10);
inflate(planMap, clearance);

nodes = [start_xy; wp_list];
[D, paths] = plan_pairwise(planMap, nodes);
[order, routeCost] = order_wp(D);
wp_ordered = wp_list(order,:);
target_path = build_path(paths, order);