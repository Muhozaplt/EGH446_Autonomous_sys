function r = eval_run(out, target_path, logical_map, obstacles)
% EVAL_RUN summarises tracking, estimation and clearance performance for
% one simulation run
%
% out          - simulation output from sim()
% target_path  - planned path [2 x N]
% logical_map  - uninflated occupancy grid used to build the map
% obstacles    - known obstacle positions [12 x 3], used for evaluation only

truth = out.true_log(:,1:2);        % true [X Y]
est   = out.est_log(:,1:2);         % estimated [X Y]

% use the common length in case one log has an extra sample
n = min(size(truth,1), size(est,1));
truth = truth(1:n,:);
est   = est(1:n,:);

P = target_path';                   % planned path as M x 2

% cross-track error: distance from each true position to the planned path
xt = zeros(n,1);
for i = 1:n
    xt(i) = pathDist(truth(i,:), P);
end

% wall hits: true positions inside occupied cells of the uninflated map
rawMap = binaryOccupancyMap(logical_map, 10);
inMap  = truth(:,1) >= 0 & truth(:,1) <= 52 & ...
         truth(:,2) >= 0 & truth(:,2) <= 41;
occ    = checkOccupancy(rawMap, truth(inMap,:));

% estimation error between true and estimated position
est_err = vecnorm(truth - est, 2, 2);

% requirement: robot centre at least 0.35 m from walls
% (inflate rounds up to whole cells, so the check is about 0.4 m)
r_robot = 0.20;  margin = 0.15;  r_obs = 0.15;
wallMap = binaryOccupancyMap(logical_map, 10);
inflate(wallMap, r_robot + margin);
occW = checkOccupancy(wallMap, truth(inMap,:));

% positions where the wall clearance requirement was not met
wallPts = truth(inMap,:);
wallPts = wallPts(occW == 1,:);

% requirement: robot centre at least 0.50 m from obstacle centres
% (known obstacle positions are used here for evaluation only)
d_obs_req = r_obs + r_robot + margin;
D = sqrt((truth(:,1) - obstacles(:,1)').^2 + (truth(:,2) - obstacles(:,2)').^2);
obsPts = truth(min(D, [], 2) < d_obs_req, :);

% results
r.max_crosstrack  = max(xt);
r.mean_crosstrack = mean(xt);
r.rms_crosstrack  = sqrt(mean(xt.^2));
r.wall_samples    = sum(occ == 1);
r.wall_violations = size(wallPts,1);
r.min_obs_dist    = min(D(:));
r.obs_violations  = size(obsPts,1);
r.left_map        = any(~inMap);
r.mean_est_error  = mean(est_err);
r.max_est_error   = max(est_err);

% plot planned, true and estimated paths with clearance information
figure; show(rawMap); hold on
hP = plot(P(:,1), P(:,2), 'r.-');
hT = plot(truth(:,1), truth(:,2), 'b-', 'LineWidth', 1.5);
hE = plot(est(:,1), est(:,2), 'g--');

th = linspace(0, 2*pi, 40);
for i = 1:size(obstacles,1)
    hO = plot(obstacles(i,1) + d_obs_req*cos(th), ...
              obstacles(i,2) + d_obs_req*sin(th), 'm-');
end

hW = plot(nan, nan, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
if ~isempty(wallPts)
    set(hW, 'XData', wallPts(:,1), 'YData', wallPts(:,2));
end

hV = plot(nan, nan, 'mx', 'MarkerSize', 10, 'LineWidth', 2);
if ~isempty(obsPts)
    set(hV, 'XData', obsPts(:,1), 'YData', obsPts(:,2));
end

legend([hP hT hE hO hW hV], {'Planned', 'True', 'Estimate', ...
       'Obstacle clearance', 'Wall violation', 'Obstacle violation'})
title(sprintf(['Cross-track max %.2f m, RMS %.2f m | ' ...
               'min obstacle dist %.2f m | wall violations %d'], ...
      r.max_crosstrack, r.rms_crosstrack, r.min_obs_dist, r.wall_violations))
end

function dmin = pathDist(p, P)
% shortest distance from point p to the polyline P
dmin = inf;
for k = 1:size(P,1)-1
    a = P(k,:);
    b = P(k+1,:);
    ab = b - a;
    t = max(0, min(1, dot(p-a, ab) / max(dot(ab,ab), 1e-12)));
    dmin = min(dmin, norm(p - (a + t*ab)));
end
end