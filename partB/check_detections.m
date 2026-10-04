% CHECK_DETECTIONS converts logged detections to world coordinates and
% plots them against the known obstacle positions.
% Run after run_test, which leaves out, logical_map and obstacles in the
% workspace.

det  = out.det_log.signals.values;           % rows x 3 x N, padded to max size
dims = out.det_log.signals.valueDimensions;  % N x 2, actual size at each step
pose = out.true_log;                         % N x 3 [X Y yaw]
n = min(size(det,3), size(pose,1));

pts = [];
for k = 1:n
    for j = 1:dims(k,1)                      % only the valid detection rows
        rng_j = det(j,1,k);
        ang_j = det(j,2,k);
        pts(end+1,:) = pose(k,1:2) + ...
            rng_j*[cos(pose(k,3) + ang_j), sin(pose(k,3) + ang_j)]; %#ok<AGROW>
    end
end

figure; show(binaryOccupancyMap(logical_map, 10)); hold on
plot(obstacles(:,1), obstacles(:,2), 'ms', 'MarkerSize', 12, 'LineWidth', 2)
if ~isempty(pts)
    plot(pts(:,1), pts(:,2), 'g.')
end
legend('Known obstacles', 'Detections in world frame')
title(sprintf('%d detections logged', size(pts,1)))