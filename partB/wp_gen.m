function wp_list = wp_gen(x_max, y_max, N, x0, logical_map, obstacles, seed, clr)
rng(seed);
res = 10; keepout = 1.5; minSep = 6;
[nRows, nCols] = size(logical_map);

R = ceil(clr*res);
[dc, dr] = meshgrid(-R:R, -R:R);
in = dr.^2 + dc.^2 <= (clr*res)^2;
dr = dr(in); dc = dc(in);

wp_list = zeros(N,2); n = 0; att = 0;
while n < N && att < 1e5
    att = att + 1;
    if mod(att, 500) == 0, n = 0; end
    c = [randi(x_max*res)-1, randi(y_max*res)-1]/res;
    [row, col] = world_to_map(c(1), c(2), res, nRows);

    rr = row + dr; cc = col + dc;
    if any(rr < 1 | rr > nRows | cc < 1 | cc > nCols), continue, end
    if any(logical_map(sub2ind([nRows nCols], rr, cc))), continue, end
    if any(vecnorm(obstacles(:,1:2) - c, 2, 2) < keepout), continue, end
    if min(vecnorm([wp_list(1:n,:); x0] - c, 2, 2)) < minSep, continue, end

    n = n + 1; wp_list(n,:) = c;
end
assert(n == N, 'Placed %d of %d waypoints in %d attempts.', n, N, att);
end

function [row, col] = world_to_map(x, y, res, nRows)
col = round(x*res) + 1;
row = nRows - round(y*res);
end