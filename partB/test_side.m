% TEST_SIDE replays the obstacle side choice offline for each obstacle near
% the planned path and prints the values the guidance would use.
% Run setupPartB first so target_path, obstacles and wall_grid exist.

R_safe = 1.0;  d_min = 0.7;  room_ok = 0.8;

Wx = target_path(1,:);
Wy = target_path(2,:);

for i = 1:size(obstacles,1)
    ox = obstacles(i,1);
    oy = obstacles(i,2);

    for n = 1:numel(Wx)-1
        xn = Wx(n);  yn = Wy(n);
        theta = atan2(Wy(n+1) - yn, Wx(n+1) - xn);
        L = hypot(Wx(n+1) - xn, Wy(n+1) - yn);
        nx = -sin(theta);  ny = cos(theta);

        Ro = (ox - xn)*cos(theta) + (oy - yn)*sin(theta);
        eo = (ox - xn)*nx + (oy - yn)*ny;

        if abs(eo) >= R_safe || Ro < -R_safe || Ro > L + R_safe
            continue
        end

        [side, d_pass, gapL, gapR] = chooseSideDbg(wall_grid, xn, yn, theta, ...
                                                   Ro, eo, R_safe, d_min, room_ok);
        pass = [ox + side*d_pass*nx, oy + side*d_pass*ny];

        fprintf(['obstacle (%g, %g)  segment %d  Ro %.2f  L %.2f  eo %.2f  ' ...
                 'gapL %.2f  gapR %.2f  side %+d  d_pass %.2f  pass (%.2f, %.2f)\n'], ...
                ox, oy, n, Ro, L, eo, gapL, gapR, side, d_pass, pass(1), pass(2));
    end
end


function [side, d_pass, gapL, gapR] = chooseSideDbg(grid, xn, yn, theta, Ro, eo, R_safe, d_min, room_ok)
% copy of chooseSide that also returns the measured gaps

gapL = inf;
gapR = inf;
for t = [Ro - R_safe, Ro, Ro + R_safe]
    gapL = min(gapL, sideGap(grid, xn, yn, theta, t, eo,  1));
    gapR = min(gapR, sideGap(grid, xn, yn, theta, t, eo, -1));
end

if eo > 0
    pref = -1;
else
    pref = 1;
end

if pref == 1
    gapPref = gapL;  gapAlt = gapR;
else
    gapPref = gapR;  gapAlt = gapL;
end

if gapPref >= R_safe + room_ok
    side = pref;
    gap = gapPref;
elseif gapAlt >= R_safe + room_ok
    side = -pref;
    gap = gapAlt;
elseif gapPref >= gapAlt
    side = pref;
    gap = gapPref;
else
    side = -pref;
    gap = gapAlt;
end

d_pass = min(R_safe, gap/2);
d_pass = max(d_pass, d_min);
end


function gap = sideGap(grid, xn, yn, theta, t, eo, side)
gap_max = 5.0;
step = 0.1;
c = cos(theta);
s = sin(theta);
bx = xn + t*c - eo*s;
by = yn + t*s + eo*c;
gap = gap_max;
for k = 1:round(gap_max/step)
    dist = k*step;
    px = bx - side*dist*s;
    py = by + side*dist*c;
    if isOccupied(grid, px, py)
        gap = dist;
        return
    end
end
end


function occ = isOccupied(grid, px, py)
res = 10;
nr = size(grid,1);
nc = size(grid,2);
col = floor(px*res) + 1;
row = nr - floor(py*res);
if row < 1 || row > nr || col < 1 || col > nc
    occ = true;
else
    occ = grid(row, col) ~= 0;
end
end