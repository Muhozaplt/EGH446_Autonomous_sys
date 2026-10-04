function [vx_star, vy_star, yawrate_star, done, avoid_shift] = ...
    Guidance_Control_replay(X, Y, yaw, target_path, obs, wall_grid)

% GUIDANCE_CONTROL_REPLAY is an offline copy of Guidance_Control with
% print statements for each detour decision. It is used to replay logged
% data and is not used in the Simulink model.
%
% Inputs:
%   X, Y         - current quadrotor position in the global frame [m]
%   yaw          - current quadrotor heading [rad]
%   target_path  - ordered waypoint coordinates [2 x N]
%   obs          - remembered obstacle positions from the sensor [12 x 2]
%   wall_grid    - inflated wall occupancy grid, row 1 at the top
%
% Outputs:
%   vx_star      - commanded body-frame x velocity [m/s]
%   vy_star      - commanded body-frame y velocity [m/s]
%   yawrate_star - commanded yaw rate [rad/s]
%   done         - 1 once the final waypoint has been reached
%   avoid_shift  - lateral shift applied for obstacle avoidance [m]

% controller and guidance tuning parameters
k_yaw = 0.3;                % proportional yaw-error gain
k_pos = 0.5;                % proportional position-error gain
v_max = 2.0;                % maximum horizontal command speed [m/s]
capture_radius = 0.5;       % intermediate waypoint capture radius [m]
final_capture_radius = 0.5; % final waypoint acceptance radius [m]
d_look_ahead = 1.0;         % RVWP look-ahead distance [m]
yaw_hold = false;           % true commands zero yaw rate (diagnostic test)

% obstacle avoidance parameters
R_safe = 1.0;               % preferred distance from obstacle centres [m]
d_min = 0.7;                % minimum distance from obstacle centres [m]
room_ok = 0.8;              % spare room beyond R_safe for a side to count as comfortable [m]
d_pre = 1.5;                % distance over which the detour ramps in [m]
d_post = 1.0;               % distance over which the detour ramps out [m]

% persistent variables retain progression between simulation steps
persistent n mission_complete pass_set pass_xy last_shift

if isempty(n)
    n = 1;
end

if isempty(mission_complete)
    mission_complete = false;
end

if isempty(pass_set)
    pass_set = false(size(obs,1), 1);   % true once a passing point is chosen
    pass_xy = zeros(size(obs,1), 2);    % passing point beside each obstacle [m]
    last_shift = 0;                     % shift applied on the previous step [m]
end

% extract x and y coordinates from the ordered waypoint path
Wx = target_path(1,:);
Wy = target_path(2,:);

num_waypoints = size(target_path,2);

% prevent waypoint indexing beyond the final path segment
if n >= num_waypoints
    n = num_waypoints - 1;
end

% current quadrotor position and heading
x = X;
y = Y;
psi = yaw;

% current waypoint and next waypoint defining the active path segment
xn = Wx(n);
yn = Wy(n);
xn1 = Wx(n+1);
yn1 = Wy(n+1);

% segment geometry and along-track position of the vehicle
theta = atan2(yn1-yn, xn1-xn);
L = sqrt((xn1-xn)^2 + (yn1-yn)^2);
R = (x - xn)*cos(theta) + (y - yn)*sin(theta);

% distance from the quadrotor to the next waypoint
d = sqrt((xn1-x)^2 + (yn1-y)^2);

% while a detour is active the vehicle is offset from the corner, so also
% advance once it is within the capture radius of the segment end
detouring = abs(last_shift) > 0.1;

% advance through every waypoint that has been captured or passed
while n < num_waypoints-1 && ...
        (d < capture_radius || R >= L || (detouring && R >= L - capture_radius))

    n = n + 1;

    xn = Wx(n);
    yn = Wy(n);
    xn1 = Wx(n+1);
    yn1 = Wy(n+1);

    theta = atan2(yn1-yn, xn1-xn);
    L = sqrt((xn1-xn)^2 + (yn1-yn)^2);
    R = (x - xn)*cos(theta) + (y - yn)*sin(theta);
    d = sqrt((xn1-x)^2 + (yn1-y)^2);
end

% determine whether the quadrotor is currently tracking the final segment
at_final_segment = (n == num_waypoints - 1);


%% RVWP guidance

% place the virtual waypoint d_look_ahead metres ahead of the projected
% position, limited to the active segment so it never passes waypoint n+1
if L > 1e-6
    s = min(max(R + d_look_ahead, 0), L);
    S_x = xn + s*cos(theta);
    S_y = yn + s*sin(theta);
else
    S_x = xn1;
    S_y = yn1;
end


%% obstacle avoidance

% unit normal to the active segment (pointing left)
nx = -sin(theta);
ny = cos(theta);

% lateral shift of the virtual waypoint, from the most demanding obstacle
shift = 0;

for i = 1:size(obs,1)

    ox = obs(i,1);
    oy = obs(i,2);

    if isnan(ox)
        continue
    end

    % obstacle position relative to the active segment
    Ro = (ox - xn)*cos(theta) + (oy - yn)*sin(theta);   % along-track
    eo = (ox - xn)*nx + (oy - yn)*ny;                   % cross-track, left positive

    % only obstacles close to the active segment need a detour
    if abs(eo) >= R_safe || Ro < -R_safe || Ro > L + R_safe
        continue
    end

    % ramp the detour in before the obstacle and out after it
    w_in  = (R - (Ro - R_safe - d_pre))/d_pre;
    w_out = ((Ro + R_safe + d_post) - R)/d_post;
    w = min([1, w_in, w_out]);

    if w <= 0
        continue
    end

    % choose the passing point once per obstacle, in world coordinates
    if ~pass_set(i)
        [side, d_pass] = chooseSide(wall_grid, xn, yn, theta, Ro, eo, ...
                                    R_safe, d_min, room_ok);
        pass_xy(i,1) = ox + side*d_pass*nx;
        pass_xy(i,2) = oy + side*d_pass*ny;
        pass_set(i) = true;

        fprintf(['obs %d at (%.2f, %.2f)  segment %d  Ro %.2f  eo %.2f  ' ...
                 'side %+d  d_pass %.2f  pass (%.2f, %.2f)\n'], ...
                i, ox, oy, n, Ro, eo, side, d_pass, pass_xy(i,1), pass_xy(i,2));
    end

    % lateral position of the passing point relative to the active segment
    e_ref = (pass_xy(i,1) - xn)*nx + (pass_xy(i,2) - yn)*ny;

    if abs(w*e_ref) > abs(shift)
        shift = w*e_ref;
    end
end

last_shift = shift;
avoid_shift = shift;

% shift the virtual waypoint sideways onto the detour
S_x = S_x + shift*nx;
S_y = S_y + shift*ny;

% calculate the desired heading from the vehicle toward the virtual waypoint
psi_d = atan2(S_y-y, S_x-x);


%% yaw control

% calculate the shortest wrapped heading error in the range [-pi, pi]
yaw_error = angdiff(psi, psi_d);

% proportional yaw controller generates the desired yaw-rate command
if yaw_hold
    yawrate_star = 0;
else
    yawrate_star = k_yaw*yaw_error;
end


%% velocity control

% calculate position error between the quadrotor and virtual waypoint
ex = S_x - x;
ey = S_y - y;

% proportional position controller generates global-frame velocity commands
vx_world = k_pos*ex;
vy_world = k_pos*ey;

% limit the velocity magnitude to the selected maximum speed
speed = sqrt(vx_world^2 + vy_world^2);

if speed > v_max
    scale = v_max/speed;
    vx_world = vx_world*scale;
    vy_world = vy_world*scale;
end

% rotate global-frame velocity commands into the quadrotor body frame
vx_star =  cos(psi)*vx_world + sin(psi)*vy_world;
vy_star = -sin(psi)*vx_world + cos(psi)*vy_world;


%% mission completion

% mark the mission complete once the final waypoint is reached
if at_final_segment && d < final_capture_radius
    mission_complete = true;
end

% command zero velocity and yaw rate once the mission is complete
if mission_complete
    vx_star = 0;
    vy_star = 0;
    yawrate_star = 0;
end

% signal mission completion to stop the simulation
done = double(mission_complete);

end


function [side, d_pass] = chooseSide(grid, xn, yn, theta, Ro, eo, R_safe, d_min, room_ok)
% CHOOSESIDE picks the detour side and passing distance from the free
% space between the obstacle and the walls on each side

% free space on each side, taken as the smallest of three cross-sections
% (before, alongside and after the obstacle)
gapL = inf;
gapR = inf;
for t = [Ro - R_safe, Ro, Ro + R_safe]
    gapL = min(gapL, sideGap(grid, xn, yn, theta, t, eo,  1));
    gapR = min(gapR, sideGap(grid, xn, yn, theta, t, eo, -1));
end

% the shorter detour passes on the side opposite the obstacle
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

% a side is comfortable when it has room_ok to spare beyond R_safe
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

% pass at R_safe when there is room, otherwise centred in the gap,
% but never closer than d_min
d_pass = min(R_safe, gap/2);
d_pass = max(d_pass, d_min);

fprintf('    gapL %.2f  gapR %.2f\n', gapL, gapR);

end


function gap = sideGap(grid, xn, yn, theta, t, eo, side)
% SIDEGAP measures the free distance from the obstacle's cross-track
% position to the inflated walls, perpendicular to the segment

gap_max = 5.0;              % search distance [m]
step = 0.1;                 % search step [m]

c = cos(theta);
s = sin(theta);

% point level with the obstacle at along-track position t
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
% ISOCCUPIED looks up a world position in the occupancy grid
% (10 cells per metre, row 1 at the top of the map)

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