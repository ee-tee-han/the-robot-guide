close all;

data = load('park.mat', 'Acceleration', 'AngularVelocity', 'Position');

lat = data.Position.latitude;
long = data.Position.longitude;
acceleration = data.Acceleration;
angularVelocity = data.AngularVelocity;

% convert GPS to local XY in metres (flat earth approximation, good enough
% for a small area like QEOP)
x = (long - long(1)) * 111320 * cosd(lat(1));
y = (lat - lat(1)) * 110540;

map = binaryOccupancyMap(1200,1200,0.25);

% x comes out negative (we're west of origin) so shift everything right
x_shifted = x + 1050;
y_shifted = y + 200;

setOccupancy(map, [x_shifted y_shifted], 0)

%% Key points - detected by spinning 360 degrees at each landmark ----------
v = angularVelocity.Z;
fs = 10;        % gyro sample rate (Hz)
dt = 1/fs;
windowSize = 10 * fs;   % 10s window
threshold = 3;          % need at least 3 full rotations in the window

delta_theta = abs(v) * dt;
rotations_in_window = movsum(delta_theta, windowSize) / (2*pi);

spin_index = find(rotations_in_window > threshold);

d = diff(spin_index);
breaks = [0; find(d > 1); length(spin_index)];

unique_spin_idx = [];
for i = 1:length(breaks)-1
    segment = spin_index(breaks(i)+1 : breaks(i+1));
    mid_idx = segment(round(length(segment)/2));
    unique_spin_idx(end+1) = mid_idx;
end

% gyro and GPS run at different rates so scale indices across
gyro_len = length(v);
pos_len  = length(x_shifted);
scale    = pos_len / gyro_len;

pos_index = round(unique_spin_idx * scale);
pos_index(pos_index < 1)       = 1;
pos_index(pos_index > pos_len) = pos_len;

lat5 = y_shifted(pos_index);
lon5 = x_shifted(pos_index);

setOccupancy(map, [lon5 lat5], 1)

%% Signal points - detected by standing still for 2+ seconds --------------
speed_threshold     = 0.2;   % m/s
gyro_threshold      = 0.1;   % rad/s
min_stationary_secs = 2;

% compute GPS speed using haversine between consecutive points
R = 6371000;
lat_rad = deg2rad(lat);
lon_rad = deg2rad(long);
dlat = diff(lat_rad);
dlon = diff(lon_rad);
a    = sin(dlat/2).^2 + cos(lat_rad(1:end-1)).*cos(lat_rad(2:end)).*sin(dlon/2).^2;
c    = 2 * atan2(sqrt(a), sqrt(1-a));
dist = R * c;

if exist('gps_time','var')
    gps_dt_mean = mean(diff(gps_time));
else
    gps_dt_mean = 1;
    gps_time = (0:length(lat)-1)' * gps_dt_mean;
end
speed = [0; dist] ./ gps_dt_mean;

gyro_speed = abs(v(:));
if ~exist('gyro_time','var')
    gyro_time = (0:length(gyro_speed)-1)' * dt;
end

% resample GPS speed onto gyro timebase so both signals line up
speed_on_gyro = interp1(gps_time, speed, gyro_time, 'linear', 'extrap');

% both speed AND rotation need to be low to count as stationary
stationary_mask = (speed_on_gyro <= speed_threshold) & (gyro_speed <= gyro_threshold);

% throw away very short stationary blips - need at least 2 seconds
min_samples = ceil(min_stationary_secs / mean(diff(gyro_time)));
dch    = diff([0; stationary_mask; 0]);
starts = find(dch ==  1);
ends   = find(dch == -1) - 1;
dur    = ends - starts + 1;
valid  = dur >= min_samples;
starts = starts(valid);
ends   = ends(valid);

% map each stationary segment back to its GPS position
nseg = numel(starts);
stationary_positions = zeros(nseg, 2);
stationary_xy        = zeros(nseg, 2);
for k = 1:nseg
    mid_t   = gyro_time(round((starts(k)+ends(k))/2));
    gps_idx = round(interp1(gps_time, (1:length(gps_time))', mid_t, 'nearest', 'extrap'));
    gps_idx = max(1, min(length(lat), gps_idx));
    stationary_positions(k,:) = [lat(gps_idx), long(gps_idx)];
    stationary_xy(k,:)        = [x_shifted(gps_idx), y_shifted(gps_idx)];
end

lon4 = stationary_xy(:,1);
lat4 = stationary_xy(:,2);

% merge any signal points that ended up within 30m of each other
% (happens when you stop twice in almost the same spot)
merge_radius = 30;
keep = true(length(lon4), 1);
for i = 1:length(lon4)
    if ~keep(i); continue; end
    for j = i+1:length(lon4)
        if ~keep(j); continue; end
        d = norm([lon4(i)-lon4(j), lat4(i)-lat4(j)]);
        if d < merge_radius
            lon4(i) = (lon4(i) + lon4(j)) / 2;
            lat4(i) = (lat4(i) + lat4(j)) / 2;
            keep(j) = false;
            fprintf('Merged S%d into S%d (were %.0fm apart)\n', j, i, d);
        end
    end
end
lon4 = lon4(keep);
lat4 = lat4(keep);
fprintf('Signal points after merging: %d\n', length(lon4));

setOccupancy(map, [lon4 lat4], 1)

%% Static obstacles --------------------------------------------------------

% West Ham Stadium outline (hand-traced from Google Maps)
lat_stadium = [51.5377 51.5374 51.5374 51.5378 51.5381 51.5382 51.5386 51.5387 ...
               51.5394 51.5398 51.5400 51.5401 51.5399 51.5396 51.5392 51.5383 51.5378];
lon_stadium = [-0.0152 -0.0159 -0.0169 -0.0177 -0.0182 -0.01848 -0.0185 -0.0186 ...
               -0.0181 -0.0176 -0.01697 -0.01634 -0.01551 -0.01513 -0.01476 -0.01459 -0.01493];

origin_lat = lat(1);
origin_lon = long(1);
wgs84 = wgs84Ellipsoid;

[x_stadium, y_stadium, ~] = geodetic2enu(lat_stadium, lon_stadium, ...
    zeros(size(lat_stadium)), origin_lat, origin_lon, 0, wgs84);
x_stadium = x_stadium + 1050;
y_stadium = y_stadium + 200;

polygon_xy     = [x_stadium(:), y_stadium(:)];
numInterp      = 50;
boundary_points = [];
for k = 1:size(polygon_xy,1)
    next_idx = mod(k, size(polygon_xy,1)) + 1;
    xi = linspace(polygon_xy(k,1), polygon_xy(next_idx,1), numInterp);
    yi = linspace(polygon_xy(k,2), polygon_xy(next_idx,2), numInterp);
    boundary_points = [boundary_points; xi(:), yi(:)];
end
setOccupancy(map, boundary_points, 1);

% Rivers - City Mill River and Waterworks River
% only drawing the banks, not filling the middle, so the width matters
lat_citymill = [51.54192; 51.54213; 51.54154; 51.54100; 51.54058; ...
                51.54030; 51.53949; 51.53892; 51.53842; 51.53756];
lon_citymill = [-0.01647; -0.01641; -0.01646; -0.01603; -0.01537; ...
                -0.01491; -0.01394; -0.01394; -0.01416; -0.01432];

lat_waterworks = [51.5480; 51.5456; 51.5452; 51.5436; 51.5431; 51.5421; ...
                  51.5418; 51.5412; 51.5405; 51.5395; 51.5383; 51.5376];
lon_waterworks = [-0.0180; -0.0172; -0.0172; -0.0167; -0.0167; -0.0159; ...
                  -0.0149; -0.0140; -0.0128; -0.0115; -0.0107; -0.0103];

rivers = {struct('lat', lat_citymill,  'lon', lon_citymill,  'width', 12), ...
          struct('lat', lat_waterworks, 'lon', lon_waterworks, 'width', 15)};

origin_lat = lat(1);
origin_lon = long(1);

for r = 1:length(rivers)
    [xR, yR, ~] = geodetic2enu(rivers{r}.lat, rivers{r}.lon, 0, ...
                                origin_lat, origin_lon, 0, wgs84);
    xR_shifted = xR + 1050;
    yR_shifted = yR + 200;

    river_pts   = [xR_shifted(:), yR_shifted(:)];
    bank_offset = rivers{r}.width / 2;

    for k = 1:size(river_pts,1)-1
        p1  = river_pts(k,:);
        p2  = river_pts(k+1,:);
        v   = p2 - p1;
        len = norm(v);
        u   = v / len;
        n   = [-u(2), u(1)];   % perpendicular

        num_pts     = ceil(len * 2);
        s           = linspace(0, 1, num_pts)';
        segment_pts = p1 + s * v;

        setOccupancy(map, segment_pts + n * bank_offset, 1);
        setOccupancy(map, segment_pts - n * bank_offset, 1);
    end
end

% Buildings - all converted from GPS and shifted same as the rest of the map

% Marshgate
lat_mg = [51.5381, 51.5379, 51.5372, 51.5374];
lon_mg = [-0.0117, -0.01246, -0.01260, -0.01134];
[x_local, y_local, ~] = geodetic2enu(lat_mg, lon_mg, zeros(size(lat_mg)), ...
    origin_lat, origin_lon, 0, wgs84);
x_local = x_local + 1050; y_local = y_local + 300;
polygon_xy = [x_local(:), y_local(:)];
boundary_points = [];
for k = 1:size(polygon_xy,1)
    next_idx = mod(k, size(polygon_xy,1)) + 1;
    xi = linspace(polygon_xy(k,1), polygon_xy(next_idx,1), numInterp);
    yi = linspace(polygon_xy(k,2), polygon_xy(next_idx,2), numInterp);
    boundary_points = [boundary_points; xi(:), yi(:)];
end
setOccupancy(map, boundary_points, 1);

% One Pool Street
lat_ops = [51.5384, 51.5388, 51.5385, 51.5382, 51.5380];
lon_ops = [-0.0101, -0.0094, -0.0090, -0.0096, -0.0100];
[x_local, y_local, ~] = geodetic2enu(lat_ops, lon_ops, zeros(size(lat_ops)), ...
    origin_lat, origin_lon, 0, wgs84);
x_local = x_local + 1050; y_local = y_local + 300;
polygon_xy = [x_local(:), y_local(:)];
boundary_points = [];
for k = 1:size(polygon_xy,1)
    next_idx = mod(k, size(polygon_xy,1)) + 1;
    xi = linspace(polygon_xy(k,1), polygon_xy(next_idx,1), numInterp);
    yi = linspace(polygon_xy(k,2), polygon_xy(next_idx,2), numInterp);
    boundary_points = [boundary_points; xi(:), yi(:)];
end
setOccupancy(map, boundary_points, 1);

% Aquatics Centre
lat_ops = [51.5406, 51.5407, 51.5404, 51.5397, 51.5396, 51.5399];
lon_ops = [-0.0115, -0.0113, -0.0101, -0.0097, -0.0100, -0.0111];
[x_local, y_local, ~] = geodetic2enu(lat_ops, lon_ops, zeros(size(lat_ops)), ...
    origin_lat, origin_lon, 0, wgs84);
x_local = x_local + 1050; y_local = y_local + 300;
polygon_xy = [x_local(:), y_local(:)];
boundary_points = [];
for k = 1:size(polygon_xy,1)
    next_idx = mod(k, size(polygon_xy,1)) + 1;
    xi = linspace(polygon_xy(k,1), polygon_xy(next_idx,1), numInterp);
    yi = linspace(polygon_xy(k,2), polygon_xy(next_idx,2), numInterp);
    boundary_points = [boundary_points; xi(:), yi(:)];
end
setOccupancy(map, boundary_points, 1);

% Copper Box Arena
lat_cba = [51.5446, 51.5446, 51.5440, 51.5440];
lon_cba = [-0.0194, -0.0207, -0.0208, -0.0195];
[x_local, y_local, ~] = geodetic2enu(lat_cba, lon_cba, zeros(size(lat_cba)), ...
    origin_lat, origin_lon, 0, wgs84);
x_local = x_local + 1050; y_local = y_local + 300;
polygon_xy = [x_local(:), y_local(:)];
boundary_points = [];
for k = 1:size(polygon_xy,1)
    next_idx = mod(k, size(polygon_xy,1)) + 1;
    xi = linspace(polygon_xy(k,1), polygon_xy(next_idx,1), numInterp);
    yi = linspace(polygon_xy(k,2), polygon_xy(next_idx,2), numInterp);
    boundary_points = [boundary_points; xi(:), yi(:)];
end
setOccupancy(map, boundary_points, 1);

% Here East
lat_he = [51.5469, 51.5466, 51.5477, 51.5480];
lon_he = [-0.0232, -0.0238, -0.0251, -0.0245];
[x_local, y_local, ~] = geodetic2enu(lat_he, lon_he, zeros(size(lat_he)), ...
    origin_lat, origin_lon, 0, wgs84);
x_local = x_local + 1050; y_local = y_local + 100;
disp([min(x_local), max(x_local)])
disp([min(y_local), max(y_local)])
polygon_xy = [x_local(:), y_local(:)];
boundary_points = [];
for k = 1:size(polygon_xy,1)
    next_idx = mod(k, size(polygon_xy,1)) + 1;
    xi = linspace(polygon_xy(k,1), polygon_xy(next_idx,1), numInterp);
    yi = linspace(polygon_xy(k,2), polygon_xy(next_idx,2), numInterp);
    boundary_points = [boundary_points; xi(:), yi(:)];
end
setOccupancy(map, boundary_points, 1);

%% Bridges - cut gaps in the Waterworks River so the graph can cross -------
% coordinates read directly off the map axes
bridge_centers = [980, 270;   % lower - between Aquatics and stadium area
                  790, 490];  % upper - above Aquatics Centre

bridge_width  = 60;
bridge_height = 20;

for b = 1:size(bridge_centers,1)
    bx = bridge_centers(b,1);
    by = bridge_centers(b,2);
    for dx = -bridge_width:0.5:bridge_width
        for dy = -bridge_height:0.5:bridge_height
            setOccupancy(map, [bx+dx, by+dy], 0);
        end
    end
    fprintf('Bridge %d at x=%.0f y=%.0f\n', b, bx, by);
end

%% Point cleanup -----------------------------------------------------------
% print any signal points that landed in the bottom-right clump
for i = 1:length(lon4)
    if lon4(i) > 920 && lon4(i) < 1060 && lat4(i) > 100 && lat4(i) < 200
        fprintf('Clump: S%d at (%.0f, %.0f)\n', i, lon4(i), lat4(i));
    end
end

% nudge any point that ended up sitting on an obstacle cell
for i = 1:length(lon4)
    if checkOccupancy(map, [lon4(i), lat4(i)]) == 1
        fprintf('S%d on obstacle at (%.0f, %.0f) - nudging\n', i, lon4(i), lat4(i));
        lon4(i) = lon4(i) + 40;
    end
end
for i = 1:length(lon5)
    if checkOccupancy(map, [lon5(i), lat5(i)]) == 1
        fprintf('K%d on obstacle at (%.0f, %.0f) - nudging\n', i, lon5(i), lat5(i));
        lon5(i) = lon5(i) + 40;
    end
end

% W1 landed on the river bank - shift it clear
lon4(8) = lon4(8) - 80;
lat4(8) = lat4(8) + 20;

% any point sitting inside the top-left building, move it out
for i = 1:length(lon4)
    if lon4(i) > 280 && lon4(i) < 360 && lat4(i) > 920 && lat4(i) < 990
        fprintf('Building clip: S%d at (%.0f, %.0f)\n', i, lon4(i), lat4(i));
        lon4(i) = lon4(i) - 30;
    end
end

%% Plot the base map -------------------------------------------------------
figure
show(map)
hold on;
scatter(lon5, lat5, 80, 'o', 'MarkerEdgeColor', 'r');  % key points (spins)
scatter(lon4, lat4, 80, 'o', 'MarkerEdgeColor', 'g');  % signal points (stops)

%% Graph construction ------------------------------------------------------
% W1 = signal point 1, W2 = signal point 8 (chosen to be on either side
% of the map so the robot has good coverage)
WAITING_IDX    = [1, 8];
CONNECT_RADIUS = 600;   % nodes further than this don't get an edge

numKey    = length(lon5);
key_x     = lon5(:);
key_y     = lat5(:);
key_label = arrayfun(@(i) sprintf('K%d', i), 1:numKey, 'UniformOutput', false)';

numSig   = length(lon4);
sig_x    = lon4(:);
sig_y    = lat4(:);
sig_label = cell(numSig, 1);
sig_type  = repmat({'S'}, numSig, 1);
for i = 1:numSig
    if ismember(i, WAITING_IDX)
        sig_label{i} = sprintf('W%d', find(WAITING_IDX == i));
        sig_type{i}  = 'W';
    else
        sig_label{i} = sprintf('S%d', i);
    end
end

node_x     = [key_x;  sig_x];
node_y     = [key_y;  sig_y];
node_label = [key_label; sig_label];
node_type  = [repmat({'K'}, numKey, 1); sig_type];
N = length(node_x);

fprintf('Nodes: %d key points + %d signal/waiting = %d total\n', numKey, numSig, N);

% build adjacency list - connect nodes within radius if no obstacle blocks
% the straight line between them
adj = cell(N, 1);
for i = 1:N
    adj{i} = struct('to', {}, 'w', {});
end

edge_count = 0;
for i = 1:N
    for j = i+1:N
        d = norm([node_x(i)-node_x(j), node_y(i)-node_y(j)]);
        if d <= CONNECT_RADIUS
            % skip a few samples near each end since nodes sometimes sit
            % close to boundaries and would falsely trigger the check
            t     = linspace(0, 1, 50);
            t_mid = t(4:end-3);
            sample_x = node_x(i) + t_mid * (node_x(j) - node_x(i));
            sample_y = node_y(i) + t_mid * (node_y(j) - node_y(i));
            blocked  = sum(checkOccupancy(map, [sample_x(:), sample_y(:)]) == 1) > 2;

            if ~blocked
                adj{i}(end+1) = struct('to', j, 'w', d);
                adj{j}(end+1) = struct('to', i, 'w', d);
                edge_count = edge_count + 1;
            end
        end
    end
end
fprintf('Edges: %d  (radius = %g m)\n\n', edge_count, CONNECT_RADIUS);

fprintf('--- Adjacency List ---\n');
for i = 1:N
    fprintf('  %s -> ', node_label{i});
    for e = adj{i}
        fprintf('%s(%.0fm)  ', node_label{e.to}, e.w);
    end
    fprintf('\n');
end
fprintf('\n');

%% HOW TO USE --------------------------------------------------------------
% Change query_waiting and query_keypoint then re-run to try different routes
%
%   waiting  = 1 or 2    (W1 or W2)
%   keypoint = 1 to 7    (K1 ... K7)
%
% Result struct fields:
%   .path_labels   e.g. 'W1 -> S3 -> K2'
%   .path_nodes    node indices for plotting
%   .distance      total metres
%   .hops          number of edges
%   .time_ms       runtime
%--------------------------------------------------------------------------

%% Demo: W1 to K1 ----------------------------------------------------------
fprintf('=== Demo: W1 to K1 ===\n');
r_bfs = run_bfs(1, 1);
r_lin = run_dijkstra_linear(1, 1);
r_hp  = run_dijkstra_heap(1, 1);

fprintf('\nBFS\n');
fprintf('  Path     : %s\n', r_bfs.path_labels);
fprintf('  Hops     : %d\n', r_bfs.hops);
fprintf('  Distance : %.1f m\n', r_bfs.distance);
fprintf('  Time     : %.4f ms\n', r_bfs.time_ms);

fprintf('\nDijkstra (linear scan)\n');
fprintf('  Path     : %s\n', r_lin.path_labels);
fprintf('  Distance : %.1f m\n', r_lin.distance);
fprintf('  Time     : %.4f ms\n', r_lin.time_ms);

fprintf('\nDijkstra (heap)\n');
fprintf('  Path     : %s\n', r_hp.path_labels);
fprintf('  Distance : %.1f m\n', r_hp.distance);
fprintf('  Time     : %.4f ms\n', r_hp.time_ms);

%% Benchmark ---------------------------------------------------------------
fprintf('\n--- Benchmark (50 reps, all W->K combos) ---\n');
T_bfs = []; T_lin = []; T_hp = [];
for w = 1:2
    for k = 1:numKey
        t=0; for rep=1:50; t=t+run_bfs(w,k).time_ms;             end; T_bfs(end+1)=t/50;
        t=0; for rep=1:50; t=t+run_dijkstra_linear(w,k).time_ms; end; T_lin(end+1)=t/50;
        t=0; for rep=1:50; t=t+run_dijkstra_heap(w,k).time_ms;   end; T_hp(end+1) =t/50;
    end
end
fprintf('  BFS             mean: %.4f ms\n', mean(T_bfs));
fprintf('  Dijkstra linear mean: %.4f ms\n', mean(T_lin));
fprintf('  Dijkstra heap   mean: %.4f ms\n', mean(T_hp));

%% Custom query - change these and re-run ----------------------------------
query_waiting  = 1;   % W1 or W2
query_keypoint = 3;   % K1 to K7

fprintf('\n=== Custom Query: W%d to K%d ===\n', query_waiting, query_keypoint);
r_custom_bfs = run_bfs(query_waiting, query_keypoint);
r_custom_lin = run_dijkstra_linear(query_waiting, query_keypoint);
r_custom_hp  = run_dijkstra_heap(query_waiting, query_keypoint);

fprintf('BFS:             %s  (%.0fm, %d hops)\n', r_custom_bfs.path_labels, r_custom_bfs.distance, r_custom_bfs.hops);
fprintf('Dijkstra linear: %s  (%.0fm)\n', r_custom_lin.path_labels, r_custom_lin.distance);
fprintf('Dijkstra heap:   %s  (%.0fm)\n', r_custom_hp.path_labels, r_custom_hp.distance);

if exist('map', 'var')
    plot_paths(r_custom_bfs, r_custom_lin, r_custom_hp);
end

% return leg back to waiting point after dropping off the user
fprintf('\n--- Return to W%d ---\n', query_waiting);
r_return = run_dijkstra_heap(query_waiting, query_keypoint);
fprintf('Return: %s  (%.0fm)\n', r_return.path_labels, r_return.distance);
fprintf('Round trip total: %.0fm\n', r_custom_hp.distance + r_return.distance);

%% Plot demo paths and run TSP tour ----------------------------------------
if exist('map', 'var')
    plot_paths(r_bfs, r_lin, r_hp);
end
run_tourist_guide(1);


%% =========================================================================
%  PART II  Velocity, Filtering, Accelerometer Comparison, Pattern Detection
%% =========================================================================

%% i  GPS Velocity (vx, vy, vz) --------------------------------------------
dt_gps = diff(gps_time(:));
dt_gps(dt_gps == 0) = 1e-6;

vx_gps = diff(double(x_shifted(:))) ./ dt_gps;
vy_gps = diff(double(y_shifted(:))) ./ dt_gps;

try
    alt    = double(data.Position.altitude(:));
    alt    = alt - alt(1);
    vz_gps = diff(alt) ./ dt_gps;
catch
    vz_gps = zeros(size(vx_gps));
end

t_vel      = (gps_time(1:end-1) + gps_time(2:end)) / 2;
vtotal_gps = sqrt(vx_gps.^2 + vy_gps.^2 + vz_gps.^2);

fprintf('\n--- GPS Velocity ---\n');
fprintf('  Peak speed : %.2f m/s\n', max(vtotal_gps));
fprintf('  Mean speed : %.2f m/s\n', mean(vtotal_gps));

%% Filtering - centred moving average (same settings on all axes) ----------
% Moving average chosen: zero-phase, no toolbox needed, window = 9 samples
% (~9 s at 1 Hz) removes GPS noise while keeping walking-speed transitions.
filt_win    = 9;
vx_filt     = movmean(vx_gps, filt_win);
vy_filt     = movmean(vy_gps, filt_win);
vz_filt     = movmean(vz_gps, filt_win);
vtotal_filt = sqrt(vx_filt.^2 + vy_filt.^2 + vz_filt.^2);

%% Velocity subplots -------------------------------------------------------
figure('Name', 'GPS Velocity Components (Raw vs Filtered)');
ax1 = subplot(3,1,1);
plot(t_vel, vx_gps, 'Color', [0.75 0.75 0.75]); hold on;
plot(t_vel, vx_filt, 'b', 'LineWidth', 1.8);
ylabel('v_x (m/s)'); title('X Velocity'); grid on;
legend('Raw GPS','Moving avg (win=9)','Location','best');

ax2 = subplot(3,1,2);
plot(t_vel, vy_gps, 'Color', [0.75 0.75 0.75]); hold on;
plot(t_vel, vy_filt, 'r', 'LineWidth', 1.8);
ylabel('v_y (m/s)'); title('Y Velocity'); grid on;
legend('Raw GPS','Moving avg (win=9)','Location','best');

ax3 = subplot(3,1,3);
plot(t_vel, vz_gps, 'Color', [0.75 0.75 0.75]); hold on;
plot(t_vel, vz_filt, 'm', 'LineWidth', 1.8);
ylabel('v_z (m/s)'); xlabel('Time (s)'); title('Z Velocity'); grid on;
legend('Raw GPS','Moving avg (win=9)','Location','best');
linkaxes([ax1 ax2 ax3], 'x');

figure('Name', 'Total Speed (Raw vs Filtered)');
plot(t_vel, vtotal_gps, 'Color', [0.75 0.75 0.75]); hold on;
plot(t_vel, vtotal_filt, 'k', 'LineWidth', 2);
ylabel('Speed (m/s)'); xlabel('Time (s)');
title('Total Speed   v = sqrt(vx^2+vy^2+vz^2)'); grid on;
legend('Raw GPS','Moving avg filtered','Location','best');

%% ii  Accelerometer integration -------------------------------------------
% Gravity removed with a 2-s moving average. cumtrapz gives velocity.
% Drift accumulates from any residual DC offset - mitigate with Kalman/ZUPT.
ax_raw = double(acceleration.X(:));
ay_raw = double(acceleration.Y(:));
az_raw = double(acceleration.Z(:));

win_g  = max(3, 2*round(1/mean(diff(gyro_time))));
ax_dg  = ax_raw - movmean(ax_raw, win_g);
ay_dg  = ay_raw - movmean(ay_raw, win_g);
az_dg  = az_raw - movmean(az_raw, win_g);

vx_acc     = cumtrapz(gyro_time, ax_dg);
vy_acc     = cumtrapz(gyro_time, ay_dg);
vtotal_acc = sqrt(vx_acc.^2 + vy_acc.^2 + cumtrapz(gyro_time, az_dg).^2);

vx_gps_rs     = interp1(t_vel, vx_filt,     gyro_time, 'linear', 'extrap');
vy_gps_rs     = interp1(t_vel, vy_filt,     gyro_time, 'linear', 'extrap');
vtotal_gps_rs = interp1(t_vel, vtotal_filt, gyro_time, 'linear', 'extrap');

figure('Name', 'Accelerometer Integration vs GPS');
subplot(3,1,1);
plot(gyro_time, vx_acc,    'Color',[0.9 0.4 0.1]); hold on;
plot(gyro_time, vx_gps_rs, 'b', 'LineWidth', 1.5);
ylabel('v_x (m/s)'); title('X: Accel integrated vs GPS'); grid on;
legend('Accel','GPS filtered');

subplot(3,1,2);
plot(gyro_time, vy_acc,    'Color',[0.9 0.4 0.1]); hold on;
plot(gyro_time, vy_gps_rs, 'r', 'LineWidth', 1.5);
ylabel('v_y (m/s)'); title('Y: Accel integrated vs GPS'); grid on;
legend('Accel','GPS filtered');

subplot(3,1,3);
plot(gyro_time, vtotal_acc,    'Color',[0.9 0.4 0.1]); hold on;
plot(gyro_time, vtotal_gps_rs, 'k', 'LineWidth', 1.5);
ylabel('Speed (m/s)'); xlabel('Time (s)');
title('Total Speed: Accel integrated vs GPS'); grid on;
legend('Accel','GPS filtered');

fprintf('\n--- Accel vs GPS ---\n');
fprintf('  GPS  peak : %.2f m/s\n', max(vtotal_gps_rs));
fprintf('  Accel peak: %.2f m/s  (drift causes divergence over time)\n', max(vtotal_acc));
fprintf('  Mitigation: Kalman filter fusion, zero-velocity updates at stops.\n');

%% iii  Pattern detection ---------------------------------------------------
% Threshold-based, sample-by-sample classification.
% Yaw rate from gyro Z resampled onto GPS timebase.
THR_STOP = 0.3;   % m/s
THR_SLOW = 0.8;   % m/s
THR_TURN = 0.4;   % rad/s

gyro_z_rs = interp1(gyro_time, abs(double(angularVelocity.Z(:))), ...
                    t_vel, 'linear', 'extrap');

n_samp   = length(vtotal_filt);
patterns = cell(n_samp, 1);
for ii = 1:n_samp
    spd = vtotal_filt(ii);  w = gyro_z_rs(ii);
    if     spd < THR_STOP;   patterns{ii} = 'Stop';
    elseif w   > THR_TURN;   patterns{ii} = 'Turning';
    elseif spd < THR_SLOW;   patterns{ii} = 'Slow walk';
    else;                    patterns{ii} = 'Fast';
    end
end

pat_types = {'Stop','Slow walk','Turning','Fast'};
fprintf('\n--- Pattern Detection ---\n');
for pi = 1:numel(pat_types)
    cnt = sum(strcmp(patterns, pat_types{pi}));
    fprintf('  %-12s: %3d samples (%.1f%%)\n', pat_types{pi}, cnt, 100*cnt/n_samp);
end

px_vel = (double(x_shifted(1:end-1)) + double(x_shifted(2:end))) / 2;
py_vel = (double(y_shifted(1:end-1)) + double(y_shifted(2:end))) / 2;

%% iv  Pattern map ----------------------------------------------------------
pat_clrs = {[0.9 0.1 0.1],[0.2 0.7 0.2],[1.0 0.6 0.0],[0.1 0.4 0.9]};

figure('Name', 'Motion Patterns on Occupancy Map');
show(map); hold on;

h_pat = gobjects(1, 4);
for pi = 1:4
    mask = strcmp(patterns, pat_types{pi});
    if any(mask)
        h_pat(pi) = scatter(px_vel(mask), py_vel(mask), 18, 'filled', ...
            'MarkerFaceColor', pat_clrs{pi}, 'MarkerEdgeColor', 'none', ...
            'MarkerFaceAlpha', 0.7);
    else
        h_pat(pi) = scatter(nan, nan, 18, 'filled', ...
            'MarkerFaceColor', pat_clrs{pi}, 'MarkerEdgeColor', 'none');
    end
end

h_key = scatter(lon5, lat5, 60, 'o', 'MarkerEdgeColor', 'r',       'LineWidth', 1.5);
h_sig = scatter(lon4, lat4, 60, 'o', 'MarkerEdgeColor', [0 0.6 0], 'LineWidth', 1.5);

legend([h_pat(1) h_pat(2) h_pat(3) h_pat(4) h_key h_sig], ...
    {'Stop','Slow walk','Turning','Fast','Key pts (K)','Signal pts (S)'}, ...
    'Location','northeast','FontSize',8);
title('Motion Patterns on Occupancy Map');

fprintf('\nMap: red=Stop  green=Slow walk  orange=Turning  blue=Fast\n');
fprintf('Stops/turns cluster near landmarks, validating point placement.\n');


%% =========================================================================
%  FUNCTIONS
%% =========================================================================

function result = run_bfs(waiting, keypoint)
% BFS - finds path with fewest hops, doesn't care about edge weights.
% Fine when all edges are roughly similar length, O(V+E).
    [adj, node_x, node_y, node_label, src, dst] = load_graph(waiting, keypoint);
    N = length(adj);

    t0           = tic;
    visited      = false(1, N);
    parent       = zeros(1, N);
    queue        = src;
    visited(src) = true;
    parent(src)  = -1;
    found        = false;

    while ~isempty(queue)
        u = queue(1); queue(1) = [];
        if u == dst; found = true; break; end
        for e = adj{u}
            if ~visited(e.to)
                visited(e.to) = true;
                parent(e.to)  = u;
                queue(end+1)  = e.to;
            end
        end
    end
    elapsed = toc(t0) * 1000;

    if ~found
        result = pack_result([], 0, 0, elapsed, node_label, node_x, node_y);
        return;
    end

    path = dst;
    while parent(path(1)) ~= -1
        path = [parent(path(1)), path];
    end
    result = pack_result(path, path_dist(path, node_x, node_y), length(path)-1, elapsed, node_label, node_x, node_y);
end


function result = run_dijkstra_linear(waiting, keypoint)
% Dijkstra with a linear scan to find the next cheapest node.
% Correct shortest path by distance but slow on large graphs: O(V^2).
    [adj, node_x, node_y, node_label, src, dst] = load_graph(waiting, keypoint);
    N = length(adj);

    t0   = tic;
    dist = inf(1, N);
    prev = zeros(1, N);
    done = false(1, N);
    dist(src) = 0;
    prev(src) = -1;

    for iter = 1:N
        u = -1; best = inf;
        for i = 1:N
            if ~done(i) && dist(i) < best
                best = dist(i); u = i;
            end
        end
        if u == -1 || isinf(dist(u)); break; end
        done(u) = true;
        if u == dst; break; end

        for e = adj{u}
            alt = dist(u) + e.w;
            if alt < dist(e.to)
                dist(e.to) = alt;
                prev(e.to) = u;
            end
        end
    end
    elapsed = toc(t0) * 1000;

    if isinf(dist(dst))
        result = pack_result([], 0, 0, elapsed, node_label, node_x, node_y);
        return;
    end

    path = dst;
    while prev(path(1)) ~= -1
        path = [prev(path(1)), path];
    end
    result = pack_result(path, dist(dst), length(path)-1, elapsed, node_label, node_x, node_y);
end


function result = run_dijkstra_heap(waiting, keypoint)
% Dijkstra using a min-heap (Nx2 matrix sorted by distance column).
% Same result as linear version but faster: O((V+E) log V).
    [adj, node_x, node_y, node_label, src, dst] = load_graph(waiting, keypoint);
    N = length(adj);

    t0   = tic;
    dist = inf(1, N);
    prev = zeros(1, N);
    dist(src) = 0;
    prev(src) = -1;
    heap = [src, 0];

    while ~isempty(heap)
        [~, idx] = min(heap(:,2));
        u = heap(idx,1);
        heap(idx,:) = [];
        if u == dst; break; end

        for e = adj{u}
            alt = dist(u) + e.w;
            if alt < dist(e.to)
                dist(e.to) = alt;
                prev(e.to) = u;
                row = find(heap(:,1) == e.to, 1);
                if isempty(row)
                    heap(end+1,:) = [e.to, alt];
                else
                    heap(row,2) = alt;
                end
            end
        end
    end
    elapsed = toc(t0) * 1000;

    if isinf(dist(dst))
        result = pack_result([], 0, 0, elapsed, node_label, node_x, node_y);
        return;
    end

    path = dst;
    while prev(path(1)) ~= -1
        path = [prev(path(1)), path];
    end
    result = pack_result(path, dist(dst), length(path)-1, elapsed, node_label, node_x, node_y);
end


function run_tourist_guide(waiting)
% Greedy nearest-neighbour TSP. At each step picks the closest unvisited
% key point. Not globally optimal but simple and good enough for a demo.
    adj         = evalin('base', 'adj');
    node_x      = evalin('base', 'node_x');
    node_y      = evalin('base', 'node_y');
    node_label  = evalin('base', 'node_label');
    numKey      = evalin('base', 'numKey');
    WAITING_IDX = evalin('base', 'WAITING_IDX');

    fprintf('\n=== Tourist-Guide Mode (W%d) ===\n', waiting);

    start_node   = numKey + WAITING_IDX(waiting);
    current_node = start_node;
    unvisited    = 1:numKey;
    total_dist   = 0;
    tour_labels  = {node_label{start_node}};

    while ~isempty(unvisited)
        best_dist = inf; best_node = -1;

        for k = unvisited
            [dv, pk] = dijkstra_heap_from_node(adj, current_node, k, node_x, node_y);
            if dv(k) < best_dist && ~isempty(pk)
                best_dist = dv(k); best_node = k;
            end
        end

        if best_node == -1
            fprintf('  Warning: could not reach all key points\n');
            break;
        end

        total_dist  = total_dist + best_dist;
        tour_labels{end+1} = node_label{best_node};
        fprintf('  -> %s  (%.0f m)\n', node_label{best_node}, best_dist);
        unvisited(unvisited == best_node) = [];
        current_node = best_node;
    end

    [d_ret, p_ret] = dijkstra_heap_from_node(adj, current_node, start_node, node_x, node_y);
    if ~isempty(p_ret)
        total_dist = total_dist + d_ret(start_node);
        tour_labels{end+1} = node_label{start_node};
        fprintf('  -> %s  (return, %.0f m)\n', node_label{start_node}, d_ret(start_node));
    end

    fprintf('Total: %.0f m\n', total_dist);
    fprintf('Route: %s\n', strjoin(tour_labels, ' -> '));
end


function plot_paths(r_bfs, r_lin, r_hp)
% Draws edges, colour-coded nodes, and the three algorithm paths on the map.
    adj        = evalin('base', 'adj');
    node_x     = evalin('base', 'node_x');
    node_y     = evalin('base', 'node_y');
    node_label = evalin('base', 'node_label');
    node_type  = evalin('base', 'node_type');
    map        = evalin('base', 'map');
    N = length(adj);

    figure('Name', 'Path Planning on Occupancy Map');
    show(map); hold on;

    for i = 1:N
        for e = adj{i}
            if e.to > i
                plot([node_x(i), node_x(e.to)], [node_y(i), node_y(e.to)], ...
                     '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.7);
            end
        end
    end

    colours = struct('K','b', 'S','g', 'W','m');
    for i = 1:N
        plot(node_x(i), node_y(i), 'o', ...
             'MarkerFaceColor', colours.(node_type{i}), ...
             'MarkerEdgeColor', 'k', 'MarkerSize', 8);
        text(node_x(i)+5, node_y(i)+5, node_label{i}, 'FontSize', 7);
    end

    if ~isempty(r_bfs.path_nodes)
        plot(node_x(r_bfs.path_nodes), node_y(r_bfs.path_nodes), '--', ...
             'Color', [0.9 0.4 0], 'LineWidth', 2.5);
    end
    if ~isempty(r_lin.path_nodes)
        plot(node_x(r_lin.path_nodes), node_y(r_lin.path_nodes), '-', ...
             'Color', [0 0.6 0.9], 'LineWidth', 2);
    end
    if ~isempty(r_hp.path_nodes)
        plot(node_x(r_hp.path_nodes), node_y(r_hp.path_nodes), ':', ...
             'Color', [0.8 0 0.8], 'LineWidth', 2);
    end

    legend({'Edges','Key (K)','Signal (S)','Waiting (W)', ...
            'BFS','Dijkstra linear','Dijkstra heap'}, ...
           'Location', 'northeast', 'FontSize', 8);
    hold off;
end


% --- helpers --------------------------------------------------------------

function [adj, node_x, node_y, node_label, src, dst] = load_graph(waiting, keypoint)
% pulls everything needed from the base workspace so callers stay clean
    adj         = evalin('base', 'adj');
    node_x      = evalin('base', 'node_x');
    node_y      = evalin('base', 'node_y');
    node_label  = evalin('base', 'node_label');
    numKey      = evalin('base', 'numKey');
    WAITING_IDX = evalin('base', 'WAITING_IDX');
    src = numKey + WAITING_IDX(waiting);
    dst = keypoint;
end


function result = pack_result(path, distance, hops, time_ms, node_label, node_x, node_y)
% packages algorithm output into a consistent struct
    result.path_nodes = path;
    result.distance   = distance;
    result.hops       = hops;
    result.time_ms    = time_ms;
    if isempty(path)
        result.path_labels = 'No path found';
    else
        result.path_labels = strjoin(node_label(path), ' -> ');
    end
end


function d = path_dist(path, node_x, node_y)
% sums Euclidean distances along a sequence of nodes
    d = 0;
    for i = 1:length(path)-1
        d = d + norm([node_x(path(i))-node_x(path(i+1)), ...
                      node_y(path(i))-node_y(path(i+1))]);
    end
end


function [dist, path] = dijkstra_heap_from_node(adj, src, dst, node_x, node_y)
% same as run_dijkstra_heap but accepts a raw node index as source
% needed by tourist guide so it can start from any node mid-tour
    N    = length(adj);
    dist = inf(1, N);
    prev = zeros(1, N);
    dist(src) = 0;
    prev(src) = -1;
    heap = [src, 0];

    while ~isempty(heap)
        [~, idx] = min(heap(:,2));
        u = heap(idx,1);
        heap(idx,:) = [];
        if u == dst; break; end

        for e = adj{u}
            alt = dist(u) + e.w;
            if alt < dist(e.to)
                dist(e.to) = alt;
                prev(e.to) = u;
                row = find(heap(:,1) == e.to, 1);
                if isempty(row)
                    heap(end+1,:) = [e.to, alt];
                else
                    heap(row,2) = alt;
                end
            end
        end
    end

    if isinf(dist(dst))
        path = []; return;
    end

    path = dst;
    while prev(path(1)) ~= -1
        path = [prev(path(1)), path];
    end
end