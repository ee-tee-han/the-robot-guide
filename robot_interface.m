% robot_interface.m
%
% Task 3 - Human-Robot Interaction: Lookup for places, request handling
% -----------------------------------------------------------------------
% CLI interface for robot navigation queries over Queen Elizabeth Olympic
% Park. Answers 10 customisable questions using two lookup structures:
%
%   Linked List  (Node.m + SinglyLinkedList.m)  --  O(N) linear scan
%   KD-Tree      (MATLAB KDTreeSearcher)         --  O(log N)
%
% Graph pathfinding is done with BFS and Dijkstra (min-heap).
%
% HOW TO RUN
%   1. Make sure park.mat and all .m files are in the same folder.
%   2. Run this file directly in MATLAB.
%      It calls kaya_code.m automatically to build the map and graph.
%   3. Pick a query number (1-10), supply the requested arguments,
%      and the robot will answer with path / distance / timing info.
% -----------------------------------------------------------------------

clc;

%% ---- Build map and graph (only if not already in workspace) ------------
if ~exist('adj', 'var') || ~exist('node_x', 'var')
    fprintf('Building map and graph from park.mat - please wait...\n\n');
    run('kaya_code.m');
    fprintf('\n>>> Map ready. Entering navigation interface.\n');
end

%% ---- Build lookup structures from detected points ----------------------

% Linked list: key points only (type = 'key')
ll_keys = SinglyLinkedList();
for i = 1:numKey
    ll_keys.addLast(struct('name', key_label{i}, ...
        'x', key_x(i), 'y', key_y(i), 'type', 'key'));
end

% Linked list: all nodes - keys, signals, and waiting (type = K / S / W)
ll_all = SinglyLinkedList();
for i = 1:N
    ll_all.addLast(struct('name', node_label{i}, ...
        'x', node_x(i), 'y', node_y(i), 'type', node_type{i}));
end

% KD-Tree: key points only
kd_keys = KDTreeSearcher([key_x, key_y]);

% KD-Tree: all nodes
kd_all = KDTreeSearcher([node_x, node_y]);

fprintf('\nLookup structures built:\n');
fprintf('  Linked List (keys) : %d nodes\n', numKey);
fprintf('  Linked List (all)  : %d nodes\n', N);
fprintf('  KD-Tree     (keys) : %d nodes\n', numKey);
fprintf('  KD-Tree     (all)  : %d nodes\n\n', N);

%% ---- Main menu loop ----------------------------------------------------
running = true;
while running
    print_menu(numKey, key_label, key_x, key_y, sig_label, sig_x, sig_y, ...
        node_label, node_x, node_y, WAITING_IDX, numSig);
    choice = input('Enter choice (0-11): ');
    if isempty(choice); continue; end
    fprintf('\n');

    switch choice

        case 0
            fprintf('Robot shutting down. Goodbye!\n');
            running = false;

        %------------------------------------------------------------------
        % Q1  Route from waiting area W to key point K
        %------------------------------------------------------------------
        case 1
            w = prompt_int('  Waiting area number (1 or 2): ', 1, 2);
            k = prompt_int(sprintf('  Key point number  (1 to %d): ', numKey), 1, numKey);
            q1_route(w, k, adj, node_x, node_y, node_label, numKey, WAITING_IDX);

        %------------------------------------------------------------------
        % Q2  N closest key points to an arbitrary position (x, y)
        %------------------------------------------------------------------
        case 2
            x = input('  Position x: ');
            y = input('  Position y: ');
            n = prompt_int(sprintf('  How many closest points N (1-%d): ', numKey), 1, numKey);
            q2_n_closest_keys(x, y, n, ll_keys, kd_keys, key_x, key_y, key_label);

        %------------------------------------------------------------------
        % Q3  Graph distance from waiting area W to key point K
        %------------------------------------------------------------------
        case 3
            w = prompt_int('  Waiting area number (1 or 2): ', 1, 2);
            k = prompt_int(sprintf('  Key point number  (1 to %d): ', numKey), 1, numKey);
            q3_distance(w, k, adj, node_x, node_y, node_label, numKey, WAITING_IDX);

        %------------------------------------------------------------------
        % Q4  N closest key points to a waiting area
        %------------------------------------------------------------------
        case 4
            w = prompt_int('  Waiting area number (1 or 2): ', 1, 2);
            n = prompt_int(sprintf('  How many closest points N (1-%d): ', numKey), 1, numKey);
            q4_closest_to_waiting(w, n, ll_keys, kd_keys, key_x, key_y, ...
                key_label, node_x, node_y, numKey, WAITING_IDX);

        %------------------------------------------------------------------
        % Q5  Tourist tour: visit all key points from waiting area W
        %------------------------------------------------------------------
        case 5
            w = prompt_int('  Start from waiting area (1 or 2): ', 1, 2);
            q5_tour(w, adj, node_x, node_y, node_label, numKey, WAITING_IDX);

        %------------------------------------------------------------------
        % Q6  All key points reachable within D metres of (x, y)
        %------------------------------------------------------------------
        case 6
            x = input('  Position x: ');
            y = input('  Position y: ');
            d = input('  Max straight-line distance (metres): ');
            q6_reachable(x, y, d, ll_keys, kd_keys, key_x, key_y, key_label);

        %------------------------------------------------------------------
        % Q7  Shortest path between two key points K1 and K2
        %------------------------------------------------------------------
        case 7
            k1 = prompt_int(sprintf('  First key point  (1 to %d): ', numKey), 1, numKey);
            k2 = prompt_int(sprintf('  Second key point (1 to %d): ', numKey), 1, numKey);
            q7_key_to_key(k1, k2, adj, node_x, node_y, node_label);

        %------------------------------------------------------------------
        % Q8  All key points ranked by distance from (x, y)
        %------------------------------------------------------------------
        case 8
            x = input('  Position x: ');
            y = input('  Position y: ');
            q8_all_sorted(x, y, ll_keys, kd_keys, key_x, key_y, key_label, numKey);

        %------------------------------------------------------------------
        % Q9  Minimum hop count from waiting area W to key point K
        %------------------------------------------------------------------
        case 9
            w = prompt_int('  Waiting area number (1 or 2): ', 1, 2);
            k = prompt_int(sprintf('  Key point number  (1 to %d): ', numKey), 1, numKey);
            q9_hops(w, k, adj, node_x, node_y, node_label, numKey, WAITING_IDX);

        %------------------------------------------------------------------
        % Q10  Closest signal / waiting point to (x, y)
        %------------------------------------------------------------------
        case 10
            x = input('  Position x: ');
            y = input('  Position y: ');
            q10_closest_signal(x, y, ll_all, kd_all, ...
                node_x, node_y, node_label, node_type, numKey);

        %------------------------------------------------------------------
        % Q11  Full distance / time report for ALL point combinations
        %------------------------------------------------------------------
        case 11
            q11_full_report(adj, node_x, node_y, node_label, ...
                key_x, key_y, key_label, sig_x, sig_y, sig_label, ...
                numKey, numSig, WAITING_IDX, kd_keys);

        otherwise
            fprintf('[!] Invalid choice. Please enter a number between 0 and 11.\n');
    end
end


%% =========================================================================
%  QUERY FUNCTIONS  (Q1 - Q10)
%% =========================================================================

% -------------------------------------------------------------------------
function q1_route(w, k, adj, node_x, node_y, node_label, numKey, WAITING_IDX)
% Q1: How do I get from waiting area W to key point K?
%     Runs both BFS (min hops) and Dijkstra heap (min distance) and prints
%     each path with its distance and runtime.
    src = numKey + WAITING_IDX(w);
    dst = k;
    sep = repmat('-', 1, 60);
    fprintf('[Q1] Route from %s to %s\n%s\n', node_label{src}, node_label{dst}, sep);

    % BFS
    t0 = tic;
    [path_b, hops_b] = ri_bfs(adj, src, dst, length(adj));
    t_b = toc(t0) * 1000;

    % Dijkstra heap
    t0 = tic;
    [path_d, dist_d] = ri_dijkstra(adj, src, dst, length(adj));
    t_d = toc(t0) * 1000;

    fprintf('BFS  (min hops):\n');
    if isempty(path_b)
        fprintf('  No path found.\n');
    else
        fprintf('  Path     : %s\n', strjoin(node_label(path_b), ' -> '));
        fprintf('  Hops     : %d\n', hops_b);
        fprintf('  Distance : %.1f m\n', ri_path_len(path_b, node_x, node_y));
        fprintf('  Time     : %.4f ms  |  O(V + E)\n', t_b);
    end

    fprintf('Dijkstra (min distance):\n');
    if isempty(path_d)
        fprintf('  No path found.\n');
    else
        fprintf('  Path     : %s\n', strjoin(node_label(path_d), ' -> '));
        fprintf('  Hops     : %d\n', length(path_d) - 1);
        fprintf('  Distance : %.1f m\n', dist_d);
        fprintf('  Time     : %.4f ms  |  O((V + E) log V)\n', t_d);
    end
    perf_summary(t_b, t_d, 'BFS', 'Dijkstra');
end


% -------------------------------------------------------------------------
function q2_n_closest_keys(x, y, n, ll_keys, kd_keys, key_x, key_y, key_label)
% Q2: What are the N closest key points to position (x, y)?
%     Compares Linked List linear scan vs KD-Tree search.
    sep = repmat('-', 1, 60);
    fprintf('[Q2] %d closest key point(s) to position (%.1f, %.1f)\n%s\n', n, x, y, sep);

    % Linked list linear scan  O(K)
    t0 = tic;
    ll_res = ll_n_closest(ll_keys, x, y, n);
    t_ll = toc(t0) * 1000;

    % KD-Tree  O(log K + n)
    t0 = tic;
    [idx_kd, dist_kd] = knnsearch(kd_keys, [x, y], 'K', n);
    t_kd = toc(t0) * 1000;

    fprintf('Linked List  [O(K) linear scan]:\n');
    for i = 1:length(ll_res)
        fprintf('  %d. %-25s  %.1f m\n', i, ll_res(i).name, ll_res(i).dist);
    end
    fprintf('  Time: %.4f ms\n\n', t_ll);

    fprintf('KD-Tree      [O(log K)]:\n');
    for i = 1:length(idx_kd)
        fprintf('  %d. %-25s  %.1f m\n', i, key_label{idx_kd(i)}, dist_kd(i));
    end
    fprintf('  Time: %.4f ms\n', t_kd);
    perf_summary(t_ll, t_kd, 'Linked List', 'KD-Tree');
end


% -------------------------------------------------------------------------
function q3_distance(w, k, adj, node_x, node_y, node_label, numKey, WAITING_IDX)
% Q3: What is the graph distance from waiting area W to key point K?
%     Reports both the graph (navigable) distance and the straight-line
%     distance, plus the detour factor.
    src = numKey + WAITING_IDX(w);
    dst = k;
    sep = repmat('-', 1, 60);
    fprintf('[Q3] Distance from %s to %s\n%s\n', node_label{src}, node_label{dst}, sep);

    [path, dist] = ri_dijkstra(adj, src, dst, length(adj));

    if isempty(path)
        fprintf('  No navigable path found.\n');
    else
        straight = norm([node_x(src) - node_x(dst), node_y(src) - node_y(dst)]);
        fprintf('  Path          : %s\n', strjoin(node_label(path), ' -> '));
        fprintf('  Graph dist    : %.1f m  (navigable route)\n', dist);
        fprintf('  Straight-line : %.1f m\n', straight);
        fprintf('  Detour factor : %.2fx\n', dist / max(straight, 0.01));
    end
end


% -------------------------------------------------------------------------
function q4_closest_to_waiting(w, n, ll_keys, kd_keys, key_x, key_y, ...
                                key_label, node_x, node_y, numKey, WAITING_IDX)
% Q4: What are the N closest key points to waiting area W?
    src = numKey + WAITING_IDX(w);
    wx  = node_x(src);
    wy  = node_y(src);
    sep = repmat('-', 1, 60);
    fprintf('[Q4] %d closest key point(s) to W%d at (%.1f, %.1f)\n%s\n', ...
        n, w, wx, wy, sep);

    % Linked list  O(K)
    t0 = tic;
    ll_res = ll_n_closest(ll_keys, wx, wy, n);
    t_ll = toc(t0) * 1000;

    % KD-Tree  O(log K)
    t0 = tic;
    [idx_kd, dist_kd] = knnsearch(kd_keys, [wx, wy], 'K', n);
    t_kd = toc(t0) * 1000;

    fprintf('Linked List  [O(K) linear scan]:\n');
    for i = 1:length(ll_res)
        fprintf('  %d. %-25s  %.1f m (straight-line)\n', i, ll_res(i).name, ll_res(i).dist);
    end
    fprintf('  Time: %.4f ms\n\n', t_ll);

    fprintf('KD-Tree      [O(log K)]:\n');
    for i = 1:length(idx_kd)
        fprintf('  %d. %-25s  %.1f m (straight-line)\n', i, key_label{idx_kd(i)}, dist_kd(i));
    end
    fprintf('  Time: %.4f ms\n', t_kd);
    perf_summary(t_ll, t_kd, 'Linked List', 'KD-Tree');
end


% -------------------------------------------------------------------------
function q5_tour(w, adj, node_x, node_y, node_label, numKey, WAITING_IDX)
% Q5: Greedy nearest-neighbour tour from waiting area W visiting all keys.
%     At each step Dijkstra finds the closest unvisited key point.
    src = numKey + WAITING_IDX(w);
    sep = repmat('-', 1, 60);
    fprintf('[Q5] Tourist tour from W%d through all %d key points\n%s\n', ...
        w, numKey, sep);

    t0        = tic;
    current   = src;
    unvisited = 1:numKey;
    total     = 0;
    tour      = {node_label{src}};

    while ~isempty(unvisited)
        best_d = inf;  best_k = -1;
        for k = unvisited
            [~, dv] = ri_dijkstra(adj, current, k, length(adj));
            if dv < best_d
                best_d = dv;  best_k = k;
            end
        end
        if best_k == -1
            fprintf('  [!] Could not reach remaining key points.\n');
            break;
        end
        total = total + best_d;
        tour{end+1} = node_label{best_k};
        fprintf('  -> %-25s  %.1f m\n', node_label{best_k}, best_d);
        unvisited(unvisited == best_k) = [];
        current = best_k;
    end

    % return leg back to waiting point
    [~, d_ret] = ri_dijkstra(adj, current, src, length(adj));
    if ~isinf(d_ret)
        total = total + d_ret;
        tour{end+1} = node_label{src};
        fprintf('  -> %-25s  %.1f m  (return)\n', node_label{src}, d_ret);
    end

    t_tour = toc(t0) * 1000;
    fprintf('\n  Full route : %s\n', strjoin(tour, ' -> '));
    fprintf('  Total dist : %.1f m\n', total);
    fprintf('  Algo time  : %.2f ms  |  O(K * (V+E) log V) greedy TSP\n', t_tour);
end


% -------------------------------------------------------------------------
function q6_reachable(x, y, max_dist, ll_keys, kd_keys, key_x, key_y, key_label)
% Q6: Which key points lie within max_dist metres (straight-line) of (x,y)?
%     Uses Linked List full scan and KD-Tree range search.
    sep = repmat('-', 1, 60);
    fprintf('[Q6] Key points within %.1f m of (%.1f, %.1f)\n%s\n', max_dist, x, y, sep);

    % Linked list range scan  O(K)
    t0 = tic;
    ll_res = ll_range(ll_keys, x, y, max_dist);
    t_ll = toc(t0) * 1000;

    % KD-Tree range search  O(log K + R)
    t0 = tic;
    [idx_kd, dist_kd] = rangesearch(kd_keys, [x, y], max_dist);
    idx_kd = idx_kd{1};  dist_kd = dist_kd{1};
    t_kd = toc(t0) * 1000;

    fprintf('Linked List  [O(K), %d found]:\n', length(ll_res));
    if isempty(ll_res)
        fprintf('  (none)\n');
    else
        for i = 1:length(ll_res)
            fprintf('  %-25s  %.1f m\n', ll_res(i).name, ll_res(i).dist);
        end
    end
    fprintf('  Time: %.4f ms\n\n', t_ll);

    fprintf('KD-Tree      [O(log K + R), %d found]:\n', length(idx_kd));
    if isempty(idx_kd)
        fprintf('  (none)\n');
    else
        for i = 1:length(idx_kd)
            fprintf('  %-25s  %.1f m\n', key_label{idx_kd(i)}, dist_kd(i));
        end
    end
    fprintf('  Time: %.4f ms\n', t_kd);
    perf_summary(t_ll, t_kd, 'Linked List', 'KD-Tree');
end


% -------------------------------------------------------------------------
function q7_key_to_key(k1, k2, adj, node_x, node_y, node_label)
% Q7: What is the shortest path directly between two key points K1 and K2?
%     Runs BFS and Dijkstra for a direct comparison.
    sep = repmat('-', 1, 60);
    fprintf('[Q7] Shortest path between %s and %s\n%s\n', ...
        node_label{k1}, node_label{k2}, sep);

    t0 = tic;
    [path_b, hops_b] = ri_bfs(adj, k1, k2, length(adj));
    t_b = toc(t0) * 1000;

    t0 = tic;
    [path_d, dist_d] = ri_dijkstra(adj, k1, k2, length(adj));
    t_d = toc(t0) * 1000;

    fprintf('BFS  (min hops):\n');
    if isempty(path_b)
        fprintf('  No path found.\n');
    else
        fprintf('  Path     : %s\n', strjoin(node_label(path_b), ' -> '));
        fprintf('  Hops     : %d  |  Distance : %.1f m  |  Time : %.4f ms\n', ...
            hops_b, ri_path_len(path_b, node_x, node_y), t_b);
    end

    fprintf('Dijkstra (min distance):\n');
    if isempty(path_d)
        fprintf('  No path found.\n');
    else
        fprintf('  Path     : %s\n', strjoin(node_label(path_d), ' -> '));
        fprintf('  Hops     : %d  |  Distance : %.1f m  |  Time : %.4f ms\n', ...
            length(path_d)-1, dist_d, t_d);
    end
    perf_summary(t_b, t_d, 'BFS', 'Dijkstra');
end


% -------------------------------------------------------------------------
function q8_all_sorted(x, y, ll_keys, kd_keys, key_x, key_y, key_label, numKey)
% Q8: List ALL key points ranked from closest to furthest from (x, y).
%     Shows the full ranked table from both methods side-by-side.
    sep = repmat('-', 1, 60);
    fprintf('[Q8] All %d key points ranked from (%.1f, %.1f)\n%s\n', ...
        numKey, x, y, sep);

    % Linked list  O(K log K) - scan + sort
    t0 = tic;
    ll_res = ll_n_closest(ll_keys, x, y, numKey);
    t_ll = toc(t0) * 1000;

    % KD-Tree  O(K log K) - returns sorted by default
    t0 = tic;
    [idx_kd, dist_kd] = knnsearch(kd_keys, [x, y], 'K', numKey);
    t_kd = toc(t0) * 1000;

    fprintf('Rank  %-25s %10s    %-25s %10s\n', ...
        'Linked List', 'Dist(m)', 'KD-Tree', 'Dist(m)');
    fprintf('%s\n', repmat('-', 1, 78));
    for i = 1:numKey
        ll_name = ll_res(i).name;
        kd_name = key_label{idx_kd(i)};
        flag = '';
        if ~strcmp(ll_name, kd_name); flag = ' <<'; end
        fprintf('  %2d.  %-25s %8.1f    %-25s %8.1f%s\n', ...
            i, ll_name, ll_res(i).dist, kd_name, dist_kd(i), flag);
    end
    fprintf('\n  Linked List time : %.4f ms  |  KD-Tree time : %.4f ms\n', t_ll, t_kd);
    perf_summary(t_ll, t_kd, 'Linked List', 'KD-Tree');
end


% -------------------------------------------------------------------------
function q9_hops(w, k, adj, node_x, node_y, node_label, numKey, WAITING_IDX)
% Q9: How many intermediate stops (hops) from waiting area W to key K?
%     BFS gives the minimum-hop path; Dijkstra gives the shortest-distance.
    src = numKey + WAITING_IDX(w);
    dst = k;
    sep = repmat('-', 1, 60);
    fprintf('[Q9] Hop count from %s to %s\n%s\n', node_label{src}, node_label{dst}, sep);

    [path_b, hops_b] = ri_bfs(adj, src, dst, length(adj));
    [path_d, dist_d] = ri_dijkstra(adj, src, dst, length(adj));

    if isempty(path_b)
        fprintf('  No path found.\n');
    else
        d_bfs = ri_path_len(path_b, node_x, node_y);
        fprintf('  BFS path     : %s\n', strjoin(node_label(path_b), ' -> '));
        fprintf('  Min hops     : %d  |  Distance along BFS path : %.1f m\n', hops_b, d_bfs);
        fprintf('  Dijkstra path: %s\n', strjoin(node_label(path_d), ' -> '));
        fprintf('  Dijkstra hops: %d  |  Shortest distance : %.1f m\n', length(path_d)-1, dist_d);
        if hops_b < length(path_d)-1
            fprintf('  BFS saves %d hop(s) but Dijkstra saves %.1f m.\n', ...
                (length(path_d)-1) - hops_b, d_bfs - dist_d);
        elseif hops_b == length(path_d)-1
            fprintf('  Both paths have equal hop counts.\n');
        end
    end
end


% -------------------------------------------------------------------------
function q10_closest_signal(x, y, ll_all, kd_all, ...
                             node_x, node_y, node_label, node_type, numKey)
% Q10: What is the nearest signal (S) or waiting (W) point to (x, y)?
%      Linked List scans all nodes and filters by type.
%      KD-Tree retrieves sorted neighbours then picks the first S/W type.
    sep = repmat('-', 1, 60);
    fprintf('[Q10] Closest signal / waiting point to (%.1f, %.1f)\n%s\n', x, y, sep);

    % Linked list type-filtered scan  O(N)
    t0 = tic;
    ll_res = ll_closest_typed(ll_all, x, y, {'S', 'W'});
    t_ll = toc(t0) * 1000;

    % KD-Tree: get all nodes sorted by dist, iterate until S or W found
    t0 = tic;
    [idx_kd, dist_kd] = knnsearch(kd_all, [x, y], 'K', length(node_x));
    kd_name = '(none)';  kd_dist = inf;
    for i = 1:length(idx_kd)
        t = node_type{idx_kd(i)};
        if strcmp(t, 'S') || strcmp(t, 'W')
            kd_name = node_label{idx_kd(i)};
            kd_dist = dist_kd(i);
            break;
        end
    end
    t_kd = toc(t0) * 1000;

    fprintf('Linked List  [O(N) scan + type filter]:\n');
    if ~isempty(ll_res)
        fprintf('  Found : %-20s  %.1f m  |  Time: %.4f ms\n', ll_res.name, ll_res.dist, t_ll);
    else
        fprintf('  No signal/waiting point found.\n');
    end

    fprintf('KD-Tree      [O(N) worst-case with type filter]:\n');
    fprintf('  Found : %-20s  %.1f m  |  Time: %.4f ms\n', kd_name, kd_dist, t_kd);
    perf_summary(t_ll, t_kd, 'Linked List', 'KD-Tree');
end


%% =========================================================================
%  GRAPH ALGORITHMS
%% =========================================================================

function [path, hops] = ri_bfs(adj, src, dst, N)
% BFS - shortest path by hop count.  O(V + E).
    visited      = false(1, N);
    parent       = zeros(1, N);
    queue        = src;
    visited(src) = true;
    parent(src)  = -1;
    found        = false;

    while ~isempty(queue)
        u = queue(1);  queue(1) = [];
        if u == dst;  found = true;  break;  end
        for e = adj{u}
            if ~visited(e.to)
                visited(e.to) = true;
                parent(e.to)  = u;
                queue(end+1)  = e.to;
            end
        end
    end

    if ~found;  path = [];  hops = 0;  return;  end
    path = dst;
    while parent(path(1)) ~= -1
        path = [parent(path(1)), path];
    end
    hops = length(path) - 1;
end


function [path, dist_to_dst] = ri_dijkstra(adj, src, dst, N)
% Dijkstra with a min-heap (Nx2 matrix).  O((V + E) log V).
    dist_arr      = inf(1, N);
    prev          = zeros(1, N);
    dist_arr(src) = 0;
    prev(src)     = -1;
    heap          = [src, 0];

    while ~isempty(heap)
        [~, idx] = min(heap(:, 2));
        u = heap(idx, 1);
        heap(idx, :) = [];
        if u == dst;  break;  end
        for e = adj{u}
            alt = dist_arr(u) + e.w;
            if alt < dist_arr(e.to)
                dist_arr(e.to) = alt;
                prev(e.to)     = u;
                row = find(heap(:, 1) == e.to, 1);
                if isempty(row)
                    heap(end+1, :) = [e.to, alt];
                else
                    heap(row, 2) = alt;
                end
            end
        end
    end

    dist_to_dst = dist_arr(dst);
    if isinf(dist_to_dst);  path = [];  return;  end
    path = dst;
    while prev(path(1)) ~= -1
        path = [prev(path(1)), path];
    end
end


function d = ri_path_len(path, node_x, node_y)
% Sum of Euclidean distances along a node-index path.
    d = 0;
    for i = 1:length(path) - 1
        d = d + norm([node_x(path(i)) - node_x(path(i+1)), ...
                      node_y(path(i)) - node_y(path(i+1))]);
    end
end


%% =========================================================================
%  LINKED LIST HELPERS
%% =========================================================================

function result = ll_n_closest(ll, x, y, n)
% Returns struct array of the N closest 'key'-type entries, sorted by dist.
    names = {};
    dists = [];
    cur = ll.head;
    while ~isempty(cur)
        p = cur.data;
        if strcmp(p.type, 'key')
            names{end+1} = p.name;
            dists(end+1) = norm([p.x - x, p.y - y]);
        end
        cur = cur.next;
    end
    if isempty(names);  result = struct('name', {}, 'dist', {});  return;  end
    [sorted_d, order] = sort(dists);
    sorted_names = names(order);
    n = min(n, length(names));
    result = struct('name', sorted_names(1:n), 'dist', num2cell(sorted_d(1:n)));
end


function result = ll_range(ll, x, y, max_dist)
% Returns struct array of all 'key'-type entries within max_dist.
    names = {};
    dists = [];
    cur = ll.head;
    while ~isempty(cur)
        p = cur.data;
        if strcmp(p.type, 'key')
            d = norm([p.x - x, p.y - y]);
            if d <= max_dist
                names{end+1} = p.name;
                dists(end+1) = d;
            end
        end
        cur = cur.next;
    end
    if isempty(names);  result = struct('name', {}, 'dist', {});  return;  end
    result = struct('name', names, 'dist', num2cell(dists));
end


function result = ll_closest_typed(ll, x, y, types)
% Returns the single closest entry whose type appears in the types cell array.
    best_d = inf;
    result = [];
    cur = ll.head;
    while ~isempty(cur)
        p = cur.data;
        if any(strcmp(p.type, types))
            d = norm([p.x - x, p.y - y]);
            if d < best_d
                best_d     = d;
                result.name = p.name;
                result.dist = d;
            end
        end
        cur = cur.next;
    end
end


% -------------------------------------------------------------------------
function q11_full_report(adj, node_x, node_y, node_label, ...
                          key_x, key_y, key_label, sig_x, sig_y, sig_label, ...
                          numKey, numSig, WAITING_IDX, kd_keys)
% Q11: Full distance and timing report across ALL point combinations.
%
%  Sections printed:
%   A) Waiting -> Key   : graph distance, hops, Dijkstra time  (all W x K)
%   B) Key    -> Key    : graph distance, hops, Dijkstra time  (all K x K)
%   C) Signal -> nearest Key  : straight-line via LL scan vs KD-Tree
%   D) Straight-line distance matrix  (all K x K)
%   E) Lookup benchmark  : LL vs KD-Tree averaged over all key points

    N   = length(adj);
    sep = repmat('=', 1, 72);
    fprintf('\n%s\n  FULL DISTANCE & TIME REPORT\n%s\n', sep, sep);

    % ---- A: Waiting -> Key (graph) ---------------------------------------
    fprintf('\nA) WAITING AREA -> KEY POINT  (Dijkstra graph distance)\n');
    fprintf('%s\n', repmat('-', 1, 72));
    hdr = '  %-6s  %-25s  %10s  %6s  %10s';
    fprintf([hdr '\n'], 'Route', 'Path', 'Dist (m)', 'Hops', 'Time (ms)');
    fprintf('%s\n', repmat('-', 1, 72));

    T_wk = zeros(2, numKey);
    for w = 1:2
        src = numKey + WAITING_IDX(w);
        for k = 1:numKey
            t0 = tic;
            [path, dist] = ri_dijkstra(adj, src, k, N);
            t_ms = toc(t0) * 1000;
            T_wk(w, k) = t_ms;
            lbl = sprintf('W%d->K%d', w, k);
            if isempty(path)
                fprintf('  %-6s  %-25s  %10s  %6s  %10.4f\n', ...
                    lbl, '(no path)', '-', '-', t_ms);
            else
                short_path = strjoin(node_label(path), '->');
                if length(short_path) > 25
                    short_path = [short_path(1:22) '...'];
                end
                fprintf('  %-6s  %-25s  %10.1f  %6d  %10.4f\n', ...
                    lbl, short_path, dist, length(path)-1, t_ms);
            end
        end
    end
    fprintf('  Avg Dijkstra time (W->K): %.4f ms\n', mean(T_wk(:)));

    % ---- B: Key -> Key (graph) ------------------------------------------
    fprintf('\nB) KEY POINT -> KEY POINT  (Dijkstra graph distance)\n');
    fprintf('%s\n', repmat('-', 1, 72));
    fprintf([hdr '\n'], 'Route', 'Path', 'Dist (m)', 'Hops', 'Time (ms)');
    fprintf('%s\n', repmat('-', 1, 72));

    T_kk = [];
    for k1 = 1:numKey
        for k2 = k1+1:numKey
            t0 = tic;
            [path, dist] = ri_dijkstra(adj, k1, k2, N);
            t_ms = toc(t0) * 1000;
            T_kk(end+1) = t_ms;
            lbl = sprintf('K%d->K%d', k1, k2);
            if isempty(path)
                fprintf('  %-6s  %-25s  %10s  %6s  %10.4f\n', ...
                    lbl, '(no path)', '-', '-', t_ms);
            else
                short_path = strjoin(node_label(path), '->');
                if length(short_path) > 25
                    short_path = [short_path(1:22) '...'];
                end
                fprintf('  %-6s  %-25s  %10.1f  %6d  %10.4f\n', ...
                    lbl, short_path, dist, length(path)-1, t_ms);
            end
        end
    end
    fprintf('  Avg Dijkstra time (K->K): %.4f ms\n', mean(T_kk));

    % ---- C: Each signal point -> nearest key point ----------------------
    fprintf('\nC) SIGNAL POINT -> NEAREST KEY POINT  (straight-line lookup)\n');
    fprintf('%s\n', repmat('-', 1, 72));
    hdr2 = '  %-10s  %-25s  %10s  %-10s  %-25s  %10s';
    fprintf([hdr2 '\n'], 'Signal', 'LL nearest', 'LL dist(m)', ...
        'Time(ms)', 'KD nearest', 'KD dist(m)');
    fprintf('%s\n', repmat('-', 1, 72));

    T_ll_sig = zeros(1, numSig);
    T_kd_sig = zeros(1, numSig);
    for s = 1:numSig
        sx = sig_x(s);  sy = sig_y(s);

        % Linked list scan
        t0 = tic;
        best_name_ll = ''; best_d_ll = inf;
        cur = ll_find_all_keys_inline(key_x, key_y, key_label, sx, sy);
        [best_d_ll, best_idx_ll] = min(cur);
        best_name_ll = key_label{best_idx_ll};
        T_ll_sig(s) = toc(t0) * 1000;

        % KD-Tree
        t0 = tic;
        [idx_kd, dist_kd] = knnsearch(kd_keys, [sx, sy]);
        T_kd_sig(s) = toc(t0) * 1000;

        fprintf('  %-10s  %-25s  %10.1f  %10.4f  %-25s  %10.1f\n', ...
            sig_label{s}, best_name_ll, best_d_ll, T_ll_sig(s), ...
            key_label{idx_kd}, dist_kd);
    end
    fprintf('  Avg LL time : %.4f ms   Avg KD-Tree time : %.4f ms\n', ...
        mean(T_ll_sig), mean(T_kd_sig));

    % ---- D: Straight-line distance matrix (K x K) -----------------------
    fprintf('\nD) STRAIGHT-LINE DISTANCE MATRIX  (metres, key points only)\n');
    fprintf('%s\n', repmat('-', 1, 72));
    fprintf('%8s', '');
    for k = 1:numKey;  fprintf('%8s', key_label{k});  end
    fprintf('\n');
    for k1 = 1:numKey
        fprintf('%8s', key_label{k1});
        for k2 = 1:numKey
            if k1 == k2
                fprintf('%8s', '-');
            else
                d = norm([key_x(k1)-key_x(k2), key_y(k1)-key_y(k2)]);
                fprintf('%8.0f', d);
            end
        end
        fprintf('\n');
    end

    % ---- E: Lookup benchmark (LL vs KD-Tree, 100 reps) ------------------
    fprintf('\nE) LOOKUP BENCHMARK  (100 reps x %d key points, random query points)\n', numKey);
    fprintf('%s\n', repmat('-', 1, 72));

    rng(42);
    qx = rand(100,1) * 1200;
    qy = rand(100,1) * 1200;

    t_ll_total = 0;
    t_kd_total = 0;
    for q = 1:100
        t0 = tic;
        dists_ll = ll_find_all_keys_inline(key_x, key_y, key_label, qx(q), qy(q));
        [~] = min(dists_ll);
        t_ll_total = t_ll_total + toc(t0) * 1000;

        t0 = tic;
        knnsearch(kd_keys, [qx(q), qy(q)]);
        t_kd_total = t_kd_total + toc(t0) * 1000;
    end

    fprintf('  Linked List  avg : %.4f ms   total : %.2f ms   O(K) per query\n', ...
        t_ll_total/100, t_ll_total);
    fprintf('  KD-Tree      avg : %.4f ms   total : %.2f ms   O(log K) per query\n', ...
        t_kd_total/100, t_kd_total);
    speedup = t_ll_total / max(t_kd_total, 1e-9);
    fprintf('  KD-Tree is %.1fx faster on average over 100 queries.\n', speedup);

    fprintf('\n%s\n  END OF REPORT\n%s\n', sep, sep);
end


function dists = ll_find_all_keys_inline(key_x, key_y, ~, qx, qy)
% Compute distances from (qx,qy) to every key point (array-based LL proxy).
    dists = sqrt((key_x - qx).^2 + (key_y - qy).^2);
end


%% =========================================================================
%  UI HELPERS
%% =========================================================================

function print_menu(numKey, key_label, key_x, key_y, sig_label, sig_x, sig_y, ...
                    node_label, node_x, node_y, WAITING_IDX, numSig)

    % Known landmark names for K1-K7 (fixed for this dataset)
    landmark_names = { ...
        'One Pool Street', ...
        'Marshgate', ...
        'ArcelorMittal Orbit', ...
        'West Ham Stadium', ...
        'Copper Box Arena', ...
        'Here East', ...
        'Aquatics Centre' };

    fprintf('\n+----------------------------------------------------------+\n');
    fprintf('|              ROBOT NAVIGATION INTERFACE                  |\n');
    fprintf('+----------------------------------------------------------+\n');

    % --- Key points legend -----------------------------------------------
    fprintf('|  KEY POINTS                                              |\n');
    for i = 1:numKey
        if i <= length(landmark_names)
            name_str = landmark_names{i};
        else
            name_str = '(unknown)';
        end
        line = sprintf('|    %s = %-28s (x=%.0f, y=%.0f)', ...
            key_label{i}, name_str, key_x(i), key_y(i));
        % pad to fixed width
        line = [line, repmat(' ', 1, max(0, 60 - length(line))), '|'];
        fprintf('%s\n', line);
    end

    % --- Waiting areas legend --------------------------------------------
    fprintf('|  WAITING AREAS                                           |\n');
    for w = 1:length(WAITING_IDX)
        idx  = numKey + WAITING_IDX(w);
        line = sprintf('|    W%d = %-35s (x=%.0f, y=%.0f)', ...
            w, node_label{idx}, node_x(idx), node_y(idx));
        line = [line, repmat(' ', 1, max(0, 60 - length(line))), '|'];
        fprintf('%s\n', line);
    end

    % --- Signal points legend --------------------------------------------
    fprintf('|  SIGNAL POINTS                                           |\n');
    for s = 1:numSig
        line = sprintf('|    %-4s  (x=%.0f, y=%.0f)', ...
            sig_label{s}, sig_x(s), sig_y(s));
        line = [line, repmat(' ', 1, max(0, 60 - length(line))), '|'];
        fprintf('%s\n', line);
    end

    % --- Query menu ------------------------------------------------------
    fprintf('+----------------------------------------------------------+\n');
    fprintf('|  QUERIES                                                 |\n');
    fprintf('|  1.  Route: W -> K          (BFS + Dijkstra)            |\n');
    fprintf('|  2.  N closest key points   to a position (x, y)        |\n');
    fprintf('|  3.  Distance: W -> K       (graph metres)              |\n');
    fprintf('|  4.  N closest key points   to a waiting area           |\n');
    fprintf('|  5.  Tourist tour           through all key points      |\n');
    fprintf('|  6.  Key points within D m  of (x, y)                   |\n');
    fprintf('|  7.  Key -> Key path        (BFS + Dijkstra)            |\n');
    fprintf('|  8.  All key points ranked  by distance from (x, y)     |\n');
    fprintf('|  9.  Hop count: W -> K      (BFS minimum hops)         |\n');
    fprintf('|  10. Closest signal/wait    to (x, y)                   |\n');
    fprintf('|  11. FULL REPORT  all distances + times (every combo)   |\n');
    fprintf('|  0.  Exit                                                |\n');
    fprintf('+----------------------------------------------------------+\n');
end


function val = prompt_int(msg, lo, hi)
% Repeatedly asks for an integer in [lo, hi] until the user provides one.
    val = [];
    while isempty(val) || ~isnumeric(val) || val < lo || val > hi || val ~= floor(val)
        val = input(msg);
        if isempty(val) || ~isnumeric(val) || val < lo || val > hi || val ~= floor(val)
            fprintf('[!] Please enter a whole number between %d and %d.\n', lo, hi);
            val = [];
        end
    end
end


function perf_summary(t1, t2, name1, name2)
% Prints a one-line performance comparison with speedup factor.
    fprintf('\n  -- Performance --\n');
    fprintf('  %-12s : %.4f ms\n', name1, t1);
    fprintf('  %-12s : %.4f ms\n', name2, t2);
    if t2 < t1
        fprintf('  %s is %.1fx faster.\n', name2, t1 / max(t2, 1e-9));
    elseif t1 < t2
        fprintf('  %s is %.1fx faster.\n', name1, t2 / max(t1, 1e-9));
    else
        fprintf('  Both methods ran in equal time.\n');
    end
    fprintf('\n');
end
