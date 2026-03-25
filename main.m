% main.m  -  Single entry point that integrates all project files:
%
%   Node.m              linked list node class
%   SinglyLinkedList.m  linked list with addLast / displayList / findClosest
%   testLinkedList.m    standalone linked-list demo  (hardcoded points)
%   kdtree.m            standalone KD-tree demo      (hardcoded points)
%   kaya_code.m         main script: map build, point detection, graph,
%                       BFS / Dijkstra / TSP
%
% Run this file in MATLAB. park.mat must be in the same folder.

clc; clear; close all;

%% =========================================================================
%  STEP 1 - Run the main map / graph / pathfinding script
%% =========================================================================
fprintf('========================================\n');
fprintf(' STEP 1: kaya_code  (map + pathfinding) \n');
fprintf('========================================\n\n');

run('map_search_algorithms.m');

%% =========================================================================
%  STEP 2 - Linked List integration using DETECTED points
%  (extends testLinkedList.m to use real sensor-detected coordinates)
%% =========================================================================
fprintf('\n========================================\n');
fprintf(' STEP 2: Linked List (detected points)  \n');
fprintf('========================================\n\n');

list = SinglyLinkedList();

for i = 1:numKey
    list.addLast(struct('name', key_label{i}, 'x', key_x(i), 'y', key_y(i), 'type', "key"));
end
for i = 1:numSig
    list.addLast(struct('name', sig_label{i}, 'x', sig_x(i), 'y', sig_y(i), 'type', "signal"));
end

fprintf('All detected points stored in Linked List:\n');
list.displayList();

% Change robotX / robotY to query a different robot position
robotX = 900;
robotY = 200;

fprintf('\nClosest KEY point to robot at (%.0f, %.0f):\n', robotX, robotY);
closest_ll = list.findClosest(robotX, robotY);
if ~isempty(closest_ll)
    fprintf('  Linked List -> %s  at (%.4f, %.4f)\n', closest_ll.name, closest_ll.x, closest_ll.y);
else
    fprintf('  Linked List -> no key points found\n');
end

%% =========================================================================
%  STEP 3 - KD-Tree comparison using DETECTED points
%  (extends kdtree.m to use real sensor-detected coordinates)
%% =========================================================================
fprintf('\n========================================\n');
fprintf(' STEP 3: KD-Tree (detected points)      \n');
fprintf('========================================\n\n');

keyTree = KDTreeSearcher([key_x, key_y]);
[idx_kd, dist_kd] = knnsearch(keyTree, [robotX, robotY]);
fprintf('Closest KEY point to robot at (%.0f, %.0f):\n', robotX, robotY);
fprintf('  KD-Tree     -> %s  at (%.4f, %.4f) | dist = %.4f\n', ...
    key_label{idx_kd}, key_x(idx_kd), key_y(idx_kd), dist_kd);

fprintf('\n--- Linked List vs KD-Tree summary ---\n');
fprintf('  Linked List : %s\n', closest_ll.name);
fprintf('  KD-Tree     : %s\n', key_label{idx_kd});
if strcmp(closest_ll.name, key_label{idx_kd})
    fprintf('  Both methods agree.\n');
else
    fprintf('  Results differ - check coordinate consistency.\n');
end

fprintf('\nDone. All components integrated successfully.\n');
