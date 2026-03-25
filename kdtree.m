clc;
clear;

keyNames = ["One Pool Street", "Marshgate", "ArcelorMittal Orbit", "West Ham Stadium", "Copper Box Arena", "Here East", "Aquatics Centre"];

keyPoints = [
    1048.6000  198.9000
     937.1000  112.7000
     838.1000  140.3000
     703.8000  111.6000
     399.1000  855.5000
     217.7000 1023.5000
     944.1000  374.7000
];

keyTree = KDTreeSearcher(keyPoints);

robotPos = [900 200];

[idx, dist] = knnsearch(keyTree, robotPos);

fprintf('Closest landmark:\n');
fprintf('%s | x = %.4f | y = %.4f | distance = %.4f\n', keyNames(idx), keyPoints(idx,1), keyPoints(idx,2), dist);