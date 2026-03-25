classdef SinglyLinkedList < handle
    properties
        head = []
        tail = []
        length = 0
    end
    
    methods
        function addLast(L, pointData)
            n = Node(pointData);
            
            if isempty(L.head)
                L.head = n;
                L.tail = n;
            else
                L.tail.next = n;
                L.tail = n;
            end
            
            L.length = L.length + 1;
        end
        
        function displayList(L)
            current = L.head;
            
            while ~isempty(current)
                point = current.data;
                fprintf('Name: %s | x: %.4f | y: %.4f | Type: %s\n', ...
                    point.name, point.x, point.y, point.type);
                current = current.next;
            end
        end


    function closestPoint = findClosest(L, xq, yq)
    current = L.head;
    minDist = inf;
    closestPoint = [];
    
    while ~isempty(current)
        point = current.data;
        
        if strcmp(point.type, "key")
            
            dx = point.x - xq;
            dy = point.y - yq;
            dist = sqrt(dx^2 + dy^2);
            
            if dist < minDist
                minDist = dist;
                closestPoint = point;
            end
        end
        
        current = current.next;
    end
end
    end
end