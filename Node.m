classdef Node < handle
    properties
        data
        next = []
    end
    
    methods
        function obj = Node(d)
            obj.data = d;
        end
    end
end