classdef obstacle
% This class stores the edges of an obstacle.
% 
% env_pot_field properties:
%   edges - List of 2D points representing edge vertices.
    
    properties
        edges = [];
    end
    
    methods
        function set_edges(edges_new)
            edges = edges_new;
        end
    end
    
end