classdef obstacle < handle
% This class stores the edges of an obstacle. This class simply stores
% edges and is space-agnostic, so can be used for either C-space or
% Euclidean space.
% 
% env_pot_field properties:
%   edges - List of 2D points representing edge vertices.
    
    properties
        edges = [];
    end
    
    methods
        
        function obj = obstacle(edges_new)
            obj.edges = edges_new;
        end
        
        function set_edges(obj, edges_new)
            obj.edges = edges_new;
        end
        
        function [pts_x, pts_y] = get_inside_pts(obj, x, y)
            in = inpolygon(x, y, obj.edges(:,1), obj.edges(:,2));
            
            pts_x = x(in);
            pts_y = y(in);
        end
        
    end
    
end