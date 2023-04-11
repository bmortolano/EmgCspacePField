classdef user_intent < handle
% This class is able to determine (or simulate) a desired action and create
% a corresponding potential field. This is used by the 'planner' class for
% local path planning.
% 
% user_intent properties:
%   intent_vector_xy - 2-element array of desired velocity in Euclidean
%   space
%   intent_vector_c - 2-element array of desired velocity in c-space
%   field - 2D potential field matrix
    
    properties
        size = [100 100];
        field = [];
    end
    
    methods
        
        function reset(obj, new_size)
            obj.size = new_size;
            obj.field = zeros(obj.size);
        end
        
        function create_field(obj, max1, max2) %max1 is emg1 data point. max2 is emg2 data point
            % Create a planar potential field
            [t1,t2] = meshgrid(0:obj.size(1),0:obj.size(2));
            obj.field = -1*(max1*t1/obj.size(1) + max2*t2/obj.size(2));
        end
        
    end
    
end
