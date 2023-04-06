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
        intent_vector_xy = zeros([1,2]);
        intent_vector_c = zeros([1,2]);
        size = [100 100];
        field = [];
    end
    
    methods
        
        function reset(obj, new_size)
            obj.size = new_size;
            obj.field = zeros(size);
        end
        
    end
    
end