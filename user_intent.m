classdef user_intent < handle
% This class is able to determine (or simulate) a desired action and create
% a corresponding potential field. This is used by the 'planner' class for
% local path planning.
% 
% user_intent properties:
%   intent_vector - 4-element array of desired velocity in c-space
%   field - 2D potential field matrix
    
    properties
        intent_vector = zeros([4,1]);
        alpha = zeros([4,1]);
        eta = zeros([1,2]);
        size = [100 100];
        field = [];
    end
    
    methods
        
        function obj = user_intent(alpha_coeffs_in, eta_coeffs_in)
            obj.alpha = alpha_coeffs_in;
            obj.eta = eta_coeffs_in;
        end
        
        function reset(obj, new_size)
            obj.size = new_size;
            obj.field = zeros(obj.size);
        end
        
        function input_measurement(obj, measurement)
            % Put measurements into low-pass filter
            obj.intent_vector = obj.alpha .* obj.intent_vector + (1 - obj.alpha) .* measurement;
            obj.create_field(obj.eta(1) * (obj.intent_vector(1) - obj.intent_vector(2)), ...
                obj.eta(2) * (obj.intent_vector(3) - obj.intent_vector(4)))
        end
        
        function create_field(obj, max1, max2) %max1 is emg1 data point. max2 is emg2 data point
            % Create a planar potential field
            [t1,t2] = meshgrid(0:obj.size(1),0:obj.size(2));
            obj.field = -1*(max1*t1/obj.size(1) + max2*t2/obj.size(2));
        end
        
        function plot_field(obj)
            figure
            contourf(obj.field)
            title("User Intent Artificial Potential Field")
        end
        
    end
    
end
