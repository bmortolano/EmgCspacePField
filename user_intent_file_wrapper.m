classdef user_intent_file_wrapper < handle
% This class is able to determine (or simulate) a desired action and create
% a corresponding potential field. This is used by the 'planner' class for
% local path planning.
% 
% user_intent properties:
%   intent_vector - 4-element array of desired velocity in c-space
%   field - 2D potential field matrix
    
    properties
        intent_field = user_intent([0,0,0,0], [0,0]);
        file;
        emg_data_matrix;
        row_counter = 0;
    end
    
    methods
        
        function obj = user_intent_file_wrapper(alpha_coeffs_in, eta_coeffs_in, fileIn)
            obj.intent_field = user_intent(alpha_coeffs_in, eta_coeffs_in);
            obj.emg_data_matrix = xlsread(fileIn);
            row_counter = 0;
        end
        
        function field = step(obj)
            obj.row_counter = obj.row_counter + 5;
            row = obj.emg_data_matrix(obj.row_counter, :);
            meas = [row(3), row(5), row(15), row(17)];
            obj.intent_field.input_measurement(meas')
        end
        
    end
    
end