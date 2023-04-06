classdef cspace_converter
% This class is used to convert between 2D C-Space and Euclidean Space.
    
    methods
        
        function c_pt convert_Euclidean_to_Config(e_pt, geom)
            % Perform Inverse Kinematics to bring a point from Euclidean
            % space into C-space for a Two-Link Planar Manipulator
            % Inputs: 
            % e_pt Euclidean-Space point in form [x, y]
            % geom Link lengths in form [a1, a2]
            % 
            % Outputs: 
            % c_pt C-Space point in form [theta1, theta2]
            
            x = e_pt[1];
            y = e_pt[2];
            
            a1 = geom[1];
            a2 = geom[2];
            
            t2 = acosd( (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2) );
            t1 = atan2d(y, x) + atan2d( a2*sin(t2), a1 + a2*cos(t2) );
            
            c_pt = [t1 t2]
        end
        
        function e_pt convert_Euclidean_to_Config(c_pt, geom)
            % Perform Forward Kinematics
            % Inputs: 
            % c_pt C-Space point in form [theta1, theta2]
            % geom Link lengths in form [a1, a2]
            % 
            % Outputs: 
            % e_pt Euclidean-Space point in form [x, y]
            
            % TODO
            e_pt = 0
        end
        
    end
    
end