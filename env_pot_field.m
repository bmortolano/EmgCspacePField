classdef env_pot_field
% This class is used to generated a potential field from an input of
% obstacles and goals from the environment.
% 
% env_pot_field properties:
%   field - 2D potential field matrix
%   obstacles - List of obstacles in polygon representation
%   goal - Goal stores in polygon
%   repulsion_decay - Rate of exponential decay of potential field
%   repulsion_magnitude - Strength of potential 1 unit away from obstacle


    properties
        field = [];
        obstacles = [];
        goal = [];
        size = [100 100];
        repulsion_decay = 5;
        repulsion_magnitude = 20;
    end
    
    methods
        
        function set_goal(new_goal)
            goal = new_goal;
        end
        
        function add_obstacle(obst)
            obstacles = [obstacles obst];
        end
        
        function d = point_to_line(pt, v1, v2)
            a = v1 - v2;
            b = pt - v2;
            d = norm(cross(a,b)) / norm(a);
        end
        
        function upate_pot_field(size)
            field = zeros(size);
            
            % Iterate across entire field
            for i=1:size(field,1)
                for j=1:size(field,2)
                    
                    node_potential = 0
                    
                    % Iterate across all obstacle edges
                    for obstacle = obstacles
                        for edge = obstacle.edges
                            d = point_to_line((i,j), edge[1], edge[2]))
                            node_potential = node_potential + repulsion_magnitude * exp(-repulsion_decay * d)
                        end
                    end
                    
                    field[i, j] = node_potential;
                end
            end
            
        end
        
        
    end % End methods
end % End class