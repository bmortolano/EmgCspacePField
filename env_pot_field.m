classdef env_pot_field < handle
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
        obstacles = obstacle.empty;
        goal = zeros(2);
        sz = [100 100];
        repulsion_decay = 0.1;
        repulsion_magnitude = 20;
    end
    
    methods
        
        function obj = env_pot_field(sz)
            obj.sz = sz;
        end
        
        function reset(obj, new_size)
            obj.sz = new_size;
            obj.field = zeros(sz);
        end
        
        function set_goal(obj, new_goal)
            obj.goal = new_goal;
        end
        
        function add_obstacle(obj, obst)
            obj.obstacles(end+1) = obst;
        end
        
        function d = point_to_line(obj, pt, v1, v2)
            a = v1 - v2;
            b = pt - v2;
            d = norm(cross(a,b)) / norm(a);
        end
        
        function update_pot_field(obj)
            obj.field = zeros(obj.sz);
            
            % Iterate across entire field
            for i=1:size(obj.field,1)
                for j=1:size(obj.field,2)
                    
                    node_potential = 0;
                    
                    % Iterate across all obstacle edges
                    for obstacle = obj.obstacles
                        for edge = obstacle.edges
%                             d = obj.point_to_line([i, j, 0], [edge(1) edge(2), 0], [edge(3) edge(4), 0]);
                            d = norm([i, j] - [edge(1) edge(2)]);
                            node_potential = node_potential + obj.repulsion_magnitude * exp(-obj.repulsion_decay * d);
                        end
                    end
                    
                    obj.field(i, j) = node_potential;
                end
            end
            
        end
        
        
    end % End methods
end % End class