classdef obstacle_map < handle
% This class is used to store and plot a map representation. Can be used
% for either Euclidean Space or Configuration Space.
% 
% env_pot_field properties:
%   obstacles - List of obstacles in polygon representation
%   goal - Goal stores in polygon

    properties
        obstacles = obstacle.empty;
        goal = zeros(2);
        sz = [100, 100];
    end
    
    methods
        
        function obj = obstacle_map(sz)
            obj.sz = sz;
        end
        
        function reset(obj)
            obj.obstacles = obstacle.empty;
        end
        
        function set_goal(new_goal)
            obj.goal = new_goal;
        end
        
        function add_obstacle(obj, obst, check_quadrants)
            disp(obj.obstacles)
            obj.obstacles(end+1) = obst;
            
            if (~check_quadrants)
                return
            end
            
            % Add obstacles from other 8 quadrants if necessary.
            if any(obst.edges(:,1) > 360) || any(obst.edges(:,3) > 360) && ~any(obst.edges(:,2) < 0) && ~any(obst.edges(:,4) < 0) && ~any(obst.edges(:,2) > 360) && ~any(obst.edges(:,4) > 360)
                obst2_edges = obst.edges;
                obst2_edges(:,1) = obst2_edges(:,1) - 360;
                obst2_edges(:,3) = obst2_edges(:,3) - 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            if any(obst.edges(:,1) < 0) || any(obst.edges(:,3) < 0) && ~any(obst.edges(:,2) < 0) && ~any(obst.edges(:,4) < 0) && ~any(obst.edges(:,2) > 360) && ~any(obst.edges(:,4) > 360)
                obst2_edges = obst.edges;
                obst2_edges(:,1) = obst2_edges(:,1) + 360;
                obst2_edges(:,3) = obst2_edges(:,3) + 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            if any(obst.edges(:,2) > 360) || any(obst.edges(:,4) > 360) && ~any(obst.edges(:,1) < 0) && ~any(obst.edges(:,3) < 0) && ~any(obst.edges(:,1) > 360) && ~any(obst.edges(:,3) > 360)
                obst2_edges = obst.edges;
                obst2_edges(:,2) = obst2_edges(:,2) - 360;
                obst2_edges(:,4) = obst2_edges(:,4) - 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            if any(obst.edges(:,2) < 0) || any(obst.edges(:,2) < 0) && ~any(obst.edges(:,1) < 0) && ~any(obst.edges(:,3) < 0) && ~any(obst.edges(:,1) > 360) && ~any(obst.edges(:,3) > 360)
                obst2_edges = obst.edges;
                obst2_edges(:,2) = obst2_edges(:,2) + 360;
                obst2_edges(:,4) = obst2_edges(:,4) + 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            
            if any(obst.edges(:,1) > 360) || any(obst.edges(:,3) > 360) && any(obst.edges(:,2) > 360) || any(obst.edges(:,4) > 360)
                obst2_edges = obst.edges;
                obst2_edges(:,1) = obst2_edges(:,1) - 360;
                obst2_edges(:,2) = obst2_edges(:,2) - 360;
                obst2_edges(:,3) = obst2_edges(:,3) - 360;
                obst2_edges(:,4) = obst2_edges(:,4) - 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            if any(obst.edges(:,1) > 360) || any(obst.edges(:,3) > 360) && any(obst.edges(:,2) < 0) || any(obst.edges(:,4) < 0)
                obst2_edges = obst.edges;
                obst2_edges(:,1) = obst2_edges(:,1) - 360;
                obst2_edges(:,2) = obst2_edges(:,2) + 360;
                obst2_edges(:,3) = obst2_edges(:,3) - 360;
                obst2_edges(:,4) = obst2_edges(:,4) + 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            if any(obst.edges(:,1) < 0) || any(obst.edges(:,3) < 0) && any(obst.edges(:,2) > 360) || any(obst.edges(:,4) > 360)
                obst2_edges = obst.edges;
                obst2_edges(:,1) = obst2_edges(:,1) + 360;
                obst2_edges(:,2) = obst2_edges(:,2) - 360;
                obst2_edges(:,3) = obst2_edges(:,3) + 360;
                obst2_edges(:,4) = obst2_edges(:,4) - 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            if any(obst.edges(:,1) < 0) || any(obst.edges(:,3) < 0) && any(obst.edges(:,2) < 0) || any(obst.edges(:,4) < 0)
                obst2_edges = obst.edges;
                obst2_edges(:,1) = obst2_edges(:,1) + 360;
                obst2_edges(:,2) = obst2_edges(:,2) + 360;
                obst2_edges(:,3) = obst2_edges(:,3) + 360;
                obst2_edges(:,4) = obst2_edges(:,4) + 360;
                obj.obstacles(end+1) = obstacle(obst2_edges);
            end
            
            disp(obj.obstacles)
        end
        
        function plotMap(obj, centerAtZero)
            figure
            hold on
            for obstacle = obj.obstacles
                for i = 1:size(obstacle.edges, 1)
                    edge = obstacle.edges(i,:);
                    plot([edge(1), edge(3)], [edge(2), edge(4)], 'k-')
                    plot([edge(1), edge(3)], [edge(2)+360, edge(4)+360], 'k-')
                    plot([edge(1)+360, edge(3)+360], [edge(2), edge(4)], 'k-') % Plot normal edge
                    plot([edge(1)+360, edge(3)+360], [edge(2)+360, edge(4)+360], 'k-') % Plot normal edge
                end
            end
            
            if (centerAtZero)
                xlim([-obj.sz(1)/2, obj.sz(1)/2])
                ylim([-obj.sz(1)/2, obj.sz(2)/2])
            else (centerAtZero)
                xlim([0, obj.sz(1)])
                ylim([0, obj.sz(2)])
            end
                
        end
        
        
    end % End methods
end % End class