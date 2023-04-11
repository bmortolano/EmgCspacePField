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
        sdf = [];
        obstacles = obstacle.empty;
        goal = zeros(2);
        sz = [100 100];
        repulsion_decay = 0.02;
        repulsion_magnitude = 5;
    end
    
    methods
        
        function obj = env_pot_field(sz)
            obj.sz = sz;
        end
        
        function reset(obj, new_size)
            obj.sz = new_size;
            obj.field = zeros(sz);
            obj.sdf = zeros(sz);
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
        
        function update_pot_field(obj, wrapAround)
            obj.field = zeros(obj.sz);
            sdf = 100000 * ones(size(obj.field));
            
            for obst = obj.obstacles
                sdf = obj.signed_dist_field(sdf, obst, 360);
            end
            
            obj.sdf = sdf;
            
            for i=1:size(obj.sdf,2) % Iterate over x
                for j=1:size(obj.sdf,1) % Iterate over y
                    obj.field(i, j) = obj.repulsion_magnitude * exp(-obj.repulsion_decay * obj.sdf(i, j));
                end
            end
            
        end
        
        function sdf_out = signed_dist_field(obj, sdf_in, obstacle, wrapAroundNumber)
            % Overwrite input signed distance field using some obstacle
            obs_vertices = obstacle.get_vertices();
            sdf_out = zeros(size(sdf_in));
            
            % Iterate across entire field
            for i=1:size(sdf_out,2) % Iterate over x
                for j=1:size(sdf_out,1) % Iterate over y
                    [d1, ~, ~] = obj.p_poly_dist(j, i, obs_vertices(:,2), obs_vertices(:, 1));
                    [d2, ~, ~] = obj.p_poly_dist(j, i-wrapAroundNumber, obs_vertices(:,2), obs_vertices(:, 1));
                    [d3, ~, ~] = obj.p_poly_dist(j-wrapAroundNumber, i, obs_vertices(:,2), obs_vertices(:, 1));
                    [d4, ~, ~] = obj.p_poly_dist(j-wrapAroundNumber, i-wrapAroundNumber, obs_vertices(:,2), obs_vertices(:, 1));
                    
                    d = min(d1, d2);
                    d = min(d, d3);
                    d = min(d, d4);
                    sdf_out(j, i) = min(sdf_in(j, i), d);
                end
            end
           
        end
        
        % Adapted from code by Alejandro Weinstein on File Exchange
        function [d,x_poly,y_poly] = p_poly_dist(obj, x, y, xv, yv) 
            % function:	p_poly_dist
            % Description:	distance from point to polygon whose vertices are specified by the
            %              vectors xv and yv
            % Input:  
            %    x - point's x coordinate
            %    y - point's y coordinate
            %    xv - vector of polygon vertices x coordinates
            %    yv - vector of polygon vertices x coordinates
            % Output: 
            %    d - distance from point to polygon (defined as a minimal distance from 
            %        point to any of polygon's ribs, positive if the point is outside the
            %        polygon and negative otherwise)
            %    x_poly: x coordinate of the point in the polygon closest to x,y
            %    y_poly: y coordinate of the point in the polygon closest to x,y
            %
            % Routines: p_poly_dist.m
            % Revision history:
            %    03/31/2008 - return the point of the polygon closest to x,y
            %               - added the test for the case where a polygon rib is 
            %                 either horizontal or vertical. From Eric Schmitz.
            %               - Changes by Alejandro Weinstein
            %    7/9/2006  - case when all projections are outside of polygon ribs
            %    23/5/2004 - created by Michael Yoshpe 
            % Remarks:
            %*******************************************************************************

            % If (xv,yv) is not closed, close it.
            xv = xv(:);
            yv = yv(:);
            Nv = length(xv);
            if ((xv(1) ~= xv(Nv)) || (yv(1) ~= yv(Nv)))
                xv = [xv ; xv(1)];
                yv = [yv ; yv(1)];
            %     Nv = Nv + 1;
            end
            % linear parameters of segments that connect the vertices
            % Ax + By + C = 0
            A = -diff(yv);
            B =  diff(xv);
            C = yv(2:end).*xv(1:end-1) - xv(2:end).*yv(1:end-1);
            % find the projection of point (x,y) on each rib
            AB = 1./(A.^2 + B.^2);
            vv = (A*x+B*y+C);
            xp = x - (A.*AB).*vv;
            yp = y - (B.*AB).*vv;
            % Test for the case where a polygon rib is 
            % either horizontal or vertical. From Eric Schmitz
            id = find(diff(xv)==0);
            xp(id)=xv(id);
            clear id
            id = find(diff(yv)==0);
            yp(id)=yv(id);
            % find all cases where projected point is inside the segment
            idx_x = (((xp>=xv(1:end-1)) & (xp<=xv(2:end))) | ((xp>=xv(2:end)) & (xp<=xv(1:end-1))));
            idx_y = (((yp>=yv(1:end-1)) & (yp<=yv(2:end))) | ((yp>=yv(2:end)) & (yp<=yv(1:end-1))));
            idx = idx_x & idx_y;
            % distance from point (x,y) to the vertices
            dv = sqrt((xv(1:end-1)-x).^2 + (yv(1:end-1)-y).^2);
            if(~any(idx)) % all projections are outside of polygon ribs
               [d,I] = min(dv);
               x_poly = xv(I);
               y_poly = yv(I);
            else
               % distance from point (x,y) to the projection on ribs
               dp = sqrt((xp(idx)-x).^2 + (yp(idx)-y).^2);
               [min_dv,I1] = min(dv);
               [min_dp,I2] = min(dp);
               [d,I] = min([min_dv min_dp]);
               if I==1, %the closest point is one of the vertices
                   x_poly = xv(I1);
                   y_poly = yv(I1);
               elseif I==2, %the closest point is one of the projections
                   idxs = find(idx);
                   x_poly = xp(idxs(I2));
                   y_poly = yp(idxs(I2));
               end
            end
            if(inpolygon(x, y, xv, yv)) 
               d = -d;
            end
        end
        
        function plot_sdf(obj)
            figure
            contourf(obj.sdf)
            title("Signed Distance Field")
        end
        
        function plot_field(obj)
            figure
            contourf(obj.field)
            title("Artificial Potential Field")
        end
        
        
    end % End methods
end % End class