clc; clear; close all;

%%
% Create robot parameters
link_lengths = [80, 80]; % Robot link lengths

%%
% Create a grid of size sx to use for obstacle flooding
sz = [200, 200]

linx = linspace(0, sz(1), 250);
liny = linspace(0, sz(2), 250);

[gridx, gridy] = meshgrid(linx, liny);

gridx = reshape(gridx.',1,[]);
gridy = reshape(gridy.',1,[]);

%%
euc_map = obstacle_map(sz);

% Create obstacles
obst1_edges = [
    [[50, 50], [70, 50]], 
    [[70, 50], [90, 70]],
    [[90, 70], [90, 90]],
    [[90, 90], [70, 110]],
    [[70, 110], [50, 110]],
    [[50, 110], [30, 90]],
    [[30, 90], [30, 70]],
    [[30, 70], [50, 50]]];

euc_map.add_obstacle(obstacle(obst1_edges))
euc_map.plotMap()

%%
% Create corresponding C-Space euc_map
cmap_sz = [360 360];
c_map = obstacle_map(cmap_sz);

for e_obstacle = euc_map.obstacles
    c_obstacle = convert_eObst_to_c_Obst(e_obstacle, link_lengths, gridx, gridy)
    c_map.add_obstacle(c_obstacle);
end

c_map.plotMap()

%%
env_field = env_pot_field(cmap_sz);
env_field.add_obstacle(c_map.obstacles(1));
env_field.update_pot_field();

figure
contourf(env_field.field)

%%
function [t1 t2] = convert_Euclidean_to_Config(e_pt, geom)
    % Perform Inverse Kinematics to bring a point from Euclidean
    % space into C-space for a Two-Link Planar Manipulator
    % Inputs: 
    % e_pt Euclidean-Space point in form [x, y]
    % geom Link lengths in form [a1, a2]
    % 
    % Outputs: 
    % c_pt C-Space point in form [theta1, theta2]

    x = e_pt(1);
    y = e_pt(2);

    a1 = geom(1);
    a2 = geom(2);

    t2 = acosd( (x^2 + y^2 - a1^2 - a2^2) / (2 * a1 * a2) );
    t1 = atan2d(y, x) + atan2d( a2*sin(t2), a1 + a2*cos(t2) );
end

function c_obst = convert_eObst_to_c_Obst(e_obst, geom, gridx, gridy)

    % Use grid to determine what points within the query grid lie within
    % the obstacle polygon
    [e_obst_x, e_obst_y] = e_obst.get_inside_pts(gridx, gridy);
    
    %Transform all of these into C-space
    c_obst_x = zeros(size(e_obst_x));
    c_obst_y = zeros(size(e_obst_y));
    
    for i=1:size(e_obst_x, 2)
        [c_obst_x(i), c_obst_y(i)] = convert_Euclidean_to_Config([e_obst_x(i), e_obst_y(i)], geom);
    end
    
    % Create polygon around C-Space obstacle
    shp = alphaShape(c_obst_x', c_obst_y', 20);
    [~,V] = boundaryFacets(shp);
    shp = polyshape(V,'Simplify',false);
    
    % Create edges from points
    edges = pts_to_edges(shp.Vertices);
    
    % Return C-Space obstacle
    c_obst = obstacle(edges);
end

function edges = pts_to_edges(pts)
    edges = zeros(size(pts, 1), 4);

    for i=1:(size(pts, 1)-1)
        edges(i,:) = [pts(i,:) pts(i+1,:)];
    end
    
    edges(i+1,:) = [pts(i+1,:) pts(1,:)];
end