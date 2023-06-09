clc; clear; close all;

%%
% Create robot parameters
link_lengths = [80, 80]; % Robot link lengths

%%
% Create a grid of size sx to use for obstacle flooding
sz = [300, 300]

linx = linspace(-sz(1)/2, sz(1)/2, 250);
liny = linspace(-sz(2)/2, sz(2)/2, 250);

[gridx, gridy] = meshgrid(linx, liny);

gridx = reshape(gridx.',1,[]);
gridy = reshape(gridy.',1,[]);

%%
euc_map = obstacle_map(sz);

% Create obstacles
obst1_edges = [
    [[30, 50], [50, 50]], 
    [[50, 50], [70, 70]],
    [[70, 70], [70, 90]],
    [[70, 90], [50, 110]],
    [[50, 110], [30, 110]],
    [[30, 110], [10, 90]],
    [[10, 90], [10, 70]],
    [[10, 70], [30, 50]]];

angles = linspace(0, 360, 50);
r = 30;
offset_x = -60;
offset_y = -20;
obst2_edges = [r*sind(angles)'+offset_y r*cosd(angles)'+offset_x r*sind(angles-angles(2))'+offset_y r*cosd(angles-angles(2))'+offset_x ];

euc_map.add_obstacle(obstacle(obst1_edges), 0)
euc_map.add_obstacle(obstacle(obst2_edges), 0)
euc_map.plotMap(1)
set(gcf,'Color','w')
title("Obstacles in Euclidean Space")
xlabel("x")
ylabel("y")

%%
% Create corresponding C-Space euc_map
cmap_sz = [360 360];
c_map = obstacle_map(cmap_sz);

for e_obstacle = euc_map.obstacles
    c_obstacle = convert_eObst_to_c_Obst(e_obstacle, link_lengths, gridx, gridy);
    c_map.add_obstacle(c_obstacle, 1);
end

c_map.plotMap(0)
set(gcf,'Color','w')
title("Obstacles in C-Space")
xlabel("Shoulder Angle (deg)")
ylabel("Elbow Angle (deg)")

%%
% Create and plot field
env_field = env_pot_field(cmap_sz);
for obstacle = c_map.obstacles
    env_field.add_obstacle(obstacle);
end

env_field.update_pot_field();
xlabel("Shoulder Angle (deg)")
ylabel("Elbow Angle (deg)")

%%
env_field.plot_sdf()
xlabel("Shoulder Angle (deg)")
ylabel("Elbow Angle (deg)")

env_field.plot_field()
xlabel("Shoulder Angle (deg)")
ylabel("Elbow Angle (deg)")

%%
% Create user intent map
intent = user_intent_file_wrapper(0.8 * ones(4,1), [30, 30], "Prelim_EMG_data.xlsx")

%% Create video for EMG Potential Field
vw = VideoWriter('video3.mp4', 'MPEG-4');
vw.Quality = 90;
vw.FrameRate = 25;
open(vw);

config = [125, 50];
plnr = planner();
gradient_map_x = zeros(size(intent.intent_field.field));
gradient_map_y = zeros(size(intent.intent_field.field));

figure(6)

pause(2)

for i=1:100
    i
    intent.step()
    net_field = intent.intent_field.field + env_field.field;
    config = plnr.descend_grad(config, net_field);
    
%     subplot(1,3,1) %comment both these sections out for 3d view, include
%     for 2d view
%     contourf(intent.intent_field.field, -150:1.5:150)
%     title("EMG Potential Field")
%     xlabel("Shoulder Angle (deg)")
%     ylabel("Elbow Angle (deg)")
% 
%     subplot(1,3,2)
%     contourf(env_field.field)
%     title("Environment Potential Field")
%     xlabel("Shoulder Angle (deg)")
%     ylabel("Elbow Angle (deg)")

    round_config=round(config);

    if round_config(1)>359
        round_config(1)=359;
    end

    if round_config(2)>359
        round_config(2)=359;
    end

    if round_config(1)<1
        round_config(1)=1;
    end

    if round_config(2)<1
        round_config(2)=1;
    end
    
    %subplot(1,3,3) %for 3d view, comment this out, include for 2d view
    surf(net_field,'EdgeColor','none') %for 2d view, change this to contourf and remove edge color none and add -150:1.5:150
    hold on
    set(gcf,'Color','w')

    view(i*720/1000,30) %for 2d view, comment this out
    camva(11) %for 2d view, comment this out
    xlim([0 360]) %for 2d view, comment this out
    ylim([0 360]) %for 2d view, comment this out
    zlim([-100 60]) %for 2d view, comment this out

    plot3(config(1), config(2),net_field(round_config(2),round_config(1)),'r.', 'MarkerSize', 30) %for 2d view, change plot3 to plot and remove z param net_field(round_config(2),round_config(1))
    title("Combined Potential Field")
    xlabel("Shoulder Angle (deg)")
    ylabel("Elbow Angle (deg)")
    zlabel('Potential')
    writeVideo(vw, getframe(gcf));
    hold off
    
    drawnow
    
%     for i=1:size(net_field,1)-5
%         for j=1:size(net_field,2)-5
%             grad = plnr.get_grad([i, j], net_field);
%             gradient_map_x(i,j) = grad(1);
%             gradient_map_y(i,j) = grad(2);
%         end
%     end
%     
%     figure(7)
%     contourf(gradient_map_x)
%     
%     figure(8)
%     contourf(gradient_map_y)
%     
end

vw.close();

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
    t1 = atan2d(y, x) - atan2d( a2*sind(t2), a1 + a2*cosd(t2) );
    
%     t1 = mod(t1, 360);
%     t2 = mod(t2, 360);
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
    
    if isempty(c_obst_x)
        return
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