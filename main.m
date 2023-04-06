clc; clear; close all;

%%
% Create robot parameters
link_lengths = [80, 80]; % Robot link lengths

%%
% Create a euc_map for Euclidean Space
sz = [200, 200]
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
c_map = obstacle_map(sz);

for e_obstacle = euc_map.obstacles
    e_edges = e_obstacle.edges;
    c_edges = zeros(size(e_edges));
    
    for i = 1:size(e_edges,1)
        euc_edge = e_edges(i,:);
        euc_pt1 = [euc_edge(1) euc_edge(2)];
        euc_pt2 = [euc_edge(3) euc_edge(4)];
        
%         testFxn(1)
        
        c_pt1 = convert_Euclidean_to_Config(euc_pt1, link_lengths);
        c_pt2 = convert_Euclidean_to_Config(euc_pt2, link_lengths);
        
        c_edges(i,:) = [c_pt1, c_pt2];
    end
    
    c_obstacle = obstacle(c_edges);
    c_map.add_obstacle(c_obstacle);
end

c_map.plotMap()

%%
env_field = env_pot_field()