clc; clear; close all;

%%
% Create a map and potential field
sz = [200, 200]
map = euclidean_map(sz);

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

map.add_obstacle(obstacle(obst1_edges))
map.plotMap()


%%
% figure
% hold on

% for i = 1:size(obst1_edges, 2)
%     edge = obst1_edges(i,:)
%     plot([edge(1), edge(3)], [edge(2), edge(4)], 'k-')
% end

%%
env_field = env_pot_field()