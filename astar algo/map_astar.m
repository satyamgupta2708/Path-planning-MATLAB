%% Define a small map
map = false(10);

% Add an obstacle
map (1:5, 6) = true;
map (6,8:10) = true;
map (3:7,2) = true;
%%
start_coords = [1,3];
dest_coords  = [1,9];


[route, numExpanded ,cost_matrix] = AStarGrid(map, start_coords, dest_coords,true);
pause(3);

%%
start_coords = [10,1];
dest_coords  = [2,9];


[route, numExpanded ,cost_matrix] = AStarGrid(map, start_coords, dest_coords,true);

pause(3);

%%

start_coords = [6,3];
dest_coords  = [8,9];

[route, numExpanded ,cost_matrix] = AStarGrid (map, start_coords, dest_coords,true);
pause(3);

%%


start_coords = [6,1];
dest_coords  = [10,10];

[route, numExpanded ,cost_matrix] = AStarGrid(map, start_coords, dest_coords,true);
pause(3);

%%
start_coords = [6,1];
dest_coords  = [1,10];

[route, numExpanded ,cost_matrix] = AStarGrid(map, start_coords, dest_coords,true);
pause(3);





