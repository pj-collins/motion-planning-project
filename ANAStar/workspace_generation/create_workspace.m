%% Script to create a workspace

clear all
close all

%% Parameters

% Plot edges if desired (can take awhile)
edge_plot_flag = true;

% Create edge and node txt files
create_files = true;

%% Define workspace obstacles

% Set workspace limits (2D euclidean space)
params.x_min = -50;    params.x_max = 50;     
params.y_min = -50;    params.y_max = 50;     
params.x_range = params.x_max - params.x_min; 
params.y_range = params.y_max - params.y_min;

% Define rectangular obstacles as 4-element vector v = [x1, x2, y1, y2]
% where x2 > x1, y2 > y1
num_obs = 7;
obstacles = zeros(num_obs, 4);

% Obstacles
obstacles(1,:) = add_obstacle(0.1,0.3,0.2,0.3,params);
obstacles(2,:) = add_obstacle(0.3,0.4,0.05,0.3,params);
obstacles(3,:) = add_obstacle(0.05,0.2,0.8,0.95,params);
obstacles(4,:) = add_obstacle(0.75,0.9,0.2,0.65,params);
obstacles(5,:) = add_obstacle(0.2,0.4,0.55,0.7,params);
obstacles(6,:) = add_obstacle(0.6,0.7,0,0.4,params);
obstacles(7,:) = add_obstacle(0.6,0.7,0.5,1,params);

% Plot the workspace
fig1 = figure;
title('Workspace')
xlim([params.x_min, params.x_max]);
ylim([params.y_min, params.y_max]);
for i = 1:size(obstacles,1)
    rectangle("Position",obstacles(i,:),'EdgeColor','r')
    hold on
end

%% Define the valid node

% Define node resolution and set of nodes to investigate
node_resolution = 2;
x_set = params.x_min:node_resolution:params.x_max;
y_set = params.y_min:node_resolution:params.y_max;

% Define empty node set
nodes = [];
id = 1;

% Iterate through all possible nodes, checking for collisions with
% obstacles
for i = 1:length(x_set)
    for j = 1:length(y_set)
        if ~check_collisions(x_set(i),y_set(j),obstacles)
            new_node = [id, x_set(i), y_set(j)];
            nodes = [nodes; new_node];
            id = id + 1;
        end
    end
end

% Plot the workspace and nodes
fig2 = figure;
title('Workspace and Graph')
xlim([params.x_min, params.x_max]);
ylim([params.y_min, params.y_max]);
for i = 1:size(obstacles,1)
    rectangle("Position",obstacles(i,:),'EdgeColor','r')
    hold on
end
scatter(nodes(:,2),nodes(:,3), 10, 'filled');

% Define number of nodes
num_nodes = size(nodes,1);

%% Define the valid edges

% Define empty edge set [parent ID, child ID, cost]
edges = [];

% Find the indices of nodes within some radius of any node
buffer = 0.01;
r = node_resolution*sqrt(2) + buffer*node_resolution;   % within sqrt(2) plus a tiny buffer
[neighbors,dist] = rangesearch(nodes(:,2:3),nodes(:,2:3),r);

% Iterate through all nodes and define edges
for i = 1:num_nodes
    for j = 2:size(neighbors{i},2)
        cost = dist{i}(j) * (1 + 0.2*rand);     % cost is lower bounded by distance, up to 1.2 * dist
        new_edge = [i, neighbors{i}(j), cost];
        edges = [edges; new_edge];
    end
end

% Define number of edges
num_edges = size(edges,1);

if edge_plot_flag
    % Define an edge position matrix for plotting 
    edge_plotting = [];
    for i = 1:num_edges
        id1 = edges(i,1);
        id2 = edges(i,2);
        new_edge_plotting = [nodes(id1,2:3); nodes(id2,2:3)];
        edge_plotting = [edge_plotting; new_edge_plotting];
        plot(new_edge_plotting(:,1),new_edge_plotting(:,2), 'k');
    end
end

%% Create edge and node files

if create_files
    % Create the node file
    fileID = fopen("nodes.txt","w");
    fprintf(fileID, '%d\n', num_nodes);
    for i = 1:num_nodes
        fprintf(fileID,'%d, %.4f, %.4f\n', nodes(i,:));
    end
    fclose(fileID);

    % Create the edge file
    fileID = fopen("edges_with_costs.txt","w");
    fprintf(fileID, '%d\n', num_edges);
    for i = 1:num_edges
        fprintf(fileID,'%d, %d, %.4f\n', edges(i,:));
    end
    fclose(fileID);

    % Create the obstacles files
    fileID = fopen("obstacles.txt","w");
    fprintf(fileID, '%d\n', num_obs);
    for i = 1:num_obs
        fprintf(fileID,'%.4f, %.4f, %.4f, %.4f\n', obstacles(i,:));
    end
    fclose(fileID);

end

%% Helper Functions

% Function that defines two vertices of a rectangle
function obs = add_obstacle(x_val_1, x_val_2, y_val_1, y_val_2, params)
    % Define matrix of zeros
    obs = zeros(1,4);

    % Populate bottom left vertex of rectangle
    obs(1,1:2) = [params.x_min + x_val_1*params.x_range, params.y_min + y_val_1*params.y_range];

    % Populate the width and height inputs of the rectangle
    obs(1,3:4) = [params.x_range*(x_val_2 - x_val_1), params.y_range*(y_val_2 - y_val_1)];
end

% Function that checks if a point is within an obstacle (closed set)
function flag = check_collisions(x, y, obstacles)
    % Initially set flag to false
    flag = false;
    buffer = 0.01;

    % Iterate through each obstacle
    for i = 1:size(obstacles,1)
        % Extract obstacle information
        x1 = obstacles(i,1);
        y1 = obstacles(i,2);
        w = obstacles(i,3);
        h = obstacles(i,4);

        % If position coordinates are within rectangle
        if x + buffer >= x1 && x - buffer <= x1 + w
            if y + buffer >= y1 && y - buffer <= y1 + h
                flag = true;
                break;
            end
        end
    end

end

