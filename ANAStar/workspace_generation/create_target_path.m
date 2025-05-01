%% Script the loads a workspace and gives the user prompts to create a target path

close all
clear all

%% Parameters

create_files = true;
file_name = "target_path_5";

%% Load variables

nodes_raw = csvread("workspace_files\nodes.txt");
edges_raw = csvread("workspace_files\edges_with_costs.txt");
obstacles_raw = csvread("workspace_files\obstacles.txt");

% Truncate first row of each matrix
nodes = nodes_raw(2:end,:);
edges = edges_raw(2:end,:);
obstacles = obstacles_raw(2:end,:);

% Set workspace limits (2D euclidean space)
params.x_min = -50;    params.x_max = 50;     
params.y_min = -50;    params.y_max = 50;     
params.x_range = params.x_max - params.x_min; 
params.y_range = params.y_max - params.y_min;

% Plot the graph for user visuals
% Plot the workspace and nodes
fig1 = figure;
title('Workspace, Nodes, and Target Path')
xlim([params.x_min, params.x_max]);
ylim([params.y_min, params.y_max]);
for i = 1:size(obstacles,1)
    rectangle("Position",obstacles(i,:),'EdgeColor','r')
    hold on
end
scatter(nodes(:,2),nodes(:,3), 10, 'filled');

% Prompt the user for start node inputs
x_sel = input(sprintf("Select an x-coordinate in range [%d, %d] to start at: ", params.x_min, params.x_max));
y_sel = input(sprintf("Select a y-coordinate in range [%d, %d] to start at: ", params.y_min, params.y_max));

% Define the start node ID
start_node_ID = dsearchn(nodes(:,2:3),[x_sel, y_sel]);

% Begin by defining the target path, each step defined as
% [nodeID, x_pos, y_pos, last_cost, total_cost]
cost = 0;
dist = 0;
target_path = [nodes(start_node_ID,:), cost, dist];
continue_path = true;
current_node_ID = start_node_ID;

% Plot the start node
scatter(nodes(start_node_ID,2), nodes(start_node_ID,3), 50, 'square', 'm', 'filled')


while continue_path
    % Identify the edges connected to the current node
    possible_edge_IDs =  find(edges(:,1) == current_node_ID);
    current_node_x = nodes(current_node_ID,2);
    current_node_y = nodes(current_node_ID,3);

    % Print the current node locations
    fprintf("\n\nCurrent Location [%.2f, %.2f]\n", current_node_x, current_node_y)
    fprintf("Connected Locations:\n")

    % Iterate through all connected nodes, printing their locations
    for i = 1:length(possible_edge_IDs)
        node_ID = edges(possible_edge_IDs(i),2);
        node_x = nodes(node_ID, 2);
        node_y = nodes(node_ID, 3);
        fprintf("(%d) [%.2f, %.2f], moves [%.2f, %.2f]\n",i, node_x, node_y, node_x - current_node_x, node_y - current_node_y);
    end
    fprintf("(%d) End Path", 9);

    % Prompt user for input
    selection = input("\n\nSelect an Option: ");

    % Add new nodes to target path, or end the exercise
    if selection == 9
        continue_path = false;
        break;
    else
        % Create a new line in the target path matrix
        current_node_ID = edges(possible_edge_IDs(selection),2);
        cost = edges(possible_edge_IDs(selection),3);
        dist = dist + cost;
        new_path_line = [nodes(current_node_ID,:), cost, dist];
        target_path = [target_path; new_path_line];

        % Plot the new path additions
        plot(target_path(:,2), target_path(:,3), 'm', LineWidth=1.5)
    end
end

% Plot the end point of the target
scatter(nodes(current_node_ID,2), nodes(current_node_ID,3), 50, 'square', 'k', 'filled')

% Define the number of nodes in the output path
num_nodes = size(target_path,1);

% Save files
if create_files
    % Create the node file
    fileID = fopen(sprintf("workspace_files/%s.txt", file_name),"w");
    fprintf(fileID, '%d\n', num_nodes);
    for i = 1:num_nodes
        fprintf(fileID,'%d, %.4f, %.4f, %.4f, %.4f\n', target_path(i,:));
    end
    fclose(fileID);
end
