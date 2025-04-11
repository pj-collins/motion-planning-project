% File that reads in a single instance of path planning and plots relevant
% information

clear all
close all

search_tree_raw = csvread('files/search_tree.txt');

% If output_path.txt has NO PATH FOUND in last line, read data in as a
% table and remove the last row
try
    path_raw = csvread('files/output_path.txt');
catch exception
    disp("No Path found, extracting data with tables.")
    path_raw = readtable('files/output_path.txt','Delimiter',',');
    path_raw = table2array(path_raw(1:end-1,1:3));
end

nodes_raw = csvread('files/nodes.txt');
edges_raw = csvread('files/edges_with_costs.txt');
obstacles_raw = csvread('files/obstacles.txt');
path_search_data_raw = csvread('files/path_search_data.txt');

% a bit of data processing for faster plotting
search_tree = nan(3*size(search_tree_raw, 1), 2);
search_tree(1:3:end-2, 1) = search_tree_raw(:, 2);
search_tree(2:3:end-1, 1) = search_tree_raw(:, 5);
search_tree(1:3:end-2, 2) = search_tree_raw(:, 3);
search_tree(2:3:end-1, 2) = search_tree_raw(:, 6);

% Process node data
nodes = nodes_raw(2:end,2:3);

% Process edge data
edges_raw = edges_raw(2:end,:);
edges = nan(3*size(edges_raw, 1), 2);
edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);

% Process obstacle data
obstacles = obstacles_raw(2:end,:);

% Process the path search data
path_search.elapsed_ms = path_search_data_raw(1);
path_search.iterations = path_search_data_raw(2);
path_search.best_cost = path_search_data_raw(3:end);

% Plot the search path
figure(1)
%plot(nodes(:,1), nodes(:,2), 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', 3)
hold on
%plot(edges(:,1), edges(:,2), 'k')
plot(search_tree(:, 1), search_tree(:, 2), 'm', 'LineWidth', 2);
plot(path_raw(:,2), path_raw(:,3), 'g:', 'LineWidth', 3);
scatter(path_raw(1,2),path_raw(1,3), 100, 'square', 'b', 'filled');
scatter(path_raw(end,2),path_raw(end,3), 100, 'square', 'k', 'filled');
for i = 1:size(obstacles,1)
    rectangle("Position",obstacles(i,:),'EdgeColor','r')
    hold on
end
hold off

% Plot the cost of best path found over each ANA* iteration
figure(2)
plot(path_search.best_cost, 'LineWidth', 1.25)
hold on, grid on
plot(0:path_search.iterations+1, ones(path_search.iterations+2,1)*path_search.best_cost(end),'--k')
xlabel('Iteration')
ylabel('Cost of Best Path Found')
title(sprintf("Cost of Best Path to Goal Found by ANA* over %.2f ms", path_search.elapsed_ms));
legend('Cost of Best Path Found to Goal','Cost of Best Path Found for Last Iteration')

