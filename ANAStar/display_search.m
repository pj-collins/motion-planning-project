% Adjustments were made to this file in order to properly read in the
% output path file in the case that the NO PATH FOUND error is printed in
% output_path.txt

% Copyright 2018, Michael Otte
%
% Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
%
%The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
%
%THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% this will display the search tree and path
% assuming that the files have been generated


search_tree_raw = csvread('search_tree.txt');

% If output_path.txt has NO PATH FOUND in last line, read data in as a
% table and remove the last row
try
    path_raw = csvread('output_path.txt');
catch exception
    disp("No Path found, extracting data with tables.")
    path_raw = readtable('output_path.txt','Delimiter',',');
    path_raw = table2array(path_raw(1:end-1,1:3));
end

nodes_raw = csvread('files/nodes.txt');
edges_raw = csvread('files/edges_with_costs.txt');
obstacles_raw = csvread('files/obstacles.txt');

% a bit of data processing for faster plotting
search_tree = nan(3*size(search_tree_raw, 1), 2);

search_tree(1:3:end-2, 1) = search_tree_raw(:, 2);
search_tree(2:3:end-1, 1) = search_tree_raw(:, 5);
search_tree(1:3:end-2, 2) = search_tree_raw(:, 3);
search_tree(2:3:end-1, 2) = search_tree_raw(:, 6);

nodes = nodes_raw(2:end,2:3);

edges_raw = edges_raw(2:end,:);

edges = nan(3*size(edges_raw, 1), 2);

edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);

obstacles = obstacles_raw(2:end,:);

figure(1)
plot(nodes(:,1), nodes(:,2), 'ok', 'MarkerFaceColor', 'k', 'MarkerSize', 3)
hold on
plot(edges(:,1), edges(:,2), 'k')
plot(search_tree(:, 1), search_tree(:, 2), 'm', 'LineWidth', 2);
plot(path_raw(:,2), path_raw(:,3), 'g:', 'LineWidth', 3);
for i = 1:size(obstacles,1)
    rectangle("Position",obstacles(i,:),'EdgeColor','r')
    hold on
end
hold off
