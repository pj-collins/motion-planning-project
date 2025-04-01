% Copyright 2018, Michael Otte
%
% Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

% this loads the graph from graphName_nodes.txt and graphName_edges.txt
% and plots it

% node file:
% assumes first line contains the number of nodes
% lines 2 to n+1 contain nodes 1 to n, respectively
% each node is given by ID, x_position, y_position

% edge file:
% assumes first line contains the number of edges in the file
% lines 2 to m+1 contain nodes 1 to m, respectively
% each edges is given by ID_start, ID_end, edgeLength 


nodeFile = 'files/nodes_4.txt';
edgeFile = 'files/edges_with_costs_4.txt';

startNodeID = 723;
goalNodeID = 926;

nodes = csvread(nodeFile);
edges = csvread(edgeFile);


% make things easy for matlab to plot
numNodes = nodes(1,1);
numEdges = edges(1,1);

nodes = nodes(2:end,:);   % remove node count (first row)
edges = edges(2:end,:);   % remove edge count (first row)

nodesForPlotting = nodes(2:end,2:3);
edgesForPlotting = nan(3*numEdges,2);



edgesForPlotting(1:3:end-2,1:2) = nodes(edges(:,1),2:3);  % edge starts
edgesForPlotting(2:3:end-1,1:2) = nodes(edges(:,2),2:3);  % edge ends

figure(2)
plot(edgesForPlotting(:,1),edgesForPlotting(:,2))
hold on
plot(nodesForPlotting(:,1),nodesForPlotting(:,2), 'ok')
plot(nodes(startNodeID,2) , nodes(startNodeID,3), 'or', 'markersize', 10, 'linewidth', 2)
plot(nodes(goalNodeID,2) , nodes(goalNodeID,3), 'xr', 'markersize', 10, 'linewidth', 2)
hold off




