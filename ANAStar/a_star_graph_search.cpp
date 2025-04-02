/*
Copyright 2018, Michael Otte

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>


#include "heap.h"
#include "heap.cpp"

#include "graph.h"

// returns random number between 0 and 1
float rand_f()
{
  return (float)rand() / (float)RAND_MAX;
}

// computes the euclidean distance between two nodes
double heurisitic_func(Node* thisNode, Node* goalNode)
{
  double x1 = thisNode->x;
  double x2 = goalNode->x;
  double y1 = thisNode->y;
  double y2 = goalNode->y;
  double dist = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
  return dist;
}


// A star expansion
void expand(Heap<Node> &H, Node* thisNode, Node* goalNode)
{
  for(int n = 0; n < thisNode->numOutgoingEdges; n++)
  {
    Edge* thisEdge = thisNode->outgoingEdges[n];  // pointer to this edge
    Node* neighborNode = thisEdge->endNode;  // pointer to the node on the 
                                             // other end of this edge

    // neighbor has not yet been visited or its cost_to_start is wrong 
    if(neighborNode->status == 0 || neighborNode->cost_to_start > (thisNode->cost_to_start + thisEdge->edgeCost)) 
    {
      // redefine the cost to start for the neighbor
      neighborNode->cost_to_start = (thisNode->cost_to_start + thisEdge->edgeCost);

      // add the neighbor to the heap (with an A* key)
      double neighborKey = neighborNode->cost_to_start + heurisitic_func(neighborNode, goalNode);
      H.updateNodeInHeap(neighborNode, neighborKey);
      
      // remeber this node as its parent
      neighborNode->parentNode = thisNode;

      // make sure it is in the open list
      neighborNode->status = 1;
    }
  }

  thisNode->status = 2;    // now this node is in the closed list
}



int main()
{

  Graph G;
  G.readGraphFromFiles("files/nodes.txt", "files/edges_with_costs.txt");
//  G.printGraph();

  // we want to find a path that goes from here to here
  int startNodeIndex = 430;
  int goalNodeIndex = 1740;

  // Use ID - 1 when setting the start and end nodes
  startNodeIndex--;
  goalNodeIndex--;


  Heap<Node> H(5000); // this is the heap (start's with space for 200 items
                     // but will grow automatically as needed). Adjusted from 100.


  // these are pointers to the start and end nodes
  Node* startNode = &G.nodes[startNodeIndex];
  Node* goalNode = &G.nodes[goalNodeIndex];


  // For A*, the keys assigned in the heap to each node are a function of the
  // current known cost to get from the start node to the current node, plus a
  // heuristic estimate to get from the current node to the goal node.
  // The heuristic used is the euclidean distance, as in this use case the 
  // euclidean distance is always an admissible heurisitic.

  double key = heurisitic_func(startNode, goalNode);
  H.addToHeap(startNode, key);
  startNode->status = 1;          // now the start node is in the open list
  startNode->cost_to_start = 0;   // the start node's cost to start is set to 0

  // while there are nodes left in the heap
  // (note that a better stopping criteria should be used in
  // most "real" algorithms, but this will cause us to generate
  // a random spanning tree over the graph, which is kind of cool)
  while(H.topHeap() != NULL)
  {
    Node* thisNode = H.popHeap();
    expand(H, thisNode, goalNode);

    // end the search if we pop the goal node (the optimal path has been found)
    if(thisNode == goalNode)
    {
      break;
    }
    // H.printHeap();
  }
  
  // now we want to save files that contain the search tree that we build
  // and also the best path that we found (NOTE, these add 1 to indicies
  // to make them compativle with the graph file that was used as input for
  // the search)

  G.savePathToFile("output_path.txt", goalNode, startNode);
  G.saveSearchTreeToFile("search_tree.txt");


  return 0;
}
