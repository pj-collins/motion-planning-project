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

#include <chrono>
#include <thread>

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
void aStar(Heap<Node> &H, Node* thisNode, Node* goalNode)
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

  thisNode->status = 3;    // now this node is in the closed list
}


// ANA star expansion
void anaStar(Heap<Node> &H, Node* thisNode, Node* goalNode)
{
  for(int n = 0; n < thisNode->numOutgoingEdges; n++)
  {
    Edge* thisEdge = thisNode->outgoingEdges[n];  // pointer to this edge
    Node* neighborNode = thisEdge->endNode;  // pointer to the node on the 
                                             // other end of this edge

    // Neighbor has not yet been visited or its cost_to_start is wrong 
    if(neighborNode->status == 0 || neighborNode->cost_to_start > (thisNode->cost_to_start + thisEdge->edgeCost)) 
    {
      // Remember this node as its parent
      neighborNode->parentNode = thisNode;

      // Redefine the cost to start for the neighbor
      neighborNode->cost_to_start = (thisNode->cost_to_start + thisEdge->edgeCost);

      // If the neighbor node is closed, put in in the inconsistent set
      if(neighborNode->status == 3)
      {
        neighborNode->status = 2;
      }
      // Otherwise, update the neighbor's key in the heap and add it to the open list
      else
      {
        double neighborKey = (goalNode->cost_to_start - neighborNode->cost_to_start)/heurisitic_func(neighborNode, goalNode);
        H.updateNodeInHeap(neighborNode, neighborKey);
        neighborNode->status = 1;
      }
      
    }
  }
}



int main()
{

  Graph G;
  G.readGraphFromFiles("files/nodes.txt", "files/edges_with_costs.txt");
  //G.printGraph();

  // Define timer values for ANA* to run
  // Set the duration for the timer
  //auto duration = std::chrono::milliseconds(1);
  auto duration = std::chrono::milliseconds(100);

  // we want to find a path that goes from here to here
  int startNodeIndex = 4500;
  int goalNodeIndex = 25042;

  // Use ID - 1 when setting the start and end nodes
  startNodeIndex--;
  goalNodeIndex--;


  Heap<Node> H(40000); // this is the heap (start's with space for 200 items
                     // but will grow automatically as needed). Adjusted from 100.


  // these are pointers to the start and end nodes
  Node* startNode = &G.nodes[startNodeIndex];
  Node* goalNode = &G.nodes[goalNodeIndex];


  // Add the start node to the heap, set it to an open status
  double key = heurisitic_func(startNode, goalNode);
  H.addToHeap(startNode, key);
  startNode->status = 1;          // now the start node is in the open list
  startNode->cost_to_start = 0;   // the start node's cost to start is set to 0

  int counter = 0;

  // Get the starting time
  auto start = std::chrono::steady_clock::now();

  while(std::chrono::steady_clock::now() - start < duration)
  {
    Node* topNode = H.topHeap(); 

    while(topNode != NULL && topNode->cost_to_start < goalNode->cost_to_start)
    {
      Node* thisNode = H.popHeap(); // pop a node off the heap
      thisNode->status = 3;         // add this node to the closed set

      anaStar(H, thisNode, goalNode);

      topNode = H.topHeap(); 

    }

    // Increment counter
    counter++;

    // Empty the heap by deleting it and reinitializing it
    H.deleteHeap();
    Heap<Node> H(40000);

    // Reassign nodes to different sets
    for(int i = 0; i < G.numNodes; i++)
    {
      Node* thisNode = &G.nodes[i];
      
      // If a node is in the inconsistent set, put it in the open set
      if(thisNode->status == 2)
      {
        thisNode->status = 1;
      }
      // Else if node is in closed set, put it in unvisited set
      else if(thisNode->status == 3)
      {
        thisNode->status = 0;
      } 

      // If a node is open, add it to the heap
      if(thisNode->status == 1)
      {
        key = (goalNode->cost_to_start - thisNode->cost_to_start)/heurisitic_func(thisNode, goalNode);
        H.addToHeap(thisNode, key);
      }
    }
  }

  printf("Iterations completed: %d\n", counter);

  /*
  // Astar implementation, commented out but kept for reference
  while(H.topHeap() != NULL)
  {
    Node* thisNode = H.popHeap();
    aStar(H, thisNode, goalNode);

    // end the search if we pop the goal node (the optimal path has been found)
    if(thisNode == goalNode)
    {
      break;
    }
    // H.printHeap();
  }
  */

  H.deleteHeap();

  G.savePathToFile("output_path.txt", goalNode, startNode);
  G.saveSearchTreeToFile("search_tree.txt");


  return 0;
}
