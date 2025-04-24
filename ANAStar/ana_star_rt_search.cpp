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
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "heap.h"
#include "heap.cpp"

#include "graph.h"
#include "target.h"

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
      float neighborKey = neighborNode->cost_to_start + heurisitic_func(neighborNode, goalNode);
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
void anaStar(Heap<Node> &H, Node* thisNode, Node* goalNode, double goalCost)
{
  for(int n = 0; n < thisNode->numOutgoingEdges; n++)
  {
    Edge* thisEdge = thisNode->outgoingEdges[n];  // pointer to this edge
    Node* neighborNode = thisEdge->endNode;  // pointer to the node on the 
                                             // other end of this edge

    // Neighbor has not yet been visited or its cost_to_start is wrong 
    //if(neighborNode->status == 0 || neighborNode->cost_to_start > (thisNode->cost_to_start + thisEdge->edgeCost)) 
    if(neighborNode->status == 0 || neighborNode->cost_to_start > (thisNode->cost_to_start + thisEdge->edgeCost))
    {      
      // Remember this node as its parent
      neighborNode->parentNode = thisNode;

      // Redefine the cost to start for the neighbor
      neighborNode->cost_to_start = (thisNode->cost_to_start + thisEdge->edgeCost);

      if(neighborNode->cost_to_start + heurisitic_func(neighborNode,goalNode) < goalCost)
      {
        float neighborKey = (goalCost - goalNode->cost_to_start)/heurisitic_func(neighborNode, goalNode);
        H.updateNodeInHeap(neighborNode, 1/neighborKey);
        neighborNode->status = 1;
      }
      
      // If the neighbor node is closed, put in in the inconsistent set
      if(neighborNode->status == 3)
      {
        neighborNode->status = 2;
      }
      // Otherwise, update the neighbor's key in the heap and add it to the open list
      else
      {
        float neighborKey = (goalCost - neighborNode->cost_to_start)/heurisitic_func(neighborNode, goalNode);
        H.updateNodeInHeap(neighborNode, 1/neighborKey);
        neighborNode->status = 1;
      }
      
    }
  }
}


// ANA star expansion
void anaStar_simple(Heap<Node> &H, Node* thisNode, Node* goalNode, double goalCost)
{
  for(int n = 0; n < thisNode->numOutgoingEdges; n++)
  {
    Edge* thisEdge = thisNode->outgoingEdges[n];  // pointer to this edge
    Node* neighborNode = thisEdge->endNode;  // pointer to the node on the 
                                             // other end of this edge

    // Neighbor has not yet been visited or its cost_to_start is wrong 
    //if(neighborNode->status == 0 || neighborNode->cost_to_start > (thisNode->cost_to_start + thisEdge->edgeCost)) 
    if(neighborNode->cost_to_start > (thisNode->cost_to_start + thisEdge->edgeCost))
    {      
      // Redefine the cost to start for the neighbor
      neighborNode->cost_to_start = (thisNode->cost_to_start + thisEdge->edgeCost);

      // Remember this node as its parent
      neighborNode->parentNode = thisNode;

      if(neighborNode->cost_to_start + heurisitic_func(neighborNode,goalNode) < goalCost)
      {
        float neighborKey = (goalCost - neighborNode->cost_to_start)/heurisitic_func(neighborNode, goalNode);
        H.updateNodeInHeap(neighborNode, 1/neighborKey);
        neighborNode->status = 1;
      }
      
    }
  }
}


// Function that saves path search data to a file
bool savePathSearchDataToFile(const char* pathFile, std::vector<double> bestSolutionCosts, double searchTime)
{
  // Open file, confirm it is valid
  FILE * pFile = fopen(pathFile,"w");      
  if(pFile == NULL)
  {
    return false;
  }

  // Pull the number of search iterations out 
  int iterations = bestSolutionCosts.size();

  // Bring the elapsed search time to the file
  fprintf(pFile, "%lf\n", searchTime);
  
  // Print the number of iterations to the file
  fprintf(pFile, "%d\n", iterations);

  // Print the best solution cost (goal's cost to start) for each iteration
  for(int i = 0; i < iterations; i++)
  {
    fprintf(pFile, "%lf\n", bestSolutionCosts[i]);
  }

  fclose(pFile);
  printf("Saved path search data in %s.\n\n", pathFile);

  return true;
}


int main()
{

  Graph G;
  G.readGraphFromFiles("files/nodes.txt", "files/edges_with_costs.txt");
  //G.printGraph();

  // Define the test config
  int testConfig = 4;
  bool intercept = true;

  // Initialize config values
  double agentVelocity = 6;
  double targetVelocity = 3;
  int planningTimeMS = 100;
  int startNodeID = 0;
  float target_range = 1;

  // Define and open config file
  string config_filename = "files/test" + to_string(testConfig) + "/config_" + to_string(testConfig) + ".txt";
  string target_path_filename = "files/test" + to_string(testConfig) + "/target_path_" + to_string(testConfig) + ".txt";
  ifstream inputFile(config_filename);
  string label;

  if(inputFile.is_open())
  {
    std::getline(inputFile, label, ':');
    inputFile >> agentVelocity;

    std::getline(inputFile, label, ':');
    inputFile >> targetVelocity;

    std::getline(inputFile, label, ':');
    inputFile >> planningTimeMS;

    std::getline(inputFile, label, ':');
    inputFile >> startNodeID;

    std::getline(inputFile, label, ':');
    inputFile >> target_range;

    inputFile.close(); 
  }
  else
  {
    printf("Error reading configuation file. Configurations set to default values.\n");
  }

  printf("Configurations:\nAgent Velocity = %.2f\nTarget Velocity = %.2f\nPlanning Time = %d ms\n\n", agentVelocity, targetVelocity, planningTimeMS);

  // Define output folder
  string output_folder = "files/test" + to_string(testConfig) + "/output/";

  // Define the target path object
  TargetPath TP;
  TP.readTargetPathFromFile(target_path_filename.c_str());
  TP.setTargetVelocity(targetVelocity);

  // Define timer values for ANA* to run
  // Set the duration for the timer
  auto duration = chrono::milliseconds(planningTimeMS);

  // Bool value that determines if search is ended
  bool endSearch = false;
  string output_path_filename;
  string path_search_data_filename;
  string output_subfolder;

  // Test subfolder name
  if(intercept)
  {
    output_subfolder = "intercept";
  }
  else
  {
    output_subfolder = "no_intercept";
  }

  // we want to find a path that goes from here to here
  int goalNodeID = TP.currentTargetNode();
  
  // Use ID - 1 when setting the start and end nodes
  startNodeID--;
  goalNodeID--;

  int time_step = 0;
  bool path_found = false;

  Heap<Node> H(40000);
  //Heap<Node> H(40000); // this is the heap (start's with space for 200 items

  // These are pointers to the start and end nodes
  Node* startNode = &G.nodes[startNodeID];
  Node* goalNode = &G.nodes[goalNodeID];
  float dist_from_target = 200;

  int execHorizon = 2;       // how many agent‐steps to follow one plan
  int execCounter = 0;       // how many we’ve executed so far
  bool needNewInterceptGoal = true;    // true means to recompute the goal this iteration

  while(!endSearch)
  {

    // but will grow automatically as needed). Adjusted from 100.

    // Add the start node to the heap, set it to an open status
    startNode->status = 1;          // now the start node is in the open list
    startNode->cost_to_start = 0;   // the start node's cost to start is set to 0
    double goalCost = goalNode->cost_to_start;
    std::vector<double> bestSolutionsCosts;
    double key = (goalCost - startNode->cost_to_start)/heurisitic_func(startNode, goalNode);
    H.addToHeap(startNode, 1/key);

    // Define a counter variable
    int counter = 0;

    Node* topNode = H.topHeap(); 

    // Get the starting time
    auto start = std::chrono::steady_clock::now();

    while(std::chrono::steady_clock::now() - start < duration && topNode != NULL)
    {
      // IMPROVE SOLUTION
      while (topNode != NULL)
      {
        // Pop a node off the top of the heap
        Node* thisNode = H.popHeap(); // pop a node off the heap

        // If the node is the goal node, the optimal solution has been found
        if(thisNode->id == goalNode->id)
        {
          goalCost = goalNode->cost_to_start;
          break;
        }

        anaStar_simple(H, thisNode, goalNode, goalCost); 

        topNode = H.topHeap();
      }

      // Increment counter
      counter++;

      // Add goal cost to cost tracker vector
      bestSolutionsCosts.push_back(goalCost);

      // Reassign nodes to different sets
      for(int i = 0; i < H.indexOfLast + 1; i++)
      {
        Node* thisNode = H.heapNode[i];

        if(thisNode->cost_to_start + heurisitic_func(thisNode, goalNode) >= goalCost)
        {
          H.removeNodeFromHeap(thisNode);
        }
        else
        {
          key = (goalCost - thisNode->cost_to_start)/heurisitic_func(thisNode, goalNode);
          H.updateNodeInHeap(thisNode, 1/key);
        }

      }

      topNode = H.topHeap();

    }

    // Record elapsed time for search
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    double elapsed_ms = elapsed.count();
  
    // Print search info to terminal
    printf("Iterations completed: %d\n", counter);
    printf("Elapsed Time: %.2f ms\n\n", elapsed_ms);

    // Save data to files
    //G.savePathToFile("files/output_path.txt", goalNode, startNode);
    //G.saveSearchTreeToFile("files/search_tree.txt");
    //savePathSearchDataToFile("files/path_search_data.txt", bestSolutionsCosts, elapsed_ms);

    // Check if a path is found
    if(goalNode->parentNode != NULL)
    {
      path_found = true;
    }
    else
    {
      path_found = false;
    }

    output_path_filename = "files/test" + to_string(testConfig) + "/" + output_subfolder + "/output_paths/output_path_t" + to_string(time_step) + ".txt";
    G.savePathToFile(output_path_filename.c_str(), goalNode, startNode);

    path_search_data_filename = "files/test" + to_string(testConfig)+ + "/" + output_subfolder + "/path_search_data/path_search_data_t" + to_string(time_step) + ".txt";
    savePathSearchDataToFile(path_search_data_filename.c_str(), bestSolutionsCosts, elapsed_ms);


    // Find the reachable node for the agent, set the next start node ID
    if(path_found)
    {
      startNodeID = G.findReachableNode(goalNode, startNode, agentVelocity);
    }
    
    // Find the reachable node for the target, set the next target goal ID
    TP.stepForward();
    
    if(intercept)
    {
      // 1) if it’s time to replan our intercept goal:
      if (needNewInterceptGoal) 
      {
        // first compute distance from agent to last‐seen target position 
        Node* AN = &G.nodes[startNodeID];
        int  idx = TP.current_idx;
        double tx = TP.x[idx], ty = TP.y[idx];
        double d  = hypot(tx - AN->x, ty - AN->y);

        // choose predHorizon = ceil(time‐steps to reach last‐seen)
        // this means we choose the prediction horizon to be equal to the time required for the agent to reach the current target position 
        int predHorizon = max(1, (int)ceil(d / agentVelocity));

        // copy TP, step it forward predHorizon times (TODO: change this to non-cheating prediction)
        TargetPath TPcopy = TP;
        for(int i=0; i<predHorizon; ++i) 
        {
          TPcopy.stepForward();
        }
        double predX = TPcopy.x[TPcopy.current_idx]; 
        double predY = TPcopy.y[TPcopy.current_idx];

        // snap to nearest valid node (i.e., out of an obstacle if happened)
        int snapped = G.findClosestNode(predX, predY, agentVelocity*1.5);
        if (snapped < 0) 
        {
          snapped = TP.currentTargetNode();
        }
        // set that as our next intercept goal
        goalNodeID = snapped;

        // write predicted positions to a file for debugging and plotting (uncomment if needed)
        // string file = "files/test" + to_string(testConfig) + "/" + output_subfolder + "/predicted_positions.txt";
        // ofstream flog(file, ios::app);
        // Node &n = G.nodes[snapped];
        // flog << n.x << "," << n.y << "\n";

        // reset our execution counter and mark “goal fresh”
        execCounter = 0;
        needNewInterceptGoal = false;
      }
      // 2) We still plan here as in greedy, but with the same goalNodeID
      
      // 3) After planning & moving the agent one step (below), we increment our execCounter, and once we hit execHorizon,
      // we’ll replan a fresh intercept 

      execCounter++;
      if (execCounter >= execHorizon) 
      {
        needNewInterceptGoal = true;
      }
    }
    else
    {
      // greedy pursuit normally if intercept = false 
      goalNodeID = TP.currentTargetNode();
    }

    time_step++;
    
    for(int i = 0; i < G.numNodes; i++)
    {
      Node* thisNode = &G.nodes[i];
      H.removeNodeFromHeap(thisNode);

      thisNode->parentNode = NULL;

    }

    startNode = &G.nodes[startNodeID];
    goalNode = &G.nodes[goalNodeID];

    dist_from_target = heurisitic_func(startNode, goalNode);
    printf("Agent Distance from target: %.2f\n", dist_from_target);
    
    if(dist_from_target < target_range)
    {
      endSearch = true;
      printf("Target Found!!!\n\n");
    }

    // Uncomment below if you only want 1 time step to run
    //endSearch = true;

  }


  return 0;
}
