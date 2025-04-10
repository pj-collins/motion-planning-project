/*
File that includes class and function definitions for robot plus its dynamics
*/

#ifndef TARGET_H
#define TARGET_H

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <random>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>


class TargetPath
{

public:
    // Target path state vectors
    std::vector<int> node_id;           // Target node ID
    std::vector<double> x;              // Target x-position
    std::vector<double> y;              // Target y-position
    std::vector<double> step_cost;      // Target step cost
    std::vector<double> total_cost;     // Target total cost

    int num_nodes;                      // Number of nodes in the path
    int current_idx;                    // Current index of path
    double v;                           // Target velocity

    TargetPath() 
    { 
        current_idx = 0;
    }

    ~TargetPath() { }

    // Function to set target velocity
    bool setTargetVelocity(double vel)
    {
        v = vel;

        return true;
    }

    // Function for reading target path information from a text file
    bool readTargetPathFromFile(const char* targetFilename)
    {
        FILE * pFile;
        
        printf("Opening target path file.\n");

        // Open the target path file
        pFile = fopen(targetFilename , "r");
        if(pFile==NULL) 
        {
            printf("Unable to open file %s.\n", targetFilename);
            return false;
        }

        // Read the number of target points
        if(fscanf(pFile, "%d\n", &num_nodes) < 1)
        {
            printf("Problem reading number of path nodes from file.\n"); 
            return false;
        }
        else
        {
            printf("%d path nodes indicated to exist.\n", num_nodes);
        }

        // Allocate variables for temporarily storing file inputs
        int node_id_p;
        double x_p;
        double y_p;
        double step_cost_p;
        double total_cost_p;

        for(int n = 0; n < num_nodes; n++)
        {
            // Read the line of data
            if(fscanf(pFile, "%d, %lf, %lf, %lf, %lf\n", &node_id_p, &x_p, &y_p, &step_cost_p, &total_cost_p) < 5)
            {
                printf("Problem reading line %d from file.\n\n",n+1); 
                return false;
            }
            // Add data to the class vectord
            node_id.push_back(node_id_p); 
            x.push_back(x_p); 
            y.push_back(y_p); 
            step_cost.push_back(step_cost_p); 
            total_cost.push_back(total_cost_p); 
        }

        fclose(pFile);

        printf("Done reading target points from file.\n\n");
        return true;

    }

    // Function for printing out the target path in order
    bool printTargetPath()
    {
        printf("\nPrinting target path consisting of %d nodes\n", num_nodes);

        for(int n = 0; n < num_nodes; n++)
        {
            printf("Step %d: Node ID = %d, Coordinates = (%.2f, %.2f), Step Cost = %.2f, Total Cost = %.2f\n", n, node_id[n], x[n], y[n], step_cost[n], total_cost[n]);
        }

        return true;
    }

    // Function that steps the target forward a number of steps capped by the velocity
    int stepForward()
    {
        // Remember the previous Index
        int previous_idx = current_idx;
        double cost_left = v;

        // Increment forward one step at a time
        while(true)
        {
            cost_left -= step_cost[current_idx+1];
            current_idx++;

            if(current_idx + 1 == num_nodes)
            {
                break;
            }
            else if(cost_left - step_cost[current_idx+1] < 0)
            {
                break;
            }
        }

        // Print the path traversed on this step
        printf("Target traversed nodes: %d ", node_id[previous_idx]);
        for(int n = previous_idx+1; n <= current_idx; n++)
        { 
            printf("-> %d ", node_id[n]);
        }
        printf("\n");

        // Return the node ID of the new current node
        return node_id[current_idx];
    }

    // Function that returns node the target is currently at
    int currentTargetNode()
    {
        return node_id[current_idx];
    }


    
};


#endif //TARGET_H