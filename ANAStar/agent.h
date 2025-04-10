/*
File that includes class and function definitions for robot plus its dynamics
*/

#ifndef AGENT_H
#define AGENT_H

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


class Agent
{

public:
    // Agent states
    int current_node;                   // Agent position node ID
    double x;                           // Agent x-position
    double y;                           // Agent y-position
    double v;                           // Agent velocity

    Agent() { }

    ~Agent() { }

    // Function to set target velocity
    bool setAgentVelocity(double vel)
    {
        v = vel;

        return true;
    }

    
};


#endif //AGENT_H