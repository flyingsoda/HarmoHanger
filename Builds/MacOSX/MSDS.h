//
//  MSDS.hpp
//  HarmoHanger - App
//
//  Created by William Brewer on 3/4/18.
//

#ifndef MSDS_hpp
#define MSDS_hpp

#include <vector>
#include <stdio.h>

#endif /* MSDS_hpp */

struct node
{
    float xpos = 0;
    float xvel = 0;
    float ypos = 0;
    float yvel = 0;
    float zpos = 0;
    float zvel = 0;
};

class SpringString
{

public:
    //Should declare a string of nodes (no greater than nodeString's declared # of nodes) that upon running
    //one instance of simulation finds the approximate value of each nodes position velocity by running a
    //simulation simIt # of times.
    //Node distance from each other is length/numNodes.
    //k_val represents spring constant 'k' in F = ma = -kx
    //mass represents individual mass of nodes
    SpringString(int numNodes = 10, int simIt = 50, float length = 10.f, float k_val = 100.f, float mass = 100.f);
    ~SpringString();
    
    float calcmovement();
    
private:
    node nodeString[100];
    
};
