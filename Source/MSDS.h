/*
  ==============================================================================

    MSDS.h
    Created: 4 Mar 2018 6:26:48pm
    Author:  William Brewer

  ==============================================================================
*/

#pragma once
//
//  MSDS.hpp
//  HarmoHanger - App
//
//  Created by William Brewer on 3/4/18.
//

//#ifndef MSDS_hpp
//#define MSDS_hpp

#include <vector>
#include <stdio.h>

//#endif /* MSDS_hpp */

struct node
{
    //updated position
    float xpos = 0;
    float xvel = 0;
    float ypos = 0;
    float yvel = 0;
    float zpos = 0;
    float zvel = 0;
    
    //variables for Runge Kutta
    float pos[4] = {0, 0, 0, 0};
    float vel[4] = {0, 0, 0, 0};
    float acc[4] = {0, 0, 0, 0};
    
    //past position
    float oldxpos = 0;
    float oldxvel = 0;
    float oldypos = 0;
    float oldyvel = 0;
    float oldzpos = 0;
    float oldzvel = 0;
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
    //numNodes MUST be at least 2, to have an anchor
    SpringString(int numNodes = 7, int simIt = 100, float length = 10.f, float k_val = 1000000.f, float mass = 1.f, float damping = 0.f);
    ~SpringString();
    
    void setSampleRate(double SR);
    float calcMovement();
    
private:
    node nodeString[100];
    float SRateStep;    // 1/SampleRate, used to determine iteration speed.
    float SRateRK;      // 1/SampleRate/6, to determine Runge-Kutta
    float nodeSpace;    //Distance between each Node
    float nodeMass;     // 1/Mass of each Node
    float nodek;        //K value (tension value) of Node
    float nodeDamp;     //Rate that Node loses energy)
    int nodeNum;        //Number of Nodes
    int nodeIt;         //number of calcMovements called before returning position of indexed node
    
    enum Method {Euler, RungeKutta4}; //Determines whether to calculate using Euler or Runge Kutta method
    Method method = RungeKutta4;
    
    int randcount = 0;  //debugging counters
    int randcount2 = 0; //
    
    //Calculates Node position using Euler's method
    float euler(float initval, float derval);
    
    //Calculates Node position using Runge Kutta method with helper functions p1-p4
    float rK4(node n[], int position);
    void rK4p1(node n[], int position);
    void rK4p2(node n[], int position);
    void rK4p3(node n[], int position);
    void rK4p4(node n[], int position);
    void rK4f(node n[], int position);

    //Pushes node values back
    void makeOld(node& handle);
};
