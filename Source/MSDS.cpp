//
//  MSDS.cpp
//  HarmoHanger - App
//
//  Created by William Brewer on 3/4/18.
//

#include "MSDS.h"
#include <iostream>
//#include <cstdlib>

SpringString::SpringString(int numNodes, int simIt, float length, float k_val, float mass, float damping){
    nodeSpace = numNodes/length;
    nodeMass = 1/mass;
    nodek = k_val;
    nodeDamp = damping;
    nodeNum = numNodes;
    nodeIt = simIt;
}

SpringString::~SpringString() {
    
}

//Should iterate through movement calculations once

//TODO implementing multiple nodes in x-dimensions

//right now I have an array of Nodes "nodeString"
//must calculate one iteration of node movement

void SpringString::setSampleRate(double SR){
    SRateStep = 1/SR; //Time step of Oscillator
    SRateRK = SRateStep/6;
    nodeString[0].xvel = 5;
    nodeString[1].xpos = 1;
    nodeString[2].xpos = -1;
}

//Does mass position calculation by iterating mathematics per node
float SpringString::calcMovement(){
    switch (method) {
        case Euler: {
            for(int j = 0; j < nodeIt; j++){
                for (int i = 0; i < nodeNum; i++){
                    makeOld(nodeString[i]);
                }
                //full body calculation of every Node
                
                //Operations on every node except beginning
                for(int i = 1; i < nodeNum-1; i++){
                    
                    float force = -(2*nodeString[i].xpos - nodeString[i-1].oldxpos - nodeString[i+1].xpos) * nodek;
                    float xaccel = force * nodeMass; //finds new acceleration for oscillator
                    
                    //sets new nodeString[i] velocity to new veloctiy from acceleration and
                    //sets and sends new nodeString[i] position using velocity
                    nodeString[i].xvel = euler(nodeString[i].xvel, xaccel/nodeIt);
                    nodeString[i].xpos = euler(nodeString[i].xpos, nodeString[i].xvel/nodeIt);
                }
                
                //Operations on only end node
                float lnxaccel = (nodeString[nodeNum-2].oldxpos - nodeString[nodeNum-1].xpos) * nodek - nodeString[nodeNum-1].oldxvel * nodeDamp;
                lnxaccel *= nodeMass;
                //set nodes for next passthrough
                nodeString[nodeNum-1].xvel = euler(nodeString[nodeNum-1].xvel, lnxaccel / nodeIt);
                nodeString[nodeNum-1].xpos = euler(nodeString[nodeNum-1].xpos, nodeString[nodeNum-1].xvel / nodeIt);
            }
            break;
        }
            
        case RungeKutta4:{
            
            for (int i = 0; i < nodeNum; i++){
                makeOld(nodeString[i]);
            }
            rK4(nodeString, 1);
            break;
        }
            
        default:
            std::cout << "Method not determined" << std::endl;
            break;
    }
    
    std::cout << nodeString[0].xpos << " " << nodeString[1].xpos << " " << nodeString[2].xpos << std::endl;
    
    return nodeString[1].xpos;
}

//Naive implementation of position calculation from the node; just X for now.
float SpringString::euler(float initval, float derval){
    return initval + SRateStep * derval; //previous position plus the velocity * 1/SampleRate (time)
}

float SpringString::rK4(node n[], int position){
    
    //Sends work to each helper function
    rK4p1(n, position);
    rK4p2(n, position);
    rK4p3(n, position);
    rK4p4(n, position);
    rK4f(n, position);

    return 0;
}

void SpringString::rK4p1(node n[], int position){
    //    x1 = x
    //    v1 = v
    //    a1 = a(x1, v1)
    
    n[position].vel[0] = n[position].xvel;
    n[position].pos[0] = n[position].xpos;
    n[position].acc[0] = -(2*n[position].xpos - n[position-1].oldxpos - n[position+1].xpos) * nodek - nodeString[position].oldxvel * nodeDamp;
    n[position].acc[0] *= nodeMass;
    
    if(position < nodeNum-1){
        rK4p1(n, position + 1);
    }
    
}

void SpringString::rK4p2(node n[], int position){
    //    x2 = x + 0.5*v1*dt
    //    v2 = v + 0.5*a1*dt
    //    a2 = a(x2, v2)
    
    n[position].pos[1] = euler(n[position].xpos, n[position].xvel * .5);
    n[position].vel[1] = euler(n[position].xvel, n[position].acc[0] * .5);
    
    if(position < nodeNum-1){
        rK4p2(n, position + 1);
    }
    
    n[position].acc[1] = -(2*n[position].pos[1] - n[position-1].pos[1] - n[position+1].pos[1]) * nodek - n[position].vel[1] * nodeDamp;
    n[position].acc[1] *= nodeMass;
}

void SpringString::rK4p3(node n[], int position){
    //    x3 = x + 0.5*v2*dt
    //    v3 = v + 0.5*a2*dt
    //    a3 = a(x3, v3)
    
    n[position].pos[2] = euler(n[position].pos[1], n[position].vel[1] * .5);
    n[position].vel[2] = euler(n[position].vel[1], n[position].acc[1] * .5);
    
    if(position < nodeNum-1){
        rK4p3(n, position + 1);
    }
    
    n[position].acc[2] = -(2*n[position].pos[2] - n[position-1].pos[2] - n[position+1].pos[2]) * nodek - n[position].vel[2] * nodeDamp;
    n[position].acc[2] *= nodeMass;
}
void SpringString::rK4p4(node n[], int position){
    //    x4 = x + v3*dt
    //    v4 = v + a3*dt
    //    a4 = a(x4, v4)
    
    n[position].pos[3] = euler(n[position].pos[2], n[position].vel[2]);
    n[position].vel[3] = euler(n[position].vel[2], n[position].acc[2]);
    
    if(position < nodeNum-1){
        rK4p4(n, position + 1);
    }
    
    n[position].acc[3] = -(2*n[position].pos[3] - n[position-1].pos[3] - n[position+1].pos[3]) * nodek - n[position].vel[3] * nodeDamp;
    n[position].acc[3] *= nodeMass;
}
void SpringString::rK4f(node n[], int position){
    //    xf = x + (dt/6.0)*(v1 + 2*v2 + 2*v3 + v4)
    //    vf = v + (dt/6.0)*(a1 + 2*a2 + 2*a3 + a4)
    
    n[position].xpos = n[position].xpos + SRateRK * (n[position].vel[0] + 2*n[position].vel[1] + 2*n[position].vel[2] + n[position].vel[3]);
    n[position].xvel = n[position].xvel + SRateRK * (n[position].acc[0] + 2*n[position].acc[1] + 2*n[position].acc[2] + n[position].acc[3]);
    
    if(position < nodeNum-1){
        rK4f(n, position + 1);
    }
}

void SpringString::makeOld(node& handle){
    handle.oldxpos = handle.xpos;
    handle.oldypos = handle.ypos;
    handle.oldzpos = handle.zpos;
    handle.oldxvel = handle.xvel;
    handle.oldyvel = handle.yvel;
    handle.oldzvel = handle.zvel;
}
