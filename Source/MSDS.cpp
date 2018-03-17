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
    nodeMass = mass;
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
    nodeString[0].xvel = 5;
    nodeString[1].xpos = 1;
    nodeString[2].xpos = -1;
    nodeString[10].xpos = 1;
}

//Does mass position calculation by iterating mathematics per node
float SpringString::calcMovement(){

    for(int j = 0; j < nodeIt; j++){
        for (int i = 0; i < nodeNum; i++){
            makeOld(nodeString[i]);
        }
        //full body calculation of every Node
        
                                //  calculation of single node
                                //    float accel = -(nodeString[10].xpos) * nodek;//- nodeString[nodeNum-1].oldxvel * nodeDamp;
                                //    accel /= nodeMass;
                                //
                                //    //set nodes for next passthrough
                                //    nodeString[10].xvel = euler(nodeString[10].xvel, accel);
                                //    nodeString[10].xpos = euler(nodeString[10].xpos, nodeString[10].xvel);
                                //
                                //
        


        
        //Operations on every node except beginning and end
        for(int i = 1; i < nodeNum-1; i++){
            
            float force = -(2*nodeString[i].xpos - nodeString[i-1].oldxpos - nodeString[i+1].xpos) * nodek;
            float xaccel = force / nodeMass; //finds new acceleration for oscillator
            
            //sets new nodeString[i] velocity to new veloctiy from acceleration and
            //sets and sends new nodeString[i] position using velocity
            nodeString[i].xvel = euler(nodeString[i].xvel, xaccel/nodeIt);
            nodeString[i].xpos = euler(nodeString[i].xpos, nodeString[i].xvel/nodeIt);
        }
        
        //Operations on only end node
        float lnxaccel = (nodeString[nodeNum-2].oldxpos - nodeString[nodeNum-1].xpos) * nodek ;//- nodeString[nodeNum-1].oldxvel * nodeDamp;
        lnxaccel /= nodeMass;
        //set nodes for next passthrough
        nodeString[nodeNum-1].xvel = euler(nodeString[nodeNum-1].xvel, lnxaccel / nodeIt);
        nodeString[nodeNum-1].xpos = euler(nodeString[nodeNum-1].xpos, nodeString[nodeNum-1].xvel / nodeIt);
    }
    //std::cout << nodeString[0].xpos << " " << nodeString[1].xpos << " " << nodeString[2].xpos << std::endl;
    
    return nodeString[1].xpos;
}

//Naive implementation of position calculation from the node; just X for now.
float SpringString::euler(float initval, float derval){
    return initval + SRateStep * derval; //previous position plus the velocity * 1/SampleRate (time)
}

float SpringString::rK4(node &n){
    return 0;
}

void SpringString::makeOld(node& handle){
    handle.oldxpos = handle.xpos;
    handle.oldypos = handle.ypos;
    handle.oldzpos = handle.zpos;
    handle.oldxvel = handle.xvel;
    handle.oldyvel = handle.yvel;
    handle.oldzvel = handle.zvel;
}
