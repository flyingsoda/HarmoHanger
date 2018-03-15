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
    nodeString[1].xvel = -3;
}

//Does mass position calculation by iterating mathematics per node
float SpringString::calcMovement(){
    
    //full body calculation of every Node
    
    //Operations on only beginning node
//    makeOld(nodeString[0]);    //Pushes initial values to old
//    for(int j = 0; j < timeStepValue; j++){
//        float fnxaccel = -(nodeString[0].oldxpos - nodeString[1].oldxpos) * nodek / nodeMass;
//        nodeString[0].xvel = euler(nodeString[0].oldxvel, fnxaccel);
//        nodeString[0].xpos = euler(nodeString[0].oldxpos, nodeString[0].oldxvel);
//    }
    
    //Operations on only end node
    makeOld(nodeString[nodeNum-1]);
    //for(int j = 0; j < timeStepValue; j++){
        float lnxaccel = (nodeString[nodeNum-2].oldxpos + nodeString[nodeNum-1].oldxpos) * nodek / nodeMass;
    //set nodes for next passthrough
        nodeString[nodeNum-1].xvel = euler(nodeString[nodeNum-1].oldxvel, -lnxaccel);
        nodeString[nodeNum-1].xpos = euler(nodeString[nodeNum-1].oldxpos, nodeString[nodeNum-1].oldxvel);
    
   //}
    
    //Operations on every node except beginning and end
    for(int i = 1; i < nodeNum-1; i++){
        makeOld(nodeString[i]);
        for(int j = 0; j < nodeIt; j++){
            float force =  -(nodeString[i].oldxpos - nodeString[i-1].oldxpos)*nodek + (nodeString[i].oldxpos - nodeString[i+1].oldxpos)*nodek;
            float force2 = -(2*nodeString[i].oldxpos - nodeString[i-1].oldxpos - nodeString[i+1].xpos) * nodek;
            
            float xaccel = force / nodeMass; //finds new acceleration for oscillator

            //sets new nodeString[i] velocity to new veloctiy from acceleration
            nodeString[i].xvel = euler(nodeString[i].oldxvel, xaccel/nodeIt);

            //sets and sends new nodeString[i] position using velocity
            nodeString[i].xpos = 1000;// euler(nodeString[i].oldxpos, nodeString[i].oldxvel/nodeIt);
            //std::cout << "Got called" << std::endl;
        }
    }
   std::cout << nodeString[0].xpos << " " << nodeString[1].xpos << " " << nodeString[2].xpos << " " << nodeString[3].xpos << std::endl;
    
    
    
    
//
//    //finds new acceleration for oscillator nodestring[0]
//    float xaccel = -nodeString[0].xpos * nodek / nodeMass;
//
//    //sets new nodeString[0] velocity to new veloctiy from acceleration
//    nodeString[0].xvel = euler(nodeString[0].xvel, xaccel);
//
//    //sets and sends new nodeString[0] position using velocity
//    nodeString[0].xpos = euler(nodeString[0].xpos, nodeString[0].xvel);
//
    //if ((nodeString[0].xpos) > 5 || nodeString[0].xpos < -5 )
        //std::cout << nodeString[0].xpos << std::endl;
//
    return nodeString[0].xpos;
}

//Naive implementation of position calculation from the node; just X for now.
float SpringString::euler(float initval, float derval){
    return initval + SRateStep * derval; //previous position plus the velocity * SampleRate (time)
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
