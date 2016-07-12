//
//  bowTrack.cpp
//  detectTest
//
//  Created by Kosmas Kritsis on 01/06/16.
//  Copyright © 2016 Kosmas Kritsis. All rights reserved.
//

#include "bowTrack.hpp"


bowTrack::bowTrack()
{
    //set values for default constructor
    setType("null");
    setColour(Scalar(0,0,0));
    
}

bowTrack::bowTrack(string name){
    
    setType(name);
    
    if(name=="violin"){
        
        //TODO: use "calibration mode" to find HSV min
        //and HSV max values
        
        setHSVmin(Scalar(96,179,78));
        setHSVmax(Scalar(121,256,230));
        
        //BGR value for Blue:
        setColour(Scalar(255,0,0));
        
    }
    if(name=="bow"){
        
        //TODO: use "calibration mode" to find HSV min
        //and HSV max values
        
        setHSVmin(Scalar(26,62,62));
        setHSVmax(Scalar(35,255,255));
        
        //BGR value for Yellow:
        setColour(Scalar(0,255,255));
        
    }
}

bowTrack::~bowTrack(void)
{
}

int bowTrack::getXPos(){
    
    return bowTrack::xPos;
    
}

void bowTrack::setXPos(int x){
    
    bowTrack::xPos = x;
    
}

int bowTrack::getYPos(){
    
    return bowTrack::yPos;
    
}

void bowTrack::setYPos(int y){
    
    bowTrack::yPos = y;
    
}

Scalar bowTrack::getHSVmin(){
    
    return bowTrack::HSVmin;
    
}
Scalar bowTrack::getHSVmax(){
    
    return bowTrack::HSVmax;
}

void bowTrack::setHSVmin(Scalar min){
    
    bowTrack::HSVmin = min;
}


void bowTrack::setHSVmax(Scalar max){
    
    bowTrack::HSVmax = max;
}