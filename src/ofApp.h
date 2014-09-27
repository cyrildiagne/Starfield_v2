#pragma once

#include "ofMain.h"

#define USE_AHRS

#ifdef USE_AHRS
    #include "AHRS.h"
#else
    #include "KinectInput.h"
#endif



class ofApp : public ofBaseApp{
    
#ifdef USE_AHRS
    AHRS ahrs;
#else
    KinectInput kinect;
#endif
    
public:
    void setup();
    void update();
    void draw();

    void keyPressed(int key);
};
