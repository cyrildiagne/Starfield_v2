#pragma once

#include "ofMain.h"

//#define USE_AHRS

#ifdef USE_AHRS
    #include "AHRS.h"
#else
    #include "KinectInput.h"
#endif

#include "ofxUI.h"
#include "View.h"

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
    void setupUI();
    
    void windowResized(int w, int h);
    void keyPressed(int key);
    
private:
    
#ifdef USE_AHRS
    AHRS ahrs;
#else
    KinectInput kinect;
#endif
    
    View view;
    
    ofxUICanvas* viewControlUI;
    ofxUICanvas* sensorUI;
    
    bool bDrawDebug;
};
