//
//  AHRS.h
//  starfield_v2
//
//  Created by kikko on 26/09/14.
//
//

#ifndef __starfield_v2__AHRS__
#define __starfield_v2__AHRS__

#include "ofMain.h"
#include "Camera.h"
#include "ofxHistoryPlot.h"


class AHRS {
    
public:
    AHRS();
    ~AHRS();
    
    void setup(string port);
    void connect();
    void update();
    void draw();
    
    void applyReferenceYaw();
    
    //-- props
    
    float yaw;
    float pitch;
    float roll;
    
    float yawOffset;
    
    ofVec3f gravity;
    ofVec3f acc;
    ofVec3f velocity;
    ofVec3f position;
    
    float accHighPassFilter;
    float velHighPassFilter;
    
private:
    
    float acc_x;
    float acc_y;
    float acc_z;
    
    ofSerial serial;
    int numUpdateSinceReconnect;
    int numUpdateSinceLastPacket;
    bool isAvailable;
    bool isPacketReady;
    bool lookForSep();
    void readAsLog();
    string currLine;
    string currPort;

    void updateMotionTracking();
    
    Camera camera;
    ofBoxPrimitive box;
    void drawGrid();
    void drawAcc();
    
    ofxHistoryPlot * plot;
};

#endif /* defined(__starfield_v2__AHRS__) */
