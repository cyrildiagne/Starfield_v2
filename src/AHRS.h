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
#include "ofxSimpleKalmanFilter.h"
#include "ofxBiquadFilter.h"

class AHRS {
    
public:
    AHRS();
    ~AHRS();
    
    void setup(string port);
    void connect();
    float update();
    void draw();
    void close();
    
    void applyReferenceYaw();
    
    //-- props
    
    float yaw;
    float pitch;
    float roll;
    
    float yawOffset;
    
    ofVec3f gravity;
    ofVec3f rawAcc;
    ofVec3f acc;
    ofVec3f velocity;
    ofVec3f position;
    
    float fc_acc_high;
    float fc_acc_low;
    float fc_velocity;
    float fc_position;
    
    float base;
    
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
    
    ofVec3f scaledVel;
    
    ofVec3f kAcc;
    ofxBasicSimpleKalmanFilter<ofVec3f> kAccFilter;
    
    ofVec3f filteredAcc;
    ofxBiquadFilter3f accBiquadLow;
    ofxBiquadFilter3f accBiquadHigh;
    
    ofVec3f filteredVel;
    ofxBiquadFilter3f velBiquad;
    
    ofVec3f filteredPos;
    ofxBiquadFilter3f posBiquad;
    
    ofxHistoryPlot * plot;
};

#endif /* defined(__starfield_v2__AHRS__) */
