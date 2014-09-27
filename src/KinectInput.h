//
//  KinectInput.h
//  starfield_v2
//
//  Created by kikko on 26/09/14.
//
//

#ifndef __starfield_v2__KinectInput__
#define __starfield_v2__KinectInput__

#include "KinectInput.h"

//#define USE_DEVICE

#ifdef USE_DEVICE
    #include "ofxKinect.h"
#endif

#include "ofxOpenCv.h"

#define CAM_WIDTH 640
#define CAM_HEIGHT 480




class ROI : public ofBaseDraws, public ofRectangle{
    
public:
    void draw(float _x,float _y) const {
        draw(_x, _y, getWidth(), getHeight());
    }
    void draw(float _x,float _y,float w, float h) const {
        
        ofPushMatrix();
        ofTranslate(_x, _y);
        ofScale(w/CAM_WIDTH, h/CAM_HEIGHT);
        
        ofPushStyle();
        ofSetColor(255, 0, 0);
        ofNoFill();
        ofSetLineWidth(1);
        ofRect(*this);
        ofPopStyle();
        
        ofPopMatrix();
    }
    float getWidth() const {
        return CAM_WIDTH;
    }
    float getHeight() const {
        return CAM_HEIGHT;
    }
};





class KinectInput {
    
public:
    
    KinectInput();
    ~KinectInput();
    
    void setup();
    void update();
    void draw();
    
private:
    
#ifdef USE_DEVICE
    ofxKinect kinect;
#else
    ofVideoPlayer kinectPlayer;
    ofImage videoTempImg;
#endif
    
    void fillBlobPoly();
    void updateAvgDepth();
    
    ofxCvContourFinder  contourFinder;
    ofxCvGrayscaleImage depthImage;
    ofxCvGrayscaleImage thresholdImage;
    
    ROI roi;
    
    int kinectAngle, prevKinectAngle;
    float blobSizeMin;
    int threshold;
    int bUserFound;
    float avgDepth;
    float avgDepthSmoothed;
    
    float currPosMin;
    float currPosMax;
    float currSpeed;
    float currSpeedMax;
};


#endif /* defined(__starfield_v2__KinectInput__) */
