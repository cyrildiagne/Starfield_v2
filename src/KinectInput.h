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

#define TOP_TRACKING

#ifdef USE_DEVICE
    #include "ofxKinect.h"
#endif

#include "ofxOpenCv.h"
#include "ofxHistoryPlot.h"

#define CAM_WIDTH 640
#define CAM_HEIGHT 480

#include "ofxSimpleKalmanFilter.h"
#include "ofxBiquadFilter.h"


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
    float update();
    void draw();
    
    //--- params
    
    float blobSizeMin;
    int threshold;
    float speedLowPassFc;
    
    ROI roi;
    
private:
    
#ifdef USE_DEVICE
    ofxKinect kinect;
#else
    ofVideoPlayer kinectPlayer;
    ofImage videoTempImg;
#endif
    
    void fillBlobPoly();
    
#ifdef TOP_TRACKING
    void updateAvgX();
    vector<int> lineMax;
#else
    void updateAvgDepth();
    ofxCvGrayscaleImage thresholdImage;
    ofxCvContourFinder  contourFinder;
#endif
    
    ofxCvGrayscaleImage depthImage;
    
    int kinectAngle, prevKinectAngle;
    
    int bUserFound;
    
    float avgPos;
    float avgPrevPos;
    
    float currPosMin;
    float currPosMax;
    float currSpeed;
    float currSpeedMax;
    
    ofxHistoryPlot * plot;
    void setupPlot();
    
    float currSpeedKalman;
    ofxBasicSimpleKalmanFilter<float> speedFilter;
    
    float currSpeedBiquad;
    ofxBiquadFilter_<float> speedLowPassBiquad;
};


#endif /* defined(__starfield_v2__KinectInput__) */
