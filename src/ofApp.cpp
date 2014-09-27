#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(0);
    ofSetFrameRate(60);
    
#ifdef USE_AHRS
    ahrs.setup("/dev/tty.usbserial-A501B6XW");
//    ahrs.setup("/dev/tty.usbserial-A70063zH");
#else
    kinect.setup();
#endif
    
}

//--------------------------------------------------------------
void ofApp::update(){
#ifdef USE_AHRS
    ahrs.update();
#else
    kinect.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw(){
#ifdef USE_AHRS
    ahrs.draw();
#else
    kinect.draw();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch(key) {
#ifdef USE_AHRS
        case ' ' :
            ahrs.applyReferenceYaw();
            break;
#else
            
#endif
    }
}