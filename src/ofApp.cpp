#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(0);
    ofSetFrameRate(60);
//    ofSetVerticalSync(true);
//    ofDisableArbTex();
    
#ifdef USE_AHRS
    ahrs.setup("/dev/tty.usbserial-A501B6XW");
//    ahrs.setup("/dev/tty.usbserial-A70063zH");
#else
    kinect.setup();
#endif
    
    view.setup();
    
    bDrawDebug = false;
    
    viewControlUI = NULL;
    sensorUI = NULL;
    setupUI();
}

void ofApp::setupUI(){
    
    viewControlUI = new ofxUICanvas();
    viewControlUI->addLabel("VIEW");
    viewControlUI->addToggle("isVisible", &view.bVisible);
    viewControlUI->addSlider("speed", 0, 10000, &view.speed);
    viewControlUI->addSlider("offsetX", -1.f, 1.f, &view.offsetX);
    viewControlUI->addSlider("offsetY", -1.f, 1.f, &view.offsetY);
//    viewControlUI->addSlider("keystone up", -1.f, 1.f, &view.keyStoneUp);
    viewControlUI->addSpacer();
    viewControlUI->addLabel("Moon");
    viewControlUI->add2DPad("moon pos", ofVec2f(0,1), ofVec2f(0,1), &view.moonPos, 120, 70);
    viewControlUI->addSlider("moon rotation", 0, 360, &view.moonRot);
    viewControlUI->addSlider("moon scale", 0, 2, &view.moonScale);
    viewControlUI->addSpacer();
    viewControlUI->addLabel("Clouds");
    viewControlUI->addSlider("clouds offset Y", 0.f, -15000.f, &view.clouds.offset.y);
    viewControlUI->addSpacer();
    viewControlUI->addLabel("Stars");
    viewControlUI->addSlider("starSize", 0.f, 512.f, &view.stars.starSize);
    viewControlUI->addIntSlider("numStars", 0, 3000, &view.stars.numStars);
    viewControlUI->setVisible(bDrawDebug);
    viewControlUI->autoSizeToFitWidgets();
    viewControlUI->loadSettings("view_settings.xml");
    
    sensorUI = new ofxUICanvas("SENSOR");
    sensorUI->setPosition(viewControlUI->getRect()->getWidth() + 5, 0);
#ifdef USE_AHRS
    sensorUI->addLabel("AHRS");
    sensorUI->addSlider("fc_acc_low", 0.f, 0.5f, &ahrs.fc_acc_low);
    sensorUI->addSlider("fc_acc_high", 0.f, 0.05f, &ahrs.fc_acc_high);
    sensorUI->addSlider("fc_velocity", 0.f, 0.05f, &ahrs.fc_velocity);
    sensorUI->addSlider("fc_position", 0.f, 1.f, &ahrs.fc_position);
    sensorUI->loadSettings("ahrs_settings.xml");
#else
    sensorUI->addLabel("KINECT");
    sensorUI->addSlider("roiX", 0, 640, &kinect.roi.x);
    sensorUI->addSlider("roiY", 0, 480, &kinect.roi.y);
    sensorUI->addSlider("roiWidth", 0, 640, &kinect.roi.width);
    sensorUI->addSlider("roiHeight", 0, 480, &kinect.roi.height);
    sensorUI->addSlider("blobSizeMin", 0.f, 50000.f, &kinect.blobSizeMin);
    sensorUI->addIntSlider("threshold", 0, 255, &kinect.threshold);
    sensorUI->addSlider("speedLowPassFc", 0.f, 0.1f, &kinect.speedLowPassFc);
    sensorUI->autoSizeToFitWidgets();
    sensorUI->loadSettings("kinect_settings.xml");
#endif
    sensorUI->setVisible(bDrawDebug);
    
}


//--------------------------------------------------------------
void ofApp::exit(){
#ifdef USE_AHRS
    ahrs.close();
    sensorUI->saveSettings("ahrs_settings.xml");
#else
    sensorUI->saveSettings("kinect_settings.xml");
#endif
    delete sensorUI;
    
    if (viewControlUI) {
        viewControlUI->saveSettings("view_settings.xml");
        delete viewControlUI;
    }
    
    view.exit();
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
    view.setSize(w,h);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    float speed = 0.f;
    
#ifdef USE_AHRS
    speed = ahrs.update();
#else
    speed = kinect.update();
//    speed = 0.05;
#endif
    
    view.update(speed);
    
    ofSetWindowTitle(ofToString(ofGetFrameRate(),1)+"fps");
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    view.draw();
    
    if(!bDrawDebug) return;
    
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
        case 'f':
            ofToggleFullscreen();
            break;
            
        case 's':
            bDrawDebug = !bDrawDebug;
            view.bDrawDebug = bDrawDebug;
            viewControlUI->setVisible(bDrawDebug);
            sensorUI->setVisible(bDrawDebug);
            break;
    }
}