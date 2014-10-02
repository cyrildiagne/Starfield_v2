//
//  AHRS.cpp
//  starfield_v2
//
//  Created by kikko on 26/09/14.
//
//

#include "AHRS.h"

#define BINARY_OUTPUT          "#ob"
#define CONTINUOUS_STREAMING   "#o1"
#define DISABLE_ERROR_MESSAGE  "#oe0"
#define REQUEST_SYNC_TOKEN     "#s00"
#define SYNC_TOKEN             "#SYNCH00\r\n"
#define PACKET_SEPARATOR       "#n\r\n"

AHRS::AHRS(){
    yaw   = 0.0f;
    pitch = 0.0f;
    roll  = 0.0f;
    
    acc_x = 0.0f;
    acc_y = 0.0f;
    acc_z = 0.0f;
    
    yawOffset = 0.0f;
    
    fc_acc_low = 0.1;
    accBiquadLow.setType(OFX_BIQUAD_TYPE_LOWPASS);
    
    fc_acc_high = 0.005;
    accBiquadHigh.setType(OFX_BIQUAD_TYPE_HIGHPASS);
    
    fc_velocity = 0.005;
    velBiquad.setType(OFX_BIQUAD_TYPE_HIGHPASS);
    
    fc_position = 0.001;
    posBiquad.setType(OFX_BIQUAD_TYPE_HIGHPASS);
    
    base = 0;
    
    isAvailable = false;
    isPacketReady = false;
    numUpdateSinceReconnect = 0;
    numUpdateSinceLastPacket = 0;
    currLine = "";
    currPort = "";
    
    kAcc = ofVec3f::zero();
    kAccFilter.setup(ofVec3f::zero(), ofVec3f::zero(), ofVec3f::zero(), 1e-4, 1e-2);
    
    plot = NULL;
}

AHRS::~AHRS(){
    close();
    if(plot){
        delete plot;
    }
}

void AHRS::close(){
    serial.close();
}

void AHRS::setup(string port){
    currPort = port;
    ofSetLogLevel("ofSerial", OF_LOG_ERROR);
    
    camera.setup();
    camera.target.y = 500;
    camera.distance = 2000;
    camera.latitude = -10;
    
    box.set(650, 40, 250, 1, 1, 1);

    plot = new ofxHistoryPlot(240, true);
//    plot->add(&acc.x, "Acc X", ofColor(255,0,0,80));
    //    plot->add(&acc.y, "Acc Y", ofColor(0,255,0,80));
    plot->add(&base, "zero", ofColor(255,255,255,70));
    plot->add(&acc.z, "Acc Z", ofColor(0,125,255,60));
//    plot->add(&scaledVel.x, "Velocity X", ofColor::magenta);
//    plot->add(&scaledVel.y, "Velocity Y", ofColor::yellow);
//    plot->add(&scaledVel.y, "Velocity Y", ofColor::white);
//    plot->add(&kAcc.x, "kalman acc X", ofColor::red);
//    plot->add(&kAcc.y, "kalman acc Y", ofColor::green);
//    plot->add(&kAcc.z, "kalman acc Z", ofColor(0,125,255,150));
    plot->add(&filteredAcc.z, "biquad acc Z", ofColor(0,125,255,150));
    plot->add(&filteredVel.z, "biquad vel Z", ofColor(125,180,255,200));
//    plot->add(&filteredPos.z, "highpass pos", ofColor(255,255,255,255));
    plot->setRange(-256, 256);
    plot->setRespectBorders(true);
    
    connect();
}

void AHRS::connect(){
    
    serial.setup(currPort, 57600);
    if(serial.isInitialized()) {
        serial.writeBytes((unsigned char *)BINARY_OUTPUT, 3);
        serial.writeBytes((unsigned char *)CONTINUOUS_STREAMING, 3);
        serial.writeBytes((unsigned char *)DISABLE_ERROR_MESSAGE, 4);
    }
}

float AHRS::update(){
    
    if(!serial.isInitialized()) {
        if(numUpdateSinceReconnect++ > 120){
            connect();
            numUpdateSinceReconnect = 0;
        }
        return;
    }
    
    isPacketReady = isPacketReady || lookForSep();
    if(isPacketReady) {
        while (serial.available() >= 24) {
            serial.readBytes((unsigned char*)&yaw, 4);
            serial.readBytes((unsigned char*)&pitch, 4);
            serial.readBytes((unsigned char*)&roll, 4);
            serial.readBytes((unsigned char*)&acc_x, 4);
            serial.readBytes((unsigned char*)&acc_y, 4);
            serial.readBytes((unsigned char*)&acc_z, 4);
            if(yawOffset > -0.0001f && yawOffset < 0.0001f) {
                applyReferenceYaw();
            }
            isPacketReady = lookForSep();
        }
        isAvailable = true;
        numUpdateSinceLastPacket = 0;
    } else {
        if (++numUpdateSinceLastPacket > 120) {
            serial.close();
            isAvailable = false;
            connect();
            numUpdateSinceLastPacket = 0;
        }
    }
    
    updateMotionTracking();
    
    if( filteredVel.z < 0){
        return filteredVel.z * 0.00002;
    } else {
        return filteredVel.z * 0.00007;
    }
}

void AHRS::updateMotionTracking(){
    
    ofQuaternion q = box.getOrientationQuat();
    gravity = q * ofVec3f(0.f, 256.f, 0.f);
    acc.x = acc_y+gravity.x;
    acc.y = acc_z-gravity.y;
    acc.z = -(acc_x-gravity.z);
//    acc.x = acc_y;
//    acc.y = acc_z;
//    acc.z = -acc_x;
    
    accBiquadLow.setFc(fc_acc_low);
    accBiquadLow.update(acc);
    filteredAcc = accBiquadLow.value();
    
//    filteredAcc = acc;
    
    accBiquadHigh.setFc(fc_acc_high);
    accBiquadHigh.update(filteredAcc);
    filteredAcc = accBiquadHigh.value();
    
    velocity += filteredAcc;
    velBiquad.setFc(fc_velocity);
    velBiquad.update(velocity);
    filteredVel = velBiquad.value();
//    filteredVel = velocity;
    
    position += filteredVel;
    posBiquad.setFc(fc_position);
    posBiquad.update(position);
    filteredPos = posBiquad.value();
    
    kAcc = kAccFilter.update(acc);
    
    box.setPosition(filteredPos * 0.05);
//    box.setPosition(ofVec3f(0,0,position.z));
}

void AHRS::readAsLog(){
    unsigned char buffer[256];
    memset(buffer, 0, 256);
    
    int num = serial.available();
    serial.readBytes(buffer, num);
    currLine += reinterpret_cast<char*>(buffer);
    vector<string> lines = ofSplitString(currLine, "\r\n");
    for (int i=0; i<lines.size()-1; i++) {
        ofLog() << lines[i];
    }
    currLine = lines[lines.size()-1];
}

void AHRS::draw(){
    
    if(!serial.isInitialized() || !isAvailable){
        ostringstream buff;
        if (!serial.isInitialized()) {
            buff << "Can't connect to serial port " << currPort;
            ofDrawBitmapString(buff.str(), (ofGetWidth()-400)*0.5, ofGetHeight()*0.5);
        } else if(!isAvailable) {
            buff << "Not receiving data from AHRS";
            ofDrawBitmapString(buff.str(), (ofGetWidth()-250)*0.5, ofGetHeight()*0.5);
        }
        return;
    }
    
    camera.update();
    camera.begin();
    
    ofEnableDepthTest();
    
    box.setOrientation(ofVec3f::zero());
    ofQuaternion qr(roll, ofVec3f(0,0,1));
    ofQuaternion qp(-pitch, ofVec3f(1,0,0));
    ofQuaternion qy(-(yaw-yawOffset), ofVec3f(0,1,0));
    ofQuaternion qt = qr * qp * qy;
    box.setOrientation(qt);
    
    ofPushStyle();
    ofSetColor(0, 125, 255);
    box.drawWireframe();
    ofPopStyle();
    
    drawGrid();
    drawAcc();
    
    ofDisableDepthTest();
    
    camera.end();
    
//    ostringstream buff;
//    buff << "yaw   : " << yaw   << endl
//         << "pitch : " << pitch << endl
//         << "roll  : " << roll  << endl;
//    ofDrawBitmapString(buff.str(), 10, 15);
    
    ofPushStyle();
    int w = ofGetWidth() * 0.25;
    int h = ofGetHeight()*0.25;
    plot->draw(ofGetWidth()-10-w, 10, w, h);
//    int w = ofGetWidth() * 0.5;
//    plot->draw((ofGetWidth()-w)*0.5, 10, w, 250);
    ofPopStyle();
}

void AHRS::drawAcc(){
    
    ofPushStyle();
    ofSetColor(255, 0, 0);
    ofLine(ofVec3f::zero(), ofVec3f(1,0,0).scale(acc.x));
    ofSetColor(0, 255, 0);
    ofLine(ofVec3f::zero(), ofVec3f(0,1,0).scale(acc.y));
    ofSetColor(0, 0, 255);
    ofLine(ofVec3f::zero(), ofVec3f(0,0,1).scale(acc.z));
    ofSetColor(255,255,255);
    
    ofSetColor(255, 255, 0);
    ofDrawArrow(ofVec3f::zero(), gravity, 5);
    
    ofPopStyle();
}

void AHRS::drawGrid(){
    ofPushMatrix();
    ofTranslate(0, -100);
    ofRotateZ(90);
    ofPushStyle();
    ofSetColor(90, 90, 90);
    ofDrawGridPlane(750);
    ofPopStyle();
    ofPopMatrix();
}

//-----

void AHRS::applyReferenceYaw() {
    yawOffset = yaw;
}

bool AHRS::lookForSep(){
    string sep = PACKET_SEPARATOR;
    bool bStarted = false;
    int currPos = 0;
    while (serial.available()>0) {
        if (bStarted) {
            if(serial.readByte() != sep[currPos]) {
                bStarted = false;
                currPos = 0;
                continue;
            }
            currPos++;
            if(currPos == sep.length()) {
                return true;
            }
        }
        else if(serial.readByte() == sep[0]) {
            bStarted = true;
            currPos++;
        }
    }
    return false;
}