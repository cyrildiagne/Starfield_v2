//
//  KinectInput.cpp
//  starfield_v2
//
//  Created by kikko on 26/09/14.
//
//

#include "KinectInput.h"

KinectInput::KinectInput():plot(NULL){
    
}

KinectInput::~KinectInput(){
#ifdef USE_DEVICE
    kinect.close();
#endif
}

void KinectInput::setup(){
#ifdef USE_DEVICE
    kinect.init();
    kinect.open();
#else
    kinectPlayer.loadMovie("kinect_depth_v2.mov");
    kinectPlayer.play();
    kinectPlayer.setLoopState(OF_LOOP_NORMAL);
    videoTempImg.allocate(CAM_WIDTH, CAM_HEIGHT, OF_IMAGE_COLOR);
#endif
    
    depthImage.allocate(CAM_WIDTH, CAM_HEIGHT);
    depthImage.setUseTexture(true);
    
#ifdef TOP_TRACKING
    
#else
    thresholdImage.allocate(CAM_WIDTH, CAM_HEIGHT);
#endif
    
    kinectAngle = prevKinectAngle = 0;
    threshold = 10;
    bUserFound = 0;
    avgPos = avgPrevPos = 0.f;
    blobSizeMin = 5000.f;
    
    currPosMin = currPosMin = 0.f;
    currSpeed = currSpeedMax = 0.f;
    
    currSpeedKalman = 0.f;
    speedFilter.setup(0.f, 0.f, 0.f, 1e-4, 1e-2);
    
    currSpeedBiquad = 0.f;
    speedLowPassFc = 0.05;
    speedLowPassBiquad.setFc(speedLowPassFc);
    speedLowPassBiquad.setType(OFX_BIQUAD_TYPE_LOWPASS);
    
    roi.x = 640.f*0.3;
    roi.width = 640.f*0.3;
    roi.height = 480;
    
    setupPlot();
}

void KinectInput::setupPlot(){
    plot = new ofxHistoryPlot(240, true);
    plot->add(&currSpeed, "currSpeed", ofColor(0,125,255,125));
    plot->add(&currSpeedKalman, "currSpeedKalman", ofColor(0,125,255,255));
    plot->add(&currSpeedBiquad, "currSpeedBiquad", ofColor(0,255,125,255));
    plot->setRange(-0.1, 0.1);
    plot->setRespectBorders(true);
}

float KinectInput::update(){
    
    bool isFrameNew = false;
    
#ifdef USE_DEVICE
    if(prevKinectAngle != kinectAngle) {
        prevKinectAngle = kinectAngle;
    }
    kinect.update();
    if(kinect.isFrameNew()) {
        isFrameNew = true;
        depthImage.setFromPixels(kinect.getDepthPixels(), CAM_WIDTH, CAM_HEIGHT);
    }
#else
    kinectPlayer.update();
    if(kinectPlayer.isFrameNew()) {
        isFrameNew = true;
        videoTempImg.setFromPixels(kinectPlayer.getPixels(), CAM_WIDTH, CAM_HEIGHT, OF_IMAGE_COLOR);
        videoTempImg.setImageType(OF_IMAGE_GRAYSCALE);
        depthImage.setFromPixels(videoTempImg.getPixels(), CAM_WIDTH, CAM_HEIGHT);
    }
#endif
    
    if(isFrameNew) {
        
        CvRect cvROI = cvRect(roi.x,roi.y,roi.width,roi.height);
        cvSetImageROI(depthImage.getCvImage(),cvROI);
        
        // retrieve average depth
#ifdef TOP_TRACKING
        updateAvgX();
#else
        cvSetImageROI(thresholdImage.getCvImage(), cvROI);
        thresholdImage = depthImage;
        thresholdImage.threshold(threshold);
        //find blog
        contourFinder.findContours(thresholdImage, blobSizeMin, roi.width*roi.height, 1, false);
        fillBlobPoly();
        updateAvgDepth();
#endif
        
        currSpeed = (avgPos-avgPrevPos);//*0.55;
    }
    
    currSpeedKalman = speedFilter.update(currSpeed);
    
    speedLowPassBiquad.setFc(speedLowPassFc);
    currSpeedBiquad = speedLowPassBiquad.update(currSpeed);
    
    if(currSpeed > currSpeedMax) currSpeedMax = currSpeed;
    
    if(avgPos < currPosMin) currPosMin = avgPos;
    if(avgPos > currPosMax) currPosMax = avgPos;
    avgPrevPos = avgPos;
    
    return currSpeedBiquad;
}

void KinectInput::draw(){
    
    ofPushStyle();
    ofPushMatrix();
    ofTranslate(0, ofGetHeight()-depthImage.getHeight()*0.5);
    ofScale(0.5, 0.5);
    ofSetColor(255,255,255,255);
#ifdef USE_DEVICE
    kinect.drawDepth(0,0);
#else
    kinectPlayer.draw(0, 0);
#endif
    roi.draw(0, 0);

#ifdef TOP_TRACKING
    for (int i=0; i<lineMax.size(); i++) {
        ofSetColor(255, 255, 0);
//        ofLine(i+roi.x, lineMax[i], i+1+roi.x, lineMax[i+1]);
        ofRect(roi.x+lineMax[i], i+roi.y, 2, 2);//, lineMax[i+1]+10, i+1+roi.y);
    }
    ofSetColor(255, 0, 255);
    ofLine(roi.x-avgPos*CAM_WIDTH, roi.y, roi.x-avgPos*CAM_WIDTH, roi.y+roi.height);
#else
    if(contourFinder.nBlobs > 0 &&  contourFinder.blobs[0].area > blobSizeMin) {
        contourFinder.draw(roi.x, roi.y);
    }
#endif

    ofPopMatrix();
    
    plot->update();
    int w = depthImage.getWidth()*0.5;
    int h = depthImage.getHeight()*0.5;
    plot->draw(ofGetWidth()-w, ofGetHeight()-h, w, h);
    
    ofPopStyle();
}

//----
#ifdef TOP_TRACKING

void KinectInput::updateAvgX() {
    
    int buffw = 10;
    int pos;
    unsigned char * depthPixels = depthImage.getRoiPixels();
    
    lineMax.clear();
    lineMax.resize(roi.height);
    
    for(int j=0; j<roi.height; j++) {
        int currmax = 0;
        int currmaxpos = 0;
        for (int i=0; i<roi.width; i++) {
            pos = j*roi.width+i;
//            int val = 0;
//            int buffwsafe = min(i,buffw);
//            for (int k=0; k<buffwsafe; k++) {
//                if(depthPixels[pos-k]>threshold) {
//                    val += depthPixels[pos-k];
//                }
//            }
            int val = depthPixels[pos];
            if (val > threshold && val > currmax && val < 250) {
                currmax = val;
                currmaxpos = i;
            }
        }
        lineMax[j] = currmaxpos;
    }
    
    int total;
    for (int i=0; i<lineMax.size(); i++) {
        total += lineMax[i];
    }
    
    //    avgPos = contourFinder.blobs[0].centroid.x;
    avgPos = total / lineMax.size();
    avgPos /= CAM_WIDTH;
    avgPos *= -1;
}

#else

void KinectInput::fillBlobPoly() {
    
    cvSetZero( thresholdImage.getCvImage() );
    CvPoint* cPts;
    int nPts;
    if(contourFinder.nBlobs > 0 &&  contourFinder.blobs[0].area > blobSizeMin) {
        
        const ofxCvBlob& blob = contourFinder.blobs[0];
        
        // fill blob poly
        nPts = blob.nPts+1;
        cPts = new CvPoint[nPts];
        for(int i = 0; i < blob.nPts; i++)
        {
            cPts[i].x = blob.pts[i].x;
            cPts[i].y = blob.pts[i].y;
        }
        cPts[blob.nPts].x = blob.pts[0].x;
        cPts[blob.nPts].y = blob.pts[0].y;
        //delete cPts;
        
        bUserFound = bUserFound<0 ? 0 : bUserFound+1;
        
    } else {
        
        // fill ROI poly
        
        nPts = 4;
        cPts = new CvPoint[nPts];
        cPts[0].x = roi.x;           cPts[0].y = roi.y;
        cPts[1].x = roi.x+roi.width; cPts[1].y = roi.y;
        cPts[2].x = roi.x+roi.width; cPts[2].y = roi.y + roi.height;
        cPts[3].x = roi.x;           cPts[3].y = roi.y + roi.height;
        
        bUserFound = 0;
    }
    
    CvPoint * cPtss = cPts;
    cvFillPoly( thresholdImage.getCvImage(), &cPtss, &nPts, 1, cvScalar(1,255,255,255) );
    delete cPts;
}

void KinectInput::updateAvgDepth() {
    
    int pos;
    bool bInsideROI;
    int totalDepths=0;
    int numDepths=0;
    
    unsigned char * depthPixels = depthImage.getPixels();
    unsigned char * thresholdPixels = thresholdImage.getPixels();
    
    for (int i=0; i<CAM_WIDTH; i++) {
        
        for(int j=0; j<CAM_HEIGHT; j++) {
            
            pos = j*CAM_WIDTH+i;
            
            bInsideROI = (i>=roi.x) && (i<=roi.x+roi.width) && (j>=roi.y) && (j<=roi.y+roi.height);
            
            if( thresholdPixels[pos] > 0 && bInsideROI) {
                thresholdPixels[pos] = depthPixels[pos];
                totalDepths += depthPixels[pos];
                numDepths++;
            }
        }
    }
    avgPos = 1 - (float)totalDepths / (numDepths*255);
    
    // reset avgDepth if nan
    
    if(avgPos!=avgPos) avgPos = 0;
    
    thresholdImage.flagImageChanged();
}

#endif