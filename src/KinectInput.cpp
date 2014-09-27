//
//  KinectInput.cpp
//  starfield_v2
//
//  Created by kikko on 26/09/14.
//
//

#include "KinectInput.h"

KinectInput::KinectInput(){
    
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
    kinectPlayer.loadMovie("kinect_depth.mov");
    kinectPlayer.play();
    videoTempImg.allocate(CAM_WIDTH, CAM_HEIGHT, OF_IMAGE_COLOR);
#endif
    
    depthImage.allocate(CAM_WIDTH, CAM_HEIGHT);
    thresholdImage.allocate(CAM_WIDTH, CAM_HEIGHT);
    
    kinectAngle = prevKinectAngle = 0;
    threshold = 10;
    bUserFound = 0;
    avgDepth = avgDepthSmoothed = 0.f;
    blobSizeMin = 5000.f;
    
    currPosMin = currPosMin = 0.f;
    currSpeed = currSpeedMax = 0.f;
    
    roi.x = 640.f*0.3;
    roi.width = 640.f*0.3;
    roi.height = 480;
}

void KinectInput::update(){
    
    bool isFrameNew = false;
    
#ifdef USE_DEVICE
    if(prevKinectAngle != kinectAngle) {
        //kinect.setCameraTiltAngle(kinectAngle);
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
        cvSetImageROI(thresholdImage.getCvImage(), cvROI);
        
        thresholdImage = depthImage;
        thresholdImage.threshold(threshold);
        
        //find blog
        
        contourFinder.findContours(thresholdImage, 0, roi.width*roi.height, 1, false);
        
        fillBlobPoly();
        
        // retrieve average depth
        
        updateAvgDepth();
    }
    
    // smooth it's value
    
    float delta = (avgDepth-avgDepthSmoothed)*0.15;
    avgDepthSmoothed += delta;
    
    currSpeed += (delta-currSpeed)*0.15;
    
    if(currSpeed > currSpeedMax) currSpeedMax = currSpeed;
    if(avgDepthSmoothed < currPosMin) currPosMin = avgDepthSmoothed;
    if(avgDepthSmoothed > currPosMax) currPosMax = avgDepthSmoothed;
}

void KinectInput::draw(){
    
    ofPushMatrix();
    ofScale(0.5, 0.5);
    depthImage.draw(0,0);
    roi.draw(0, 0);
    ofTranslate(0, depthImage.getHeight());
    thresholdImage.draw(0, 0);
    if(contourFinder.nBlobs > 0 &&  contourFinder.blobs[0].area > blobSizeMin) {
        contourFinder.draw(roi.x, roi.y);
    }
    roi.draw(0, 0);
    ofPopMatrix();
    
    ostringstream log;
    log << "Avg Depth : " << avgDepth << "\n";
    log << "Curr Speed : " << currSpeed;
    ofDrawBitmapString(log.str(), depthImage.getWidth()*0.5+10, 15);
}

//----

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
    avgDepth = 1 - (float)totalDepths / (numDepths*255);
    
    // reset avgDepth if nan
    
    if(avgDepth!=avgDepth) avgDepth = 0;
    
    
    thresholdImage.flagImageChanged();
}
