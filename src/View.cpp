//
//  View.cpp
//  starfield_v2
//
//  Created by kikko on 28/09/14.
//
//

#include "View.h"



//---- Background


void Background::setup(){
    
    float w = ofGetWidth(), h = ofGetHeight();
    mesh.clear();
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    mesh.setUsage(GL_STREAM_DRAW);
    
    fillMesh();
}

void Background::setSize(float w, float h){
    fillMesh();
}

void Background::fillMesh(){
    mesh.clear();
    add(ofColor(  0,   3,  15), 0);
    add(ofColor(  0,   0,  67), 0.3);
//    add(ofColor( 50, 179, 255), 0.5);
    add(ofColor( 36, 139, 187), 0.6);
//    add(ofColor(  54, 164, 202), 1);
    add(ofColor(  0,   50,  67), 1);
}

void Background::add(ofColor color, float percent){
    
    mesh.addVertex(ofVec2f(0, (float)ofGetHeight() * percent));
    mesh.addVertex(ofVec2f(ofGetWidth(), (float)ofGetHeight() * percent));
    mesh.addColor(color);
    mesh.addColor(color);
}

void Background::draw(){
    
    GLboolean depthMaskEnabled;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMaskEnabled);
    glDepthMask(GL_FALSE);
    mesh.draw();
    if(depthMaskEnabled){
        glDepthMask(GL_TRUE);
    }
}




//---- Clouds


void Clouds::setup(){
    ofLoadImage(texture, "cloud.png");
    shader.load("cloud");
    
    GLint err = glGetError();
    if (err != GL_NO_ERROR){
        ofLogNotice() << "Load Shader came back with GL error:	" << err;
    }
    
    scale = 60;
    int w = texture.getWidth();
    int h = texture.getHeight();
    
    mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);
    mesh.setUsage(GL_STATIC_DRAW);
    
    mesh.addVertex(ofVec2f(0, 0));
    mesh.addVertex(ofVec2f(w*scale, 0));
    mesh.addVertex(ofVec2f(0, h*scale));
    mesh.addVertex(ofVec2f(w*scale, h*scale));
    
//    w=1; h=1;
    mesh.addTexCoord(ofVec2f(0, h));
    mesh.addTexCoord(ofVec2f(w, h));
    mesh.addTexCoord(ofVec2f(0, 0));
    mesh.addTexCoord(ofVec2f(w, 0));
    
    offset.set(0,-7000,0);
    
    cloudSize = 9182.f;
    numClouds = 100;
    galaxySize = 20000;
}

void Clouds::update(float cameraZ){
//    ofLog() << cameraZ;
    
    for (int i=0; i<points.size(); i++) {
        if( points[i].z > cameraZ + 5000 ) {
            points.erase(points.begin()+i);
            sizes.erase(sizes.begin()+i);
            i--;
        }
    }
    
    ofPoint p;
    for (int i=points.size(); i<numClouds; i++) {
        float offSet = -2500;
        p = getRandomPoint(cameraZ);
        points.push_back(p);
        sizes.push_back(cameraZ-p.z);
    }
    
    std::sort(points.begin(), points.end(), less_than_z());
}

ofVec3f Clouds::getRandomPoint(float cameraZ){
    float offw = texture.getWidth() * scale * 0.5;
    float offh = texture.getHeight() * scale * 0.5;
//    ofLog() << cameraZ;
    return ofVec3f(ofRandomf()*40000 - offw,  ofRandomf()*2000 - offh, cameraZ-ofRandom((cameraZ>-1?-5000:50000),100000));
}

void Clouds::reloadTexture(){
    ofLoadImage(texture, "cloud.png");
}

void Clouds::draw(){
    
    shader.begin();
    
//    ofLog() << points.size();
    shader.setUniform3fv("pos", &points[0].x, numClouds);
    texture.bind();
    
    ofPushMatrix();
    ofTranslate(offset);
    
//    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
//    ofEnableAlphaBlending();
    mesh.drawInstanced(OF_MESH_FILL, numClouds);
//    mesh.draw();
//    ofDisableAlphaBlending();
//    ofDisableBlendMode();
    
    ofPopMatrix();
    
    texture.unbind();
    shader.end();
}



//---- Stars


void Stars::setup(){
    ofLoadImage(texture, "star.png");
    shader.load("star");
    
    starSize = 256.0;
    numStars = 1400;
    galaxySize = 15000;
}

void Stars::update(float cameraZ){
    
    for (int i=0; i<points.size(); i++) {
        if( points[i].z > (cameraZ+10000) ) {
            points.erase(points.begin()+i);
            sizes.erase(sizes.begin()+i);
            i--;
        }
    }
    
    ofPoint p;
    for (int i=points.size(); i<numStars; i++) {
        float offSet = 0;
        p = ofPoint(ofRandom(-galaxySize, galaxySize), ofRandom(offSet, galaxySize), cameraZ-ofRandom(-10000, 70000));
        points.push_back(p);
        sizes.push_back(cameraZ-p.z);
    }
    
    int total = (int)points.size();
    vbo.setVertexData(&points[0], total, GL_DYNAMIC_DRAW);
}

void Stars::draw(){
    
    ofEnablePointSprites();
    
    shader.begin();
    shader.setUniform1f("starSize", starSize);
    texture.bind();
    
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    
    vbo.draw(GL_POINTS, 0, (int)points.size());
    
    ofDisableBlendMode();
    
    texture.unbind();
    shader.end();
    
    ofDisablePointSprites();
}



//---- View


void View::setup(){
    
    camera.setFarClip(100000);
    
    bVisible = true;
    
    cameraZ = 0;
    
    speed = 4000;
    offsetX = 0.53;
    offsetY = 0.5;
    
    stars.setup();
    clouds.setup();
    bg.setup();
    
//    moonPos.set(0.5,0.5);
    moonRot = 90;
    moonScale = 1;
    moon.loadImage("moon.png");
    bCloudsNeedReload = true;
    
    ofAddListener(watcher.events.onItemModified, this, &View::onDirectoryWatcherItemModified);
    
    std::string folderToWatch = ofToDataPath("", true);
    bool listExistingItemsOnStart = true;
    
    WarperX = 0.5;
    
    setSize(ofGetWidth(), ofGetHeight());
    
    bDrawDebug = false;
    
    watcher.addPath(folderToWatch, listExistingItemsOnStart);
}

void View::setSize(float w, float h){
    bg.setSize(w,h);
    fbo.allocate(w, h);
    setupWarper(w, h);
}

void View::setupWarper(int srcW, int srcH){
    warperLeft.setSourceRect(ofRectangle(0, 0, srcW*0.5, srcH));
    warperLeft.setTopLeftCornerPosition(ofPoint(0, 0));
    warperLeft.setTopRightCornerPosition(ofPoint(srcW*0.5, 0));
    warperLeft.setBottomLeftCornerPosition(ofPoint(0, srcH));
    warperLeft.setBottomRightCornerPosition(ofPoint(srcW*0.5, srcH));
    warperLeft.setAnchorSize(25);
    warperLeft.setup();
    warperLeft.load("quarwarp_left");
    
    warperRight.setSourceRect(ofRectangle(srcW*0.5, 0, srcW*0.5, srcH));
    warperRight.setTopLeftCornerPosition(ofPoint(srcW*0.5, 0));
    warperRight.setTopRightCornerPosition(ofPoint(srcW, 0));
    warperRight.setBottomLeftCornerPosition(ofPoint(srcW*0.5, srcH));
    warperRight.setBottomRightCornerPosition(ofPoint(srcW, srcH));
    warperRight.setAnchorSize(25);
    warperRight.setup();
    warperRight.load("quarwarp_right");
}

void View::exit(){
    warperLeft.save("quarwarp_left");
    warperRight.save("quarwarp_right");
}

void View::update(float currSpeed){

    this->currSpeed = currSpeed;
    
    cameraZ += -currSpeed * (currSpeed>0?2:1) * speed;
    
    stars.update(cameraZ);
    clouds.update(cameraZ);
    
    if(!bVisible) return;
}

void View::draw(){
    
    if(!bVisible) return;
    
    fbo.begin();
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA,GL_ONE,GL_ONE_MINUS_SRC_ALPHA);
    {
        ofPushStyle();
        
        ofSetColor(255, 255, 255, 255);

        bg.draw();
        drawMoon();
        
        if(bCloudsNeedReload){
            clouds.reloadTexture();
            bCloudsNeedReload = false;
        }
        
        camera.setPosition(0, 0, cameraZ);
        camera.setLensOffset(ofVec2f(offsetX, offsetY));
        camera.begin();
        
        clouds.draw();
        stars.draw();
        
        camera.end();

        ofPopStyle();
    }
    glDisable(GL_BLEND);
    glPopAttrib();
    fbo.end();
    
    
//    setupWarper(ofGetWidth(), ofGetHeight());
    ofMatrix4x4 mat = warperLeft.getMatrix();
    ofPushMatrix();
    ofMultMatrix(mat.getPtr());
    fbo.draw(0,0);
    ofPopMatrix();
    
//    ofPushView();
//    ofViewport(ofRectangle(ofGetWidth()*0.5, 0, ofGetWidth(), ofGetHeight()));
    mat = warperRight.getMatrix();
    ofPushMatrix();
    ofMultMatrix(mat.getPtr());
    ofTranslate(fbo.getWidth()*0.5, 0);
    fbo.getTextureReference().drawSubsection(0, 0, fbo.getWidth()*0.5, fbo.getHeight(), fbo.getWidth()*0.5, 0);
    ofPopMatrix();
//    ofPopView();
    
    if(bDrawDebug){
        warperLeft.draw();
        warperRight.draw();
    }
}

void View::drawMoon(){
    
    ofPushMatrix();
    ofTranslate(ofGetWidth()*moonPos.x, ofGetHeight()*moonPos.y);
    ofRotate(moonRot);
    
    ofPushStyle();
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    moon.draw(- moon.getWidth()*moonScale*0.5, - moon.getHeight()*moonScale*0.5, moon.getWidth()*moonScale, moon.getHeight()*moonScale);
    ofPopStyle();
    
    ofPopMatrix();
}

void View::onDirectoryWatcherItemModified(const DirectoryEvent& evt)
{
    string filename = ofSplitString(evt.item.path(), "/").back();
    if (filename == "cloud.png"){
        bCloudsNeedReload = true;
    }
    ofLog() << "Modified: " + evt.item.path();
}