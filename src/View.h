//
//  View.h
//  starfield_v2
//
//  Created by kikko on 28/09/14.
//
//

#ifndef __starfield_v2__View__
#define __starfield_v2__View__

#include "ofMain.h"
#include "ofxIO.h"
#include "ofxQuadWarp.h"


class Background {
    
public:
    void setup();
    void draw();
    void add(ofColor color, float percent);
    void setSize(float w, float h);
private:
    void fillMesh();
    ofVboMesh mesh;
};




struct less_than_z {
    inline bool operator() (const ofVec3f& p1, const ofVec3f& p2) {
        return (p1.z < p2.z);
    }
};


class Clouds {
    
public:
    void setup();
    void update(float cameraZ);
    void draw();
    
    void reloadTexture();
    
    float cloudSize;
    float galaxySize;
    int numClouds;
    float scale;
    ofVec3f offset;
    
private:
    
    ofVec3f getRandomPoint(float cameraZ = 0);
    
    ofVboMesh mesh;
    ofShader shader;
    vector <ofVec3f> points;
    vector <float> sizes;
    ofTexture texture;
};




class Stars {
    
public:
    void setup();
    void update(float cameraZ);
    void draw();
    
    float starSize;
    float galaxySize;
    int numStars;
    
private:
    ofVbo vbo;
    ofShader shader;
    vector <ofVec3f> points;
    vector <float> sizes;
    ofTexture texture;
};


typedef ofx::IO::DirectoryWatcherManager::DirectoryEvent DirectoryEvent;

class View {
    
public:
    
//    View();
    
    void setup();
    void update(float currSpeed);
    void draw();
    void exit();
    
    void setSize(float w, float h);
    
    //-- params
    
    bool bVisible;
    
    float offsetX;
    float offsetY;
    
    float speed;
    
    Background bg;
    Stars stars;
    Clouds clouds;
    
    ofImage moon;
    ofVec3f moonPos;
    float moonRot;
    float moonScale;
    
    float WarperX;
    ofxQuadWarp warperLeft;
    ofxQuadWarp warperRight;
    
    bool bDrawDebug;
    
    void onDirectoryWatcherItemModified(const DirectoryEvent & evt);
    
private:
    
    ofCamera camera;
    ofEasyCam cam;
    
    ofFbo fbo;
    
    float currSpeed;
    float cameraZ;
    
    void setupWarper(int srcW, int srcH);
    
    void drawMoon();
    
    bool bCloudsNeedReload;
    ofx::IO::DirectoryWatcherManager watcher;
};

#endif /* defined(__starfield_v2__View__) */
