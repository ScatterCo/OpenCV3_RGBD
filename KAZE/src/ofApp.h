#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "opencv2/rgbd.hpp"
#include "opencv2/features2d.hpp"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    void keyPressed(int key);
	
    void loadFrames();
	
protected:
    ofPixels colorFrame;
    ofPixels irFrame;
    
    void detectMatches();
    vector<ofPoint> matchedColorFeatures;
    vector<ofPoint> matchedIRFeatures;
    
    float matchRatio;
    float inlierThreshold;
};
