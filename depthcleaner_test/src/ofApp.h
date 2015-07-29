#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "opencv2/rgbd.hpp"
#include "ofxGui.h"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    
    void keyPressed(int);
	
	void loadDepthFrame(string file = "DepthDataSample/frame_03121_millis_0000112690.png");
    
    void filterButtonPressed();
	
    
protected:
    int filterNumTimes;
    int frameNum;
    int numPixelsDiff;
    int windowSize;
    int maxNumPasses;
    vector<string> images;
    cv::rgbd::RgbdNormals normalComputer;
    cv::Ptr<cv::rgbd::DepthCleaner> depth_cleaner_;
    ofImage sprite;
    ofShader pointcloudShader;
    ofVboMesh points;
    
    ofShortImage inPixels;
    ofShortImage outPixels;
    
    ofxIntSlider filterSlider;
    ofxButton filterButton;
    ofxPanel gui;
    
    ofEasyCam camera;
    bool drawInPixels;
    bool cleaned;
};