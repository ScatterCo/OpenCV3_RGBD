#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "opencv2/rgbd.hpp"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
    
    void keyPressed(int);
	
	void loadDepthFrame(string file = "DepthDataSample/frame_03121_millis_0000112690.png");	
	void computeNormals();
    
protected:
    int frameNum;
    int windowSize;
	
    vector<string> images;
	cv::Ptr<cv::rgbd::RgbdNormals> normalComputer;
    
	ofImage sprite;
    ofShader pointcloudShader;
    ofVboMesh points;
	
	ofShader normalShader;
    
    ofShortPixels inPixels;
	ofTexture pointCloud;
	ofTexture normalTex;
    
    ofEasyCam camera;
	bool normalsGenerated;
};