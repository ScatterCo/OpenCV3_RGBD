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
	
	void loadDepthFrames();
	ofShortPixels loadDepthFrame(string file = "DepthDataSample/frame_03121_millis_0000112690.png");	
	void computeOdometry();
    
protected:
	cv::Ptr<cv::rgbd::ICPOdometry> odometryComputer;
    
	ofImage sprite;
    ofShader pointcloudShader;
    ofVboMesh points;
	ofTexture pointCloud;
    
    ofShortPixels southPixels;
	ofShortPixels westPixels;
	bool odometryComputed;

    ofEasyCam camera;
	
	ofShader depthDrawingShader;
};