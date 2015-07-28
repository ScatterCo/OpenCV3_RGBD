#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "opencv2/rgbd.hpp"

class ofApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
	void loadDepthFrame(string file = "DepthDataSample/frame_03121_millis_0000112690.png");
	
protected:
	ofShortPixels depthFrame;
	cv::rgbd::RgbdNormals normalComputer;
};
