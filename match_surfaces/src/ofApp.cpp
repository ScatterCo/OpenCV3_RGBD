#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {    
	odometryComputed = false;
    
    for(int y = 0; y < 424; y += 1){
        for(int x = 0; x < 512; x += 1){
            points.addVertex(ofVec3f(x,y,0));
        }
    }
    points.setMode(OF_PRIMITIVE_POINTS);
    
    ofDisableArbTex();
    ofLoadImage(sprite, "dot.png");
    sprite.update();
    ofEnableArbTex();
    pointcloudShader.load("pointcloud");
	
	depthDrawingShader.load("depth");
	
	loadDepthFrames();
	
	ofVec3f position = ofVec3f(-209.753, 402.35, -761.939);
	ofMatrix4x4 transform = ofMatrix4x4( 
										-0.0147771,	0.882499,	0.470081,	0,
										0.971513,		0.123882,	-0.202028,	0,
										-0.236525,		0.453705,	-0.85919,	0,
										0,				0,			0,			1
										);
	
	ofQuaternion orientation;
	orientation.set(transform);
	camera.setPosition(position);
	camera.setOrientation(orientation);
	
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
	if(!odometryComputed) {
		computeOdometry();
	}
}


void ofApp::draw() {
    ofBackground(255);
    ofSetColor(0);
	
	if(westPixels.isAllocated()) {
		ofSetColor(255, 255, 255);
		ofTexture westImage;
		westImage.loadData(westPixels);
		
		depthDrawingShader.begin();
		westImage.draw(20, 20, 400, 300);
		depthDrawingShader.end();
	}
	
	if(southPixels.isAllocated()) {
		ofSetColor(255, 255, 255);
		ofTexture southImage;
		southImage.loadData(southPixels);
		depthDrawingShader.begin();
		southImage.draw(440, 20, 400, 300);
		depthDrawingShader.end();
	}
	
	/*
	ofTranslate(-ofGetWidth()/2,-ofGetHeight()/2);
	camera.begin();
	ofScale (1,-1,1);
	ofTranslate(-400, -200);
	
	pointcloudShader.begin();
	
	pointcloudShader.setUniformMatrix4f("calibration", ofMatrix4x4());
	pointcloudShader.setUniform2f("dimensions", pointCloud.getWidth(), pointCloud.getHeight());
	
	pointcloudShader.setUniform2f("fov", 364.885, 364.885); //notsure..
	pointcloudShader.setUniform2f("pp", 259.913, 205.322); //notsure..
	
	pointcloudShader.setUniformTexture("texture", pointCloud, 1);
	
	ofEnablePointSprites();
	pointcloudShader.setUniformTexture("sprite", sprite, 3);
	points.draw();
	ofDisablePointSprites();
	
	pointcloudShader.end();
	camera.end();
	 */
}

void ofApp::keyPressed(int key) {    
    if(key == 'p') {
        ofLog() << camera.getOrientationQuat();
        ofLog() << camera.getPosition();
    }
}

void ofApp::loadDepthFrames() {
	southPixels = loadDepthFrame("south.png");
	westPixels = loadDepthFrame("west.png");
}

ofShortPixels ofApp::loadDepthFrame(string file) {
	ofShortPixels inPixels;
	string path = ofToDataPath(file);
	ofPixels compressedImage;
	
	if(!ofLoadImage(compressedImage, path)) {
		ofLogError("Could not load image at " + path, "app");
		return;
	}
	
    if(!inPixels.isAllocated()) {
        inPixels.allocate(compressedImage.getWidth(), compressedImage.getHeight(), OF_IMAGE_GRAYSCALE);
    }
        
	unsigned char* src = compressedImage.getData(); //this holds the RGB png loaded from disk
	unsigned short* dst = inPixels.getData();
	for(int i = 0; i < compressedImage.getWidth() * compressedImage.getHeight(); i++){
        *(dst++) = ((*src) << 8) | *(src+1); //scoot the pixels around
		src += 3;
	}
    
	return inPixels;
}

void ofApp::computeOdometry() {
	// calibration matrix
	cv::Mat calibrationMatrix(3, 3, CV_32F);
	calibrationMatrix.at<float>(0, 0) = 364.825;
	calibrationMatrix.at<float>(0, 1) = 0.0;
	calibrationMatrix.at<float>(0, 2) = 256.594;
	calibrationMatrix.at<float>(1, 0) = 0.0;
	calibrationMatrix.at<float>(1, 1) = 364.825;
	calibrationMatrix.at<float>(1, 2) = 205.189;
	calibrationMatrix.at<float>(2, 0) = 0.0;
	calibrationMatrix.at<float>(2, 1) = 0.0;
	calibrationMatrix.at<float>(2, 2) = 1.0;
	
	odometryComputer = new cv::rgbd::ICPOdometry(calibrationMatrix);
//	odometryComputer->setMaxRotation(360);
//	odometryComputer->setMaxTranslation(1000.0);
//	odometryComputer->setMaxDepth(1000);
//	odometryComputer->setMaxDepthDiff(1000);
//	odometryComputer->setTransformType(cv::rgbd::Odometry::RIGID_BODY_MOTION);
	
	cv::Mat transform;
	cv::Mat westMat = toCv(westPixels);
	cv::Mat southMat = toCv(southPixels);
	westMat.convertTo(westMat, CV_32FC1);
	southMat.convertTo(southMat, CV_32FC1);
	
	if(!cv::rgbd::isValidDepth(*westMat.data)) {
		ofLogError("odometry", "West matrix is not valid depth");
	}
	
	if(!cv::rgbd::isValidDepth(*southMat.data)) {
		ofLogError("odometry", "South matrix is not valid depth");
	}
	
	bool res = odometryComputer->compute(cv::Mat(), westMat, cv::Mat(), cv::Mat(), southMat, cv::Mat(), transform);
	if(res) {
		cout << transform << endl;
	}
	else {
		ofLogError("odometry", "Error computing odometry");
	}
	odometryComputed = true;
}