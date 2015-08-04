#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {    
	odometryComputed = false;
    
    for(int y = 0; y < 424; y += 1){
        for(int x = 0; x < 512; x += 1){
            westPoints.addVertex(ofVec3f(x,y,0));
        }
    }
    westPoints.setMode(OF_PRIMITIVE_POINTS);
	
	for(int y = 0; y < 424; y += 1){
		for(int x = 0; x < 512; x += 1){
			southPoints.addVertex(ofVec3f(x,y,0));
		}
	}
	southPoints.setMode(OF_PRIMITIVE_POINTS);
	
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
	
	enableAlignment = true;
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
	
	
	ofTranslate(-ofGetWidth()/2,-ofGetHeight()/2);
	camera.begin();
	ofScale (1,-1,1);
	ofTranslate(-400, -200);
	
	pointcloudShader.begin();
	
	pointcloudShader.setUniformMatrix4f("calibration", ofMatrix4x4());
	pointcloudShader.setUniform2f("dimensions", westPointCloud.getWidth(), westPointCloud.getHeight());
	
	pointcloudShader.setUniform2f("fov", 364.885, 364.885); //notsure..
	pointcloudShader.setUniform2f("pp", 259.913, 205.322); //notsure..
	
	pointcloudShader.setUniformTexture("texture", westPointCloud, 1);
	
	ofEnablePointSprites();
	pointcloudShader.setUniformTexture("sprite", sprite, 2);
	westPoints.draw();
	ofDisablePointSprites();
	
	pointcloudShader.end();
	
	ofMatrix4x4 transform;
	if(enableAlignment) {
		transform = odometryTransform;
	}
	pointcloudShader.begin();
	
	pointcloudShader.setUniformMatrix4f("calibration", transform);
	pointcloudShader.setUniform2f("dimensions", southPointCloud.getWidth(), southPointCloud.getHeight());
	
	pointcloudShader.setUniform2f("fov", 364.885, 364.885); //notsure..
	pointcloudShader.setUniform2f("pp", 259.913, 205.322); //notsure..
	
	pointcloudShader.setUniformTexture("texture", southPointCloud, 1);
	
	ofEnablePointSprites();
	pointcloudShader.setUniformTexture("sprite", sprite, 2);
	southPoints.draw();
	ofDisablePointSprites();
	
	pointcloudShader.end();
	
	
	camera.end();
}

void ofApp::keyPressed(int key) {    
    if(key == 'p') {
        ofLog() << camera.getOrientationQuat();
        ofLog() << camera.getPosition();
    }
	if(key == ' ') {
		enableAlignment = !enableAlignment;
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
	
	cv::Mat transform(4, 4, CV_32F);
	cv::Mat westMat = toCv(westPixels);
	cv::Mat southMat = toCv(southPixels);
//	westMat.convertTo(westMat, CV_32FC1);
//	southMat.convertTo(southMat, CV_32FC1);
	cv::rgbd::rescaleDepth(westMat, CV_32FC1, westMat);
	cv::rgbd::rescaleDepth(southMat, CV_32FC1, southMat);
	
	if(!cv::rgbd::isValidDepth(*westMat.data)) {
		ofLogError("odometry", "West matrix is not valid depth");
	}
	
	if(!cv::rgbd::isValidDepth(*southMat.data)) {
		ofLogError("odometry", "South matrix is not valid depth");
	}
	
	bool res = odometryComputer->compute(cv::Mat(), westMat, cv::Mat(), cv::Mat(), southMat, cv::Mat(), transform);
	if(!res) {
		ofLogError("odometry", "Error computing odometry");
	}	
	
	cv::Mat westPointsMat;
	cv::Mat southPointsMat;
	cv::rgbd::depthTo3d(toCv(westPixels), calibrationMatrix, westPointsMat);
	cv::rgbd::depthTo3d(toCv(southPixels), calibrationMatrix, southPointsMat);
	
	ofFloatPixels westPointsPix;
	toOf(westPointsMat, westPointsPix);
	westPointCloud.loadData(westPointsPix);
	
	ofFloatPixels southPointsPix;
	toOf(southPointsMat, southPointsPix);
	southPointCloud.loadData(southPointsPix);
	
	transform.convertTo(transform, CV_32F);
//	odometryTransform = toOf(transform);
	odometryTransform = ofMatrix4x4((float*)transform.data);
	
	cout << transform << endl;
	cout << odometryTransform << endl;
	
	odometryComputed = true;
}