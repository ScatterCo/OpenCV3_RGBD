#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {    
    ofDirectory dir;
    string folderPath = "DepthDataSample";
    dir.open(folderPath);
    int numFiles = dir.listDir();
    
    for (int i=0; i<numFiles; ++i)
    {
        images.push_back(dir.getPath(i));
    }
    
    frameNum = 0;
	normalsGenerated = false;
    
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
    
    windowSize = 5;
	loadDepthFrame(images[frameNum]);
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
	if(!normalsGenerated) {
		computeNormals();
	}
}


void ofApp::draw() {
    ofBackground(255);
    ofSetColor(0);
    
    ofDrawBitmapString("framenum: " + ofToString(frameNum) + "\n\n" +
                       "window size [w to change]: " + ofToString(windowSize),
                       20, 20);
	
	if(inPixels.isAllocated()) {
		ofSetColor(255, 255, 255);
		ofTexture inImage;
		inImage.loadData(inPixels);
		inImage.draw(20, 50);
	}
	
	if(normalTex.isAllocated()) {
		ofSetColor(255, 255, 255);
		normalTex.draw(552, 50);
	}
	
	ofTranslate(-ofGetWidth()/2,-ofGetHeight()/2);
	camera.begin();
	ofScale (1,-1,1);
	
	pointcloudShader.begin();
	
	pointcloudShader.setUniformMatrix4f("calibration", ofMatrix4x4());
	pointcloudShader.setUniform2f("dimensions", pointCloud.getWidth(), pointCloud.getHeight());
	
	pointcloudShader.setUniform2f("fov", 364.885, 364.885); //notsure..
	pointcloudShader.setUniform2f("pp", 259.913, 205.322); //notsure..
	
	pointcloudShader.setUniformTexture("texture", pointCloud, 1);
	
	ofEnablePointSprites();
	pointcloudShader.setUniformTexture("sprite", sprite, 2);
	points.draw();
	ofDisablePointSprites();
	
	pointcloudShader.end();
	camera.end();
}

void ofApp::keyPressed(int key) {
    if(key == OF_KEY_RIGHT) {
        frameNum = (frameNum + 1) % images.size();
        normalsGenerated = false;
		loadDepthFrame(images[frameNum]);
    }
    if(key == OF_KEY_LEFT) {
        frameNum = (frameNum - 1) % images.size();
		loadDepthFrame(images[frameNum]);
    }
    if(key == 'w') {
        windowSize = ((windowSize + 2) % 8);
        normalsGenerated = false;
    }
    
    if(key == 'p') {
        ofLog() << camera.getOrientationQuat();
        ofLog() << camera.getPosition();
    }
}

void ofApp::loadDepthFrame(string file) {
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
    
	normalsGenerated = false;
}

void ofApp::computeNormals() {	
	// prepare matrices structures
	cv::Mat pointsMat;
	cv::Mat normals;
	
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

	cv::rgbd::depthTo3d(toCv(inPixels), calibrationMatrix, pointsMat);
	normalComputer = new cv::rgbd::RgbdNormals(inPixels.getHeight(), inPixels.getWidth(), CV_32F, calibrationMatrix, windowSize, cv::rgbd::RgbdNormals::RGBD_NORMALS_METHOD_FALS);
	normalComputer->operator()(pointsMat, normals);
	
	ofFloatPixels pointCloudPix;
	toOf(pointsMat, pointCloudPix);
	pointCloud.loadData(pointCloudPix);
	
	ofFloatPixels normalPixels;
	toOf(normals, normalPixels);
	normalTex.loadData(normalPixels);
	normalsGenerated = true;
}
