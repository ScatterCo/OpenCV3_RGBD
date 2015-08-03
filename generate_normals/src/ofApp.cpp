#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	loadDepthFrame();
    
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
    
    ofMatrix4x4 mat = ofMatrix4x4(-0.00185878, -0.999998,    -0.000887482, 0.0,
                                  -0.999575,    0.00188394,  -0.0290703,   0.0,
                                  0.0290719,   0.000833068, -0.999577,    0.0,
                                  0.0,         0.0,          0.0,         1.0);
    ofQuaternion orientation;
    orientation.set(mat);
    
    ofVec3f pos;
    pos = ofVec3f(-4.17757, -0.130167, 151.032);
    
    camera.setPosition(pos);
    camera.setOrientation(orientation);
	
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
		inImage.draw(20, 20);
	}
	
	if(outPixels.isAllocated()) {
		ofSetColor(255, 255, 255);
		ofTexture outImage;
		outImage.loadData(outPixels);
		outImage.draw(530, 20);
	}
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
	normalComputer = new cv::rgbd::RgbdNormals(inPixels.getWidth(), inPixels.getHeight(), CV_32F, calibrationMatrix, windowSize, cv::rgbd::RgbdNormals::RGBD_NORMALS_METHOD_FALS);
	normalComputer->operator()(pointsMat, normals);
	
	toOf(normals, outPixels);
	normalsGenerated = true;
}
