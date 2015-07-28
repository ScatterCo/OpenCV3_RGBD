#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
	
	loadDepthFrame();
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

void ofApp::draw() {
	if(depthFrame.isAllocated()) {
		ofTexture tex;
		tex.loadData(depthFrame);
		tex.draw(0, 0);
	}
}


void ofApp::loadDepthFrame(string file) {
	string path = ofToDataPath(file);
	ofPixels compressedImage;
	
	if(!ofLoadImage(compressedImage, path)) {
		ofLogError("Could not load image at " + path, "app");
		return;
	}
	
	depthFrame.allocate(compressedImage.getWidth(), compressedImage.getHeight(), OF_IMAGE_GRAYSCALE);
	
	unsigned char* src = compressedImage.getData(); //this holds the RGB png loaded from disk
	unsigned short* dst = depthFrame.getData(); //this is an ofShortPixels / OF_IMAGE_GRAYSCALE of the same w/h as compressedImage
	for(int i = 0; i < compressedImage.getWidth() * compressedImage.getHeight(); i++){
		*(dst++) = ((*src) << 8) | *(src+1); //scoot the pixels around
		src += 3;
	}
}