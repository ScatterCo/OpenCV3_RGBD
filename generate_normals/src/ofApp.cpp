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
	
	normalShader.load("normal");
    
    windowSize = 5;
	numDepthCleanPasses = true;
	drawTextures = false;
	useSRI = false;
	drawNormals = false;
	animateShader = true;
	loadDepthFrame(images[frameNum]);
	
//	camera.setOrientation(ofVec3f(0.0, 0.0, -90.0));
	ofVec3f position = ofVec3f(-200, -150, -750);
	ofMatrix4x4 transform = ofMatrix4x4( 
//										-0.0147771,	0.882499,	0.470081,	0,
//										0.971513,		0.123882,	-0.202028,	0,
//										-0.236525,		0.453705,	-0.85919,	0,
//										0,				0,			0,			1
										-0.118529, 0.949529, -0.290424,        0,
										0.99121, 0.0958363, -0.091204,        0,
										-0.0587677, -0.298681, -0.952542,        0,
										0,        0,        0,        1
										);
	
	ofQuaternion orientation;
	orientation.set(transform);
	camera.setPosition(position);
	camera.setOrientation(orientation);
	
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
	if(!normalsGenerated) {
		computeNormals();
	}
}


void ofApp::draw() {
    ofBackground(255);	
	
	if(drawTextures) {
		if(inPixels.isAllocated()) {
			ofSetColor(255, 255, 255);
			ofTexture inImage;
			inImage.loadData(inPixels);
			inImage.draw(20, 70);
		}
		
		if(normalTex.isAllocated()) {
			ofSetColor(255, 255, 255);
			
			normalShader.begin();
			normalTex.draw(552, 70);
			normalShader.end();
		}
	}
	
	ofPushMatrix();
	ofEnableDepthTest();
	ofTranslate(-ofGetWidth()/2,0);
	camera.begin();
	ofTranslate(-400, -200);
	
	pointcloudShader.begin();
	
	pointcloudShader.setUniformMatrix4f("calibration", ofMatrix4x4());
	pointcloudShader.setUniform2f("dimensions", pointCloud.getWidth(), pointCloud.getHeight());
	
	pointcloudShader.setUniform2f("fov", 364.885, 364.885); //notsure..
	pointcloudShader.setUniform2f("pp", 259.913, 205.322); //notsure..
	
	pointcloudShader.setUniformTexture("texture", pointCloud, 1);
	pointcloudShader.setUniformTexture("normalMap", normalTex, 2);
	pointcloudShader.setUniform1i("drawNormals", drawNormals);
	pointcloudShader.setUniform1f("t", animateShader ? ofGetElapsedTimef() : 0.0);
	
	ofEnablePointSprites();
	pointcloudShader.setUniformTexture("sprite", sprite, 3);
	points.draw();
	ofDisablePointSprites();
	
	pointcloudShader.end();
	camera.end();
	ofPopMatrix();
	
	
	ofSetColor(5, 200);
	ofEnableAlphaBlending();
	ofDrawRectangle(0, 0, ofGetWidth(), 75);
	
	ofSetColor(255);
	ofDrawBitmapString("framenum: [LEFT/RIGHT to change]	: " + ofToString(frameNum) + "\n" +
					   "window size [w to change]		: " + ofToString(windowSize) + "\n" +
					   "depth clean passes: [c/C to change]	: " + ofToString(numDepthCleanPasses) + "\n" +
					   "draw textures: [t to toggle]		: " + ofToString(drawTextures),
					   20, 20);
	
	string algorithmString =	"algorithm: FALS/SRI [a to toggle]	: " + ofToString(useSRI) + "\n" +
								"draw normals: [n to toggle]		: " + ofToString(drawNormals) + "\n" +
								"animate shader: [m to change]		: " + ofToString(animateShader);
	ofDrawBitmapString(algorithmString, 500, 20);
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
	if(key == 'c') {
		if(numDepthCleanPasses == 0) {
			return;
		}
		
		numDepthCleanPasses --;
		normalsGenerated = false;
	}
	if(key == 'C') {
		numDepthCleanPasses ++;
		normalsGenerated = false;
	}
	if(key == 't') {
		drawTextures = !drawTextures;
	}
	if(key == 'a') {
		useSRI = !useSRI;
		normalsGenerated = false;
	}
	if(key == 'n') {
		drawNormals = !drawNormals;
	}
	if(key == 'm') {
		animateShader = !animateShader;
	}
    
    if(key == 'p') {
        ofLog() << camera.getOrientationQuat();
        ofLog() << camera.getPosition();
    }
	if(key == 's') {
		ofSaveImage(normalPixels, ofToString(numDepthCleanPasses) + "-passes.tiff");
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

ofShortPixels ofApp::cleanDepth(ofShortPixels dirtyDepth, int numPasses) {
	ofShortPixels cleanedDepth;
	cleanedDepth.allocate(dirtyDepth.getWidth(), dirtyDepth.getHeight(), dirtyDepth.getImageType());
	depth_cleaner_ = new cv::rgbd::DepthCleaner(toCv(dirtyDepth).depth(), windowSize, cv::rgbd::DepthCleaner::DEPTH_CLEANER_NIL);
	
	depth_cleaner_->operator()(toCv(dirtyDepth), toCv(cleanedDepth));
	
	for(int i = 0; i < numPasses - 1; i++) {
		depth_cleaner_->operator()(toCv(cleanedDepth), toCv(cleanedDepth));
	}
	
	return cleanedDepth;
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

	ofShortPixels depth = inPixels;
	if(numDepthCleanPasses > 0) {
		depth = cleanDepth(inPixels, numDepthCleanPasses);
	}
	cv::rgbd::depthTo3d(toCv(depth), calibrationMatrix, pointsMat);
	int algorithm = cv::rgbd::RgbdNormals::RGBD_NORMALS_METHOD_FALS;
	if(useSRI) {
		algorithm = cv::rgbd::RgbdNormals::RGBD_NORMALS_METHOD_SRI;
	}
	
	normalComputer = new cv::rgbd::RgbdNormals(depth.getHeight(), depth.getWidth(), pointsMat.depth(), calibrationMatrix, windowSize, algorithm);
	normalComputer->operator()(pointsMat, normals);
	
	ofFloatPixels pointCloudPix;
	toOf(pointsMat, pointCloudPix);
	pointCloud.loadData(pointCloudPix);
	
	toOf(normals, normalPixels);
	normalTex.loadData(normalPixels);
	normalsGenerated = true;
}
