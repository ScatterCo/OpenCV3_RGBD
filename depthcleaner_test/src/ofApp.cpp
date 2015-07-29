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
    
    for(int y = 0; y < 424; y += 1){
        for(int x = 0; x < 512; x += 1){
            points.addVertex(ofVec3f(x,y,0));
        }
    }
    points.setMode(OF_PRIMITIVE_POINTS);
    
    ofLoadImage(sprite, "dot.png");
    sprite.update();
    pointcloudShader.load("pointcloud");
    
    drawInPixels = true;
    cleaned = false;
    
    filterButton.addListener(this, &ofApp::filterButtonPressed);
    
    filterNumTimes = 1;
    windowSize = 5;
    
    gui.setup();
    gui.setPosition(10, 120);
    gui.add(filterSlider.setup("num passes", filterNumTimes, 1, 50));
    gui.add(filterButton.setup("filter"));
    
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
}

void ofApp::filterButtonPressed() {
    cleaned = false;
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
    
    if(!cleaned) {
        //frameNum = (frameNum + 1) % images.size();
        loadDepthFrame(images[frameNum]);
        
        if(!outPixels.isAllocated()) {
            outPixels.allocate(inPixels.getWidth(), inPixels.getHeight(), OF_IMAGE_GRAYSCALE);
        }
        
        depth_cleaner_ = new cv::rgbd::DepthCleaner(toCv(inPixels.getPixels()).depth(), windowSize, cv::rgbd::DepthCleaner::DEPTH_CLEANER_NIL);
        
        //(*depth_cleaner_)(toCv(inPixels.getPixels()), toCv(outPixels.getPixels()));
        depth_cleaner_->operator()(toCv(inPixels.getPixels()), toCv(outPixels.getPixels()));
  
        filterNumTimes = filterSlider;
        
        for(int i = 0; i < filterNumTimes; i++) {
            depth_cleaner_->operator()(toCv(outPixels.getPixels()), toCv(outPixels.getPixels()));
        }
        
        outPixels.update();

        ofShortPixels in = inPixels.getPixels();
        ofShortPixels out = outPixels.getPixels();
        
        numPixelsDiff = 0;
        
        for(int x = 0; x < in.getWidth(); x++) {
            for(int y = 0; y < in.getHeight(); y++) {
                ofColor inCol = in.getColor(x, y);
                ofColor outCol = out.getColor(x, y);
                
                ofColor diffCol = inCol - outCol;
                if(diffCol != ofColor(0,0,0)) {
                    numPixelsDiff += 1;
                }
            }
        }
    }
}

void ofApp::draw() {
    ofBackground(255);
    
    gui.draw();
    
    ofSetColor(0);
    
    string originalOrCleaned = drawInPixels ? "original" : "cleaned";
    
    ofDrawBitmapString("framenum: " + ofToString(frameNum) + "\n\n" +
                       "drawing: " + originalOrCleaned + "\n\n" +
                       "pixels changed: " + ofToString(numPixelsDiff) + "\n\n" +
                       "[w]indow size: " + ofToString(windowSize),
                       20, 20);
    
    if(outPixels.isAllocated()) {
        //outPixels.draw(outPixels.getWidth(), 0);
    }
    
    //WHY CANT I SEE THIS
    if(inPixels.isAllocated()) {
		//inPixels.draw(0, 0);
        
        ofTranslate(-ofGetWidth()/2,-ofGetHeight()/2);
        camera.begin();
        ofScale (1,-1,1);
        
        ofSetColor(255);
        pointcloudShader.begin();
        
        pointcloudShader.setUniformMatrix4f("calibration", ofMatrix4x4());
        pointcloudShader.setUniform2f("dimensions", inPixels.getWidth(), inPixels.getHeight());
        
        pointcloudShader.setUniform2f("fov", 364.885, 364.885); //notsure..
        pointcloudShader.setUniform2f("pp", 259.913, 205.322); //notsure..
        
        if(drawInPixels) {
            pointcloudShader.setUniformTexture("texture", inPixels, 1);
        }
        else {
            pointcloudShader.setUniformTexture("texture", outPixels, 1);
        }
        
        ofEnablePointSprites();
        pointcloudShader.setUniformTexture("sprite", sprite, 2);
        points.draw();
        ofDisablePointSprites();
        
        pointcloudShader.end();
        camera.end();
	}
}

void ofApp::keyPressed(int key) {
    if(key == OF_KEY_RIGHT) {
        frameNum = (frameNum + 1) % images.size();
        cleaned = false;
    }
    if(key == OF_KEY_LEFT) {
        frameNum = (frameNum - 1) % images.size();
        cleaned = false;
    }
    if(key == ' ') {
        drawInPixels = !drawInPixels;
    }
    if(key == 'w') {
        windowSize = ((windowSize + 2) % 8);
        cleaned = false;
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
	unsigned short* dst = inPixels.getPixels().getData();
	for(int i = 0; i < compressedImage.getWidth() * compressedImage.getHeight(); i++){
        *(dst++) = ((*src) << 8) | *(src+1); //scoot the pixels around
		src += 3;
	}
    
    inPixels.update();
    
    cleaned = true;
}











