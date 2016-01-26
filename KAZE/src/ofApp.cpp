// http://docs.opencv.org/3.0-beta/doc/tutorials/features2d/akaze_matching/akaze_matching.html#akaze

#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

void ofApp::setup() {
    ofSetWindowShape(1300, 500);
    
    matchRatio = 0.8;
    inlierThreshold = 2.5;
    
    loadFrames();
    detectMatches();
}

void ofApp::keyPressed(int key) {
    if(key == 'r') {
        matchRatio -= 0.05;
    }
    else if(key == 'R') {
        matchRatio += 0.05;
    }
    else if(key == 't') {
        inlierThreshold -= 0.05;
    }
    else if(key == 'T') {
        inlierThreshold += 0.05;
    }
    else {
        return;
    }
    
    matchRatio = MAX(MIN(matchRatio, 1), 0);
    detectMatches();
}

void ofApp::update() {
	ofSetWindowTitle(ofToString(ofGetFrameRate()));
}

void ofApp::detectMatches() {
    if(!colorFrame.isAllocated() || !irFrame.isAllocated()) {
        return;
    }
    
    cv::Mat colorMat = toCv(colorFrame);
    cv::Mat irMat = toCv(irFrame);
    
    vector<KeyPoint> colorKeypoints;
    vector<KeyPoint> irKeypoints;
    
    cv::Mat colorDescriptors;
    cv::Mat irDescriptors;

    cv::Ptr<cv::AKAZE> featureDetector = AKAZE::create();
    featureDetector->detectAndCompute(colorMat, noArray(), colorKeypoints, colorDescriptors);
    featureDetector->detectAndCompute(irMat, noArray(), irKeypoints, irDescriptors);
    
    cv::BFMatcher featureMatcher(NORM_HAMMING);
    vector< vector<DMatch> > matches;
    featureMatcher.knnMatch(colorDescriptors, irDescriptors, matches, 2);
    
    vector<KeyPoint> colorMatched, irMatched, colorInliers, irInliers;
    vector<DMatch> goodMatches;
    for(size_t i = 0; i < matches.size(); i++) {
        DMatch first = matches[i][0];
        float dist1 = matches[i][0].distance;
        float dist2 = matches[i][1].distance;
        
        if(dist1 < matchRatio * dist2) {
            colorMatched.push_back(colorKeypoints[first.queryIdx]);
            irMatched.push_back(irKeypoints[first.trainIdx]);
        }
    }
    
    for(unsigned i = 0; i < colorMatched.size(); i++) {
        Mat col = Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = colorMatched[i].pt.x;
        col.at<double>(1) = colorMatched[i].pt.y;
        
//        col = homography * col;
        col /= col.at<double>(2);
        double dist = sqrt( pow(col.at<double>(0) - irMatched[i].pt.x, 2) +
                           pow(col.at<double>(1) - irMatched[i].pt.y, 2));
        
        if(dist < inlierThreshold) {
            int new_i = static_cast<int>(colorInliers.size());
            colorInliers.push_back(colorMatched[i]);
            irInliers.push_back(irMatched[i]);
            goodMatches.push_back(DMatch(new_i, new_i, 0));
        }
    }
    
    cout << "Found " << goodMatches.size() << " good matches" << endl;
    
    matchedColorFeatures.clear();
    matchedIRFeatures.clear();
    for(int i = 0; i < goodMatches.size(); i++) {
        DMatch match = goodMatches[i];
        
        Point2f colorPointCV = colorInliers[match.queryIdx].pt;
        ofPoint colorPoint(colorPointCV.x, colorPointCV.y);
        
        Point2f irPointCV = irInliers[match.queryIdx].pt;
        ofPoint irPoint(irPointCV.x, irPointCV.y);
        
        matchedColorFeatures.push_back(colorPoint);
        matchedIRFeatures.push_back(irPoint);
    }
}

void ofApp::draw() {
    ofPushMatrix();
    ofTranslate(20, 0);
    
    int irOffset = 650;
    
	if(colorFrame.isAllocated()) {
		ofTexture tex;
		tex.loadData(colorFrame);
        ofSetColor(255);
        tex.draw(0, 0);
        
        for(int i = 0; i < matchedColorFeatures.size(); i++) {
            ofPoint pt = matchedColorFeatures[i];
            ofSetColor(0, 255, 0);
            ofNoFill();
            ofSetLineWidth(2);
            ofDrawCircle(pt, 5);
        }
	}
    
    if(irFrame.isAllocated()) {
        ofPushMatrix();
        ofTranslate(irOffset, 0);
        
        ofTexture tex;
        tex.loadData(irFrame);
        ofSetColor(255);
        tex.draw(0, 0);
        
        for(int i = 0; i < matchedIRFeatures.size(); i++) {
            ofPoint pt = matchedIRFeatures[i];
            ofSetColor(0, 255, 0);
            ofNoFill();
            ofSetLineWidth(2);
            ofDrawCircle(pt, 5);
        }
        
        ofPopMatrix();
    }
    
    for(int i = 0; i < matchedColorFeatures.size(); i++) {
        ofPoint colorPoint = matchedColorFeatures[i];
        ofPoint irPoint = matchedIRFeatures[i];
        irPoint.x += irOffset;
        ofEnableAlphaBlending();
        ofSetColor(0, 255, 0, 180);
        ofSetLineWidth(1);
        ofDrawLine(colorPoint, irPoint);
    }
    
    ofPopMatrix();
    
    ofSetColor(255);
    ofFill();
    ofDrawRectangle(0, 0, 250, 65);
    
    ofSetColor(0);
    ofDrawBitmapString("Match ratio: " + ofToString(matchRatio) + " (r/R)", 10, 10);
    ofDrawBitmapString("Inlier threshold: " + ofToString(inlierThreshold) + " (t/T)", 10, 25);
    ofDrawBitmapString(ofToString(matchedColorFeatures.size()) + " matches", 10, 40);
    
}

void ofApp::loadFrames() {
    string colorPath = ofToDataPath("COLOR.png");
    if(!ofLoadImage(colorFrame, colorPath)) {
        ofLogError("Could not load image at " + colorPath, "app");
        return;
    }
    
    string irPath = ofToDataPath("IR.png");
    if(!ofLoadImage(irFrame, irPath)) {
        ofLogError("Could not load image at " + irPath, "app");
        return;
    }
}