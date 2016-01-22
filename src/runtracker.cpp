#include "ros/ros.h"
#include "ros_faster_rcnn/KcfTracker.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "kcftracker.hpp"

#include <dirent.h>

#include <vector>

using namespace std;
using namespace cv;

// global variable 
KCFTracker g_tracker;
cv::Mat frame;

int rundemo(int argc, char *argv[]);

string convertToRectStr(int x, int y, int width, int height){
	
	string result = "";

	string temp = "";
	stringstream ss;
	ss.clear();
	ss << x;
	ss >> temp;
	result += temp + ",";
	ss.clear();
	ss << y;
	ss >> temp;
	result += temp + ",";
	ss.clear();
	ss << width;
	ss >> temp;
	result += temp + ",";
	ss.clear();
	ss << height;
	ss >> temp;
	result += temp;

	return result;
}

bool track(ros_faster_rcnn::KcfTracker::Request  &req, ros_faster_rcnn::KcfTracker::Response &res)
{
	string command = req.command;
	string path = req.path;
	string rect = req.rectangle;
	string x = command + "|" + path + "|" + rect;
	cout << x << endl;
	// ROS_INFO("request: c=%s, p=%s, r=%s", command.c_str(), path.c_str(), rectangle.c_str());
	// ROS_INFO("sending back response: [%ld]", (long int)res.sum);


	string frameName = path;
	frame = imread(frameName, CV_LOAD_IMAGE_COLOR);

	float xMin, yMin, width, height;
	int j = 0;
    int lastPos = 0;
	vector<string> list;
	while(j < (int)rect.length()){
		if(rect[j] == ','){
			string temp = rect.substr(lastPos, j-lastPos);
			list.push_back(temp);
			lastPos = j+1;			
		}
		j++;
    }	
 	string temp = rect.substr(lastPos);
    list.push_back(temp);
	
	
	if(command == "initial"){
		xMin = (float)atoi(list[0].c_str());
		yMin = (float)atoi(list[1].c_str());
		width = (float)atoi(list[2].c_str());
		height = (float)atoi(list[3].c_str());
		g_tracker.init( Rect(xMin, yMin, width, height), frame );
		string detect_rect = convertToRectStr(xMin, yMin, xMin+width, yMin+height);
		res.detection_info = detect_rect;
	}
	else{
		Rect result = g_tracker.update(frame);
		string detect_rect = convertToRectStr(result.x, result.y, result.x+result.width, result.y +result.height);
		res.detection_info = detect_rect;		
	}

	res.debug_info = "debug_info";
	return true;

}


int main(int argc, char* argv[]){
	
	/*
	int ret = rundemo(argc, argv);
	return ret;
	*/
	
	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	//bool SILENT = true;
	bool LAB = false;
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);
	g_tracker = tracker;
	


	ros::init(argc, argv, "kcf_tracker_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("kcf_tracker", track);
	ROS_INFO("Ready to tracker.");
  	ros::spin();
	return 0;

}


int rundemo(int argc, char *argv[])
{
	if (argc > 5) return -1;

	bool HOG = true;
	bool FIXEDWINDOW = false;
	bool MULTISCALE = true;
	bool SILENT = true;
	bool LAB = false;

	for(int i = 0; i < argc; i++){
		if ( strcmp (argv[i], "hog") == 0 )
			HOG = true;
		if ( strcmp (argv[i], "fixed_window") == 0 )
			FIXEDWINDOW = true;
		if ( strcmp (argv[i], "singlescale") == 0 )
			MULTISCALE = false;
		if ( strcmp (argv[i], "show") == 0 )
			SILENT = false;
		if ( strcmp (argv[i], "lab") == 0 ){
			LAB = true;
			HOG = true;
		}
		if ( strcmp (argv[i], "gray") == 0 )
			HOG = false;
	}
	
	// Create KCFTracker object
	KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

	// Frame readed
	Mat frame;

	// Tracker results
	Rect result;

	// Path to list.txt
	ifstream listFile;
	string fileName = "images.txt";
  	listFile.open(fileName);
	if(!listFile){
		cout << "image file open fail" << endl;
        }

  	// Read groundtruth for the 1st frame
  	ifstream groundtruthFile;
	string groundtruth = "region.txt";
  	groundtruthFile.open(groundtruth);
	if(!groundtruthFile){
		cout << "ground file open fail" << endl;
	}
  	string firstLine;
  	getline(groundtruthFile, firstLine);
	groundtruthFile.close();
  	
  	istringstream ss(firstLine);

  	
	// Using min and max of X and Y for groundtruth rectangle
	// float xMin =  min(x1, min(x2, min(x3, x4)));
	// float yMin =  min(y1, min(y2, min(y3, y4)));
	// float width = max(x1, max(x2, max(x3, x4))) - xMin;
	// float height = max(y1, max(y2, max(y3, y4))) - yMin;
	float xMin, yMin, width, height;
	ss >> xMin;
	ss >> yMin;
	ss >> width;
	ss >> height;
	cout << xMin << "," << yMin << "," << width << "," << height << endl;
		
	
	// Read Images
	ifstream listFramesFile;
	string listFrames = "images.txt";
	listFramesFile.open(listFrames);
	string frameName;


	// Write Results
	ofstream resultsFile;
	string resultsPath = "output.txt";
	resultsFile.open(resultsPath);

	// Frame counter
	int nFrames = 0;


	while ( getline(listFramesFile, frameName) ){
		frameName = frameName;

		// Read each frame from the list
		frame = imread(frameName, CV_LOAD_IMAGE_COLOR);

		// First frame, give the groundtruth to the tracker
		if (nFrames == 0) {
			tracker.init( Rect(xMin, yMin, width, height), frame );
			rectangle( frame, Point( xMin, yMin ), Point( xMin+width, yMin+height), Scalar( 0, 255, 255 ), 1, 8 );
			resultsFile << xMin << "," << yMin << "," << width << "," << height << endl;
		}
		// Update
		else{
			result = tracker.update(frame);
			rectangle( frame, Point( result.x, result.y ), Point( result.x+result.width, result.y+result.height), Scalar( 0, 255, 255 ), 1, 8 );
			resultsFile << result.x << "," << result.y << "," << result.width << "," << result.height << endl;
		}

		nFrames++;

		if (!SILENT){
			imshow("Image", frame);
			waitKey(1);
		}
	}
	resultsFile.close();
	listFile.close();
	return 0;
}
