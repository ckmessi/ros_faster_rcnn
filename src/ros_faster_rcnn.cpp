#include<iostream>
#include<vector>
#include <ros/ros.h>

#include <image_transport/image_transport.h>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// system
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>


#include "ros_faster_rcnn/FasterRcnnDetection.h"
#include "ros_faster_rcnn/KcfTracker.h"

#include "ObjectInfo.h"

#define MODE_DETECT 1
#define MODE_TRACK 2
#define MODE_TEST 99
#define DATA_CAMERA 1
#define DATA_VIDEO 2


using namespace cv;
using namespace std;


// global variables
string g_window_name;
cv::Mat g_last_image;
bool start_detect;
bool request_lock;
// modify this path
string temp_dir = "/home/chenkai/rosbuild_ws/package_dir/ros_faster_rcnn/temp/";
ros::ServiceClient client;
vector<ObjectInfo> objectList;

ros::ServiceClient track_client;
string track_command = "";
bool start_track = false;
bool track_request_lock;


int mode = MODE_TRACK;//MODE_TRACK;	// 1.detect; 2.track;
int data_source = DATA_CAMERA;
string video_path = "";

// function claim
void initWindow();
static void mouseCb(int event, int x, int y, int flags, void* param);
void imageCb(const sensor_msgs::ImageConstPtr& msg);
void detect();
cv::Mat drawBoundingBox(cv::Mat image);
void* sendFasterRcnnRequest(void *args);

void track(string command);
void* sendKcfTrackerRequest(void *args);
string convertToRectStr(int x, int y, int width, int height);


int main(int argc, char **argv)
{

    cout << "Hello, world!" << endl;

	// parse the parameter
	for(int i = 0; i < argc; i++){
		if(strcmp(argv[i], "-m") == 0){
			if(i + 1 < argc){
				mode = atoi(argv[i+1]);
			}
			else{
				cout << "parameter error" << endl;
				return 0;
			}
		}
		if(strcmp(argv[i], "-d") == 0){
			if(i + 1 < argc){
				data_source = atoi(argv[i+1]);
			}
			else{
				cout << "parameter error" << endl;
				return 0;
			}
		}
		if(strcmp(argv[i], "-path") == 0){
			if(i + 1 < argc){
				video_path = argv[i+1];
			}
			else{
				cout << "parameter error" << endl;
				return 0;
			}
		}
	}
	
	
	// init window
	g_window_name = "windows for ros_faster_rcnn";
	initWindow();
	start_detect = false;
	request_lock = false;
	start_track = false;
	track_request_lock = false;

	// ros initial
	ros::init(argc, argv, "ros_faster_rcnn", ros::init_options::AnonymousName);
	

	// initial the service client
	ros::NodeHandle n;
	client = n.serviceClient<ros_faster_rcnn::FasterRcnnDetection>("faster_rcnn_detection");
	track_client = n.serviceClient<ros_faster_rcnn::KcfTracker>("kcf_tracker");

	
	
	if(data_source == DATA_VIDEO){ 
		// video
		
		CvCapture* capture = cvCreateFileCapture(video_path.c_str());  
		IplImage* frame;  
		while(1)  
		{  
			frame = cvQueryFrame(capture);  
			if(!frame) 
				break;
			g_last_image = cv::Mat(frame);
			const cv::Mat &image = g_last_image;
			if(start_detect == true && request_lock == false){
        		detect();
    		}
			// track the object
			if(start_track == true && track_request_lock == false){
				track("update");
			}    

   		 	if (!g_last_image.empty()) {
		        // draw bounding box
        		g_last_image = drawBoundingBox(g_last_image);
        		const cv::Mat &image = g_last_image;
        		cv::imshow(g_window_name, image);
    		}
			char c = cvWaitKey(33);  
			if(c == 27)  
				break;  
		}  
		cvReleaseCapture(&capture); 
	}
	else if (data_source == DATA_CAMERA) {
		// camera
		
		if (ros::names::remap("image") == "image") {
		    ROS_WARN("Topic 'image' has not been remapped!");
		}

		// init image transport
		ros::NodeHandle nh;
		std::string topic = nh.resolveName("image");
		ros::NodeHandle local_nh("~");
		std::string transport;
		local_nh.param("image_transport", transport, std::string("raw"));
		ROS_INFO_STREAM("Using transport \"" << transport << "\"");
		image_transport::ImageTransport it(local_nh);
		image_transport::TransportHints hints(transport, ros::TransportHints(), local_nh);
		image_transport::Subscriber sub = it.subscribe(topic, 1, imageCb, hints);
	
		// wait
		ros::spin();
	}
	else {
		cout << "no data source specified..." << endl;
	}

	// final process
	cv::destroyWindow(g_window_name);
	
    return 0;   
}


// initial the display window
void initWindow()
{
    cv::namedWindow(g_window_name, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(g_window_name, &mouseCb);
    // Start the OpenCV window thread so we don't have to waitKey() somewhere
    cv::startWindowThread();
}



// mouse event handle function
static void mouseCb(int event, int x, int y, int flags, void* param){

	if(mode == MODE_DETECT){
		if (event == cv::EVENT_LBUTTONDOWN) {
		    cout << "left button click: detect once." << endl;
		    request_lock = false;
		    detect();			
 		} 
    	else if (event == cv::EVENT_RBUTTONDOWN) {
		    cout << "right button click: touch the detect switch." << endl;
		    // boost::mutex::scoped_lock lock(g_image_mutex);
		    start_detect = !start_detect;
		    cout << start_detect << endl;
    	}
		else{
		    return;
		}
	}	
	else if (mode == MODE_TRACK){
		if (event == cv::EVENT_LBUTTONDOWN) {
		    cout << "left button click: detect once." << endl;
		    request_lock = false;
			start_track = false;
		    detect();			
 		} 
    	else if (event == cv::EVENT_RBUTTONDOWN) {			
			cout << "right button click: update tracker switch" << endl;
			if(objectList.size() > 0){
				int target = 0;
				for(int i = 0; i < objectList.size(); i++){
					if((x >= objectList[i].x1) && (x <= objectList[i].x2) & (y >= objectList[i].y1) && (y <= objectList[i].y2)){
						target = i;
						break;
					}
				}
				ObjectInfo targetOb = objectList[target];
				objectList.clear();
				objectList.push_back(targetOb);
				track("initial");
				start_track = !start_track;
			}
			else{
				cout << "no object exists!" << endl;
			}
			

    	}
		else{
		    return;
		}
	}
	else if (mode == MODE_TEST){
		if (event == cv::EVENT_LBUTTONDOWN) {
		    cout << "left button click: detect once." << endl;
		    request_lock = false;
		    detect();	
 		}
		else if (event == cv::EVENT_RBUTTONDOWN){
			if(objectList.size() > 0){
				if((x >= objectList[0].x1) && (x <= objectList[0].x2) & (y >= objectList[0].y1) && (y <= objectList[0].y2)){				
					cout << "hit the object!" << endl;	
				}
				else{
					cout << "no hit" << endl;
				}
			}
		}		
	}
	else{
		return;
	}

    
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    // boost::mutex::scoped_lock lock(g_image_mutex);
    // Convert to OpenCV native BGR color
	try {
        g_last_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
    
    // detect the object
    if(start_detect == true && request_lock == false){
        detect();
    }
	// track the object
	if(start_track == true && track_request_lock == false){
		track("update");
	}    

    if (!g_last_image.empty()) {
        // draw bounding box
        g_last_image = drawBoundingBox(g_last_image);
        const cv::Mat &image = g_last_image;
        cv::imshow(g_window_name, image);
    }
}    

void detect()
{  
    pthread_t tid;
    int ret = pthread_create( &tid, NULL, sendFasterRcnnRequest, NULL);  
    if( ret != 0 )
    {  
        cout << "pthread_create error:error_code=" << ret << endl;  
    }
}

cv::Mat drawBoundingBox(cv::Mat image)
{
    // draw the box
    for(unsigned int i = 0; i < objectList.size(); i++){
        cv::rectangle(image, cvPoint(objectList[i].x1, objectList[i].y1), cvPoint(objectList[i].x2, objectList[i].y2), Scalar(0,255,0),1,1,0);
        cv::putText(image, objectList[i].category, cvPoint(objectList[i].x1,objectList[i].y1), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);
    }   
    return image;
}

void* sendFasterRcnnRequest(void *args){
    const cv::Mat &image = g_last_image;
    if (image.empty()) {
        ROS_WARN("Couldn't save image, no data!");
    }
    std::string filename = "0000.jpg";
    string filepath = temp_dir + filename;    
    if (cv::imwrite(filepath, image)) {
        ROS_INFO("Saved image %s", filepath.c_str());
        ros_faster_rcnn::FasterRcnnDetection srv;
        srv.request.path = filepath;
        request_lock = true;
        cout << "lock switch to true: request_lock=" << request_lock << endl;
        if(client.call(srv))
        {
            string debug_info = srv.response.debug_info;
            cout << "debug_info:" << debug_info << endl;
            string detection_info = srv.response.detection_info;
            cout << "detection_info:" << detection_info <<endl;
            objectList.clear();
            objectList = parseObjectInfoList(detection_info, ',');

        }
        else{
            ROS_ERROR("Failed to call service faster_rcnn_detection");
        }
        request_lock = false;
        cout << "lock switch to false: request_lock=" << request_lock << endl;

    }
    else {
        boost::filesystem::path full_path = boost::filesystem::complete(filepath);
        ROS_ERROR_STREAM("Failed to save image. Have permission to write there?: " << full_path);
    }

    void *ret;
    return ret;
}

void track(string command){
	if(command == "initial"){
		track_command = "initial";
	}
	else if (command == "update"){
		track_command = "update";
	}
	else{
		return;
	}
    pthread_t tid;
    int ret = pthread_create( &tid, NULL, sendKcfTrackerRequest, NULL);  
    if( ret != 0 )
    {  
        cout << "pthread_create error:error_code=" << ret << endl;  
    }
}

void* sendKcfTrackerRequest(void *args){

    const cv::Mat &image = g_last_image;
    if (image.empty()) {
        ROS_WARN("Couldn't save image, no data!");
    }
    std::string filename = "0000.jpg";
    string filepath = temp_dir + filename;    
    if (cv::imwrite(filepath, image)) {
        ROS_INFO("Saved image %s", filepath.c_str());
        ros_faster_rcnn::KcfTracker srv;
		srv.request.command = track_command;
        srv.request.path = filepath;
		srv.request.rectangle = "0,0,1,1";
		if(objectList.size() > 0){
			srv.request.rectangle = convertToRectStr(objectList[0].x1, objectList[0].y1,objectList[0].x2-objectList[0].x1, objectList[0].y2-objectList[0].y1);
		}
        track_request_lock = true;
        cout << "lock switch to true: track_request_lock=" << track_request_lock << endl;
        if(track_client.call(srv))
        {
			
            string debug_info = srv.response.debug_info;
            cout << "debug_info:" << debug_info << endl;
            string detection_info = srv.response.detection_info;
            cout << "detection_info:" << detection_info <<endl;
            // objectList.clear();
            // objectList = parseObjectInfoList(detection_info, ',');
			detection_info += ",person";
			objectList.clear();
            objectList = parseObjectInfoList(detection_info, ',');
			
        }
        else{
            ROS_ERROR("Failed to call service kcf_tracker");
        }
        track_request_lock = false;
        cout << "lock switch to false: track_request_lock=" << track_request_lock << endl;

    }
    else {
        boost::filesystem::path full_path = boost::filesystem::complete(filepath);
        ROS_ERROR_STREAM("Failed to save image. Have permission to write there?: " << full_path);
    }
    void *ret;
    return ret;
}


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


