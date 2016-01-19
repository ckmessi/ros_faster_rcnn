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

#include "ObjectInfo.h"

using namespace cv;
using namespace std;


// global variables
string g_window_name;
cv::Mat g_last_image;
bool start_detect;
bool request_lock;
string temp_dir = "/home/chenkai/rosbuild_ws/package_dir/leo_detection/temp/";
ros::ServiceClient client;
vector<ObjectInfo> objectList;

// function claim
void initWindow();
static void mouseCb(int event, int x, int y, int flags, void* param);
void imageCb(const sensor_msgs::ImageConstPtr& msg);
void detect();
cv::Mat drawBoundingBox(cv::Mat image);
void* sendFasterRcnnRequest(void *args);


int main(int argc, char **argv)
{

    cout << "Hello, world!" << endl;
    ros::init(argc, argv, "ros_faster_rcnn", ros::init_options::AnonymousName);
    if (ros::names::remap("image") == "image") {
        ROS_WARN("Topic 'image' has not been remapped!");
    }

    // init window
    g_window_name = "windows for ros_faster_rcnn";
    initWindow();
    start_detect = false;
    request_lock = false;

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


    // initial the service client
    ros::NodeHandle n;
    client = n.serviceClient<ros_faster_rcnn::FasterRcnnDetection>("faster_rcnn_detection");



    // wait
    ros::spin();

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