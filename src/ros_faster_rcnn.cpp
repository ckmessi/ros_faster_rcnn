#include<iostream>

#include <ros/ros.h>

#include <image_transport/image_transport.h>

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace std;


// global variables
string g_window_name;
cv::Mat g_last_image;

void initWindow();
static void mouseCb(int event, int x, int y, int flags, void* param);
void imageCb(const sensor_msgs::ImageConstPtr& msg);

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

    // init image transport
    ros::NodeHandle nh;
    std::string topic = nh.resolveName("image");
    ros::NodeHandle local_nh("~");
    cout << topic << endl;
    std::string transport;
    local_nh.param("image_transport", transport, std::string("raw"));
    ROS_INFO_STREAM("Using transport \"" << transport << "\"");
    image_transport::ImageTransport it(local_nh);
    image_transport::TransportHints hints(transport, ros::TransportHints(), local_nh);
    image_transport::Subscriber sub = it.subscribe(topic, 1, imageCb, hints);

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
        cout << "left button click!" << endl;
    } 
    else if (event == cv::EVENT_RBUTTONDOWN) {
        // boost::mutex::scoped_lock lock(g_image_mutex);
        // start_detect = ~start_detect;
    }
    else{
        return;
    }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  // We want to scale floating point images so that they display nicely
  // bool do_dynamic_scaling = (msg->encoding.find("F") != std::string::npos);

    //msg_last_image = *msg;

    // boost::mutex::scoped_lock lock(g_image_mutex);

    // Convert to OpenCV native BGR color
    try {
      g_last_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR_THROTTLE(30, "Unable to convert '%s' image for display: '%s'", msg->encoding.c_str(), e.what());
    }
    
    /*
    if(start_detect == true && request_lock == false){     
    pthread_t tid;
        int ret = pthread_create( &tid, NULL, sendRcnnRequest, NULL );  
        if( ret != 0 )
        {  
            cout << "pthread_create error:error_code=" << ret << endl;  
        }
    }
    */


    if (!g_last_image.empty()) {
        // draw the box
    // cout << "detection num:" << d_list.size() << endl;
    /*
            for(unsigned int i = 0; i < d_list.size(); i++){
            cv::rectangle(g_last_image, cvPoint(d_list[i].x1, d_list[i].y1), cvPoint(d_list[i].x2, d_list[i].y2), Scalar(0,255,0),1,1,0);
            cv::putText(g_last_image, d_list[i].category, cvPoint(d_list[i].x1,d_list[i].y1), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,0), 1, CV_AA);
        }   
      */  
        const cv::Mat &image = g_last_image;
        cv::imshow(g_window_name, image);
    }
}    
