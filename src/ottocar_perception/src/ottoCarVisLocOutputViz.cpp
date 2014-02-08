#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float64MultiArray.h>

#include <cv.h>
#include <cxcore.h>
#include <math.h>
#include <vector>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <ctime>


// ----------
// Parameters
// ----------

static const std::string OPENCV_WINDOW_rgb = "ottoCar_perception Output";

    

class OttoCarVisLocOutputViz{

     ros::NodeHandle nh_;     
     ros::Subscriber perfSub;
     ros::Subscriber laneStateSub;
     std_msgs::Float64MultiArray perfArray;
     std_msgs::Float64MultiArray laneStateArray;

     cv::Mat img;
     float perf_proj;
     float perf_perception;
     float perf_all;

     ros::Time inputTimestamp;

     //detectLaneMarksV2

    public:
    OttoCarVisLocOutputViz()
    {        
        img = cv::Mat::zeros(300,300, CV_8UC3);

        perfSub = nh_.subscribe<std_msgs::Float64MultiArray>("/ottocar_perception/perf", 100, performanceCallback);
        laneStateSub = nh_.subscribe<std_msgs::Float64MultiArray>("/ottocar_perception/laneState", 100, laneStateCallback);


        cv::namedWindow(OPENCV_WINDOW_rgb);

    }

    ~OttoCarVisLocOutputViz()
    {

        cv::destroyWindow(OPENCV_WINDOW_rgb);
    }

    void laneStateCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
    {
        double timestamp = array->data.[0];
    }

    void performanceCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
    {
        double timestamp = array->data.[0];
    }




};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visLocOutputViz");
    OttoCarVisLocOutputViz visLocOutputViz;
    ros::spin();
    return 0;
}

