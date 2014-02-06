#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv.h>
#include <cxcore.h>
#include <math.h>
#include <vector>
#include <stdio.h>

#include <iostream>
#include <fstream>

//using namespace cv;


// ----------
// Parameters
// ----------
static const std::string OPENCV_WINDOW_rgb = "Image window rgb";
static const std::string OPENCV_WINDOW_chess = "Image window chessboard";
static const std::string OPENCV_WINDOW_bird = "Image window birdseye";

const char* projMatFilename = "visLoc_TopviewCalib_ProjMat.txt";
const char* configFilename = "visLoc_TopviewCalib_Config.txt";


bool resizeTopviewOutput = false;
bool visCarPos = true;
static const int chessbrdCuboidWidth = 70;// in real world coords// width of chessboard cuboid in millimeters
int trgtChessbrdCuboidWidth = (int) chessbrdCuboidWidth / 7;// this defines the resolution of the result image--> 1 pixel = 7 millimeter
float srcTrgtRatio = (float)trgtChessbrdCuboidWidth / (float)chessbrdCuboidWidth;

cv::Point delta_chess0(200, 300);
cv::Point srcDelta_chess0ToCam(280, 700); // in millimeter // in real world coords
cv::Point delta_chess0ToCam(((int)(srcDelta_chess0ToCam.x * srcTrgtRatio)), ((int)(srcDelta_chess0ToCam.y * srcTrgtRatio)));

//calc car position in topview
cv::Point carPos(delta_chess0.x + delta_chess0ToCam.x, delta_chess0.y + delta_chess0ToCam.y);
static const cv::Size TOPVIEW_SIZE(500,400); // size of the topview image; be careful that the projection is less big ass the toview image
cv::Size PATTERNSIZE(8, 6);

//Hard coded intrinsics for the camera
cv::Mat intrinsicMat = (cv::Mat_<double>(3, 3) <<
        536.926104, 0.000000, 313.772698,
        0.000000, 536.726872, 240.243417,
        0.000000, 0.000000, 1.000000);

//Hard coded distortions for the camera
cv::Mat distortion = (cv::Mat_<double>(1, 4) << 0.038848, -0.122754, 0.000863, 0.000062);
    

class CalibBirdsEye{
     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_rgb;
     //image_transport::Publisher image_pub_rgb;
     //image_transport::Subscriber image_sub_depth;
     //image_transport::Publisher image_pub_depth;
     //cv::Mat depth_img_mono8;
     cv::Mat rgb_img, birds_img, chessbrd_img, H;     
     std::vector<cv::Point2f> corners;     

    public:
    CalibBirdsEye(): it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        //image_sub_rgb = it_.subscribe("/camera/rgb/image_color", 1, &CalibBirdsEye::imageMsgConvert_rgb, this);
        image_sub_rgb = it_.subscribe("/usb_cam/image_raw", 1, &CalibBirdsEye::imageMsgConvert_rgb, this);
        //image_pub_rgb = it_.advertise("/image_converter/image_rgb", 1);

        cv::namedWindow(OPENCV_WINDOW_rgb);
        cv::namedWindow(OPENCV_WINDOW_chess);
        cv::namedWindow(OPENCV_WINDOW_bird);
    }

    ~CalibBirdsEye()
    {
        cv::destroyWindow(OPENCV_WINDOW_rgb);
        cv::destroyWindow(OPENCV_WINDOW_chess);
        cv::destroyWindow(OPENCV_WINDOW_bird);
    }

    void imageMsgConvert_rgb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        rgb_img = cv::Mat(cv_ptr->image.size(), CV_8UC3);
        cv_ptr->image.copyTo(rgb_img);


        //process on input
        processImage(rgb_img);

        //image_sub_rgb.shutdown();

        // Output modified video stream
        //image_pub_rgb.publish(cv_ptr->toImageMsg());cv::FileStorage file("some_name.ext", cv::FileStorage::WRITE);
    }

    void processImage(cv::Mat& inputImg)
    {
        //init images
        chessbrd_img = inputImg.clone();
        birds_img = inputImg.clone();

        // DETECT Chessboard
        bool patternfound;
        detectChessboard(inputImg, corners, patternfound, PATTERNSIZE);
        if(patternfound)
        {
            cv::drawChessboardCorners(chessbrd_img, PATTERNSIZE, cv::Mat(corners), patternfound);
            calcBirdsEyeTransform(H, corners, PATTERNSIZE, delta_chess0, trgtChessbrdCuboidWidth);
            birdsEyeTransform(chessbrd_img, birds_img, H, TOPVIEW_SIZE);
            postprocessTopview(birds_img, resizeTopviewOutput, carPos, visCarPos);
        }
        else
        {
            // clear projMat
            //H = cv::Mat::zeros(H.rows,H.cols, H.type());
        }



        // ------
        // OUTPUT
        // ------
        cv::imshow(OPENCV_WINDOW_rgb, inputImg);
        cv::imshow(OPENCV_WINDOW_chess, chessbrd_img);
        cv::imshow(OPENCV_WINDOW_bird, birds_img);

        char k;
        k = cv::waitKey(3000);  // wait 3 sec before next image
        //printf("%d\n", k);
        if(k == 10) // save if "Enter" is clicked
        {
            // save images + H + CalibConfig
            cv::imwrite("visLoc_TopviewCalib_BirdsEye.jpg", birds_img);
            cv::imwrite("visLoc_TopviewCalib_ChessboardCalib.jpg", chessbrd_img);            
            writeDoubleMatToFile(H, projMatFilename);
            writeCalibConfigToFile(TOPVIEW_SIZE, carPos, PATTERNSIZE, delta_chess0, delta_chess0ToCam, configFilename);
        }
    }


    void writeDoubleMatToFile(cv::Mat& m, const char* filename)
    {
        std::ofstream fout(filename);
        if(!fout)
        {
            std::cout<<"File Not Opened"<<std::endl;  return;
        }
        for(int i=0; i<m.rows; i++)
        {
            for(int j=0; j<m.cols; j++)
            {
                fout<<m.at<double>(i,j)<<"\t";
            }
            fout<<std::endl;
        }
        fout.close();

        ROS_INFO_STREAM("Saved projection matrix to home folder.");
    }


    void writeCalibConfigToFile(const cv::Size& topviewSize, cv::Point& carPos, cv::Size& patternsize, cv::Point& delta_chess0, cv::Point& delta_chess0ToCam, const char* filename)
    {
        std::ofstream fout(filename);
        if(!fout)
        {
            std::cout<<"File Not Opened"<<std::endl;  return;
        }

        //write topviewsize
        fout<<topviewSize.width<<"\t";
        fout<<topviewSize.height<<std::endl;        

        //write carPos
        fout<<carPos.x<<"\t";
        fout<<carPos.y<<std::endl;

        //write patternsize
        fout<<patternsize.width<<"\t";
        fout<<patternsize.height<<std::endl;

        //write delta_chess0
        fout<<delta_chess0.x<<"\t";
        fout<<delta_chess0.y<<std::endl;

        //write delta_chess0ToCam
        fout<<delta_chess0ToCam.x<<"\t";
        fout<<delta_chess0ToCam.y<<std::endl;


        fout.close();
        ROS_INFO_STREAM("Saved calibration config to home folder.");
    }


    void detectChessboard(cv::Mat& inputImg, std::vector<cv::Point2f>& dstCorners, bool& dstPatternfound, cv::Size& patternsize )
    {
        // find corners
        dstPatternfound = cv::findChessboardCorners(inputImg,patternsize, dstCorners, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE);
        // if found, do it more precise
        if(dstPatternfound)
        {
            //cornersOut.clear();
            cv::Mat gray_image;
            cv::cvtColor(inputImg, gray_image, CV_RGB2GRAY );
            cv::cornerSubPix(gray_image, dstCorners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        }
        else
        {
            //clear Corners if not found
            dstCorners.clear();
        }        
    }

    void birdsEyeTransform(cv::Mat& inputImg, cv::Mat& outputImg, cv::Mat& projMat, const cv::Size& topviewSize)
    {
        // transformate image
        cv::warpPerspective(inputImg, outputImg, projMat, topviewSize, cv::INTER_LINEAR /*cv::INTER_CUBIC*/ | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, CV_RGB(0,0,255) ) ;
    }

    void postprocessTopview(cv::Mat& dstImg, bool& resizeOutput, cv::Point& carPos, bool& visCarPos)
    {
        if(visCarPos)
        {
//            ROS_INFO_STREAM("carpos: " << (int)carPos.x << ", " << (int)carPos.y);
//            ROS_INFO_STREAM("deltachess0tocam: " << (int)delta_chess0ToCam.x << ", " << (int)delta_chess0ToCam.y);
//            ROS_INFO_STREAM("deltachess0: " << (int)delta_chess0.x << ", " << (int)delta_chess0.y);
            cv::circle(dstImg, carPos, 5, CV_RGB(0,255,0), -1);
        }
        if(resizeOutput)
        {
            cv::resize(dstImg, dstImg, cv::Size(), .5, .5 );
        }
    }

    void calcBirdsEyeTransform(cv::Mat& dstMat, std::vector<cv::Point2f>& corners, cv::Size& patternsize, cv::Point& delta_chess0, const int& chessbrdCuboidWidth)
    {        
        // set Projection
        cv::Point2f objPts[4], imgPts[4];

        // target points in topview
        objPts[0].x = delta_chess0.x ;
        objPts[0].y = delta_chess0.y ;
        objPts[1].x = delta_chess0.x + (patternsize.width-1) * chessbrdCuboidWidth ;
        objPts[1].y = delta_chess0.y ;
        objPts[2].x = delta_chess0.x ;
        objPts[2].y = delta_chess0.y + (patternsize.height-1) * chessbrdCuboidWidth ;
        objPts[3].x = delta_chess0.x + (patternsize.width-1) * chessbrdCuboidWidth ;
        objPts[3].y = delta_chess0.y + (patternsize.height-1) * chessbrdCuboidWidth ;

        // points in camera view
        imgPts[0] = corners[0];
        imgPts[1] = corners[patternsize.width-1];
        imgPts[2] = corners[(patternsize.height-1)*patternsize.width];
        imgPts[3] = corners[(patternsize.height-1)*patternsize.width + patternsize.width-1];

        //FIND THE HOMOGRAPHY
        dstMat = cv::getPerspectiveTransform( objPts, imgPts ) ;
        //ROS_INFO_STREAM("test: " << dstMat.type() << " " << dstMat.channels());

;
    }    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calib_birds_eye");
    CalibBirdsEye calibBE;
    ros::spin();
    return 0;
}

