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

bool debugMode = true; // flags for cv::windows
bool vizMode = true; // flag for visualization
static const std::string OPENCV_WINDOW_rgb = "rgb input";
static const std::string OPENCV_WINDOW_topview = "topview";
static const std::string OPENCV_WINDOW_topview_roi = "topview ROI + line chain creation";
static const std::string OPENCV_WINDOW_lines = "line detection ";
static const std::string OPENCV_WINDOW_lines_morph = "line detection morphed";
static const std::string OPENCV_WINDOW_lanes_hor = "lane state";
static const std::string OPENCV_WINDOW_lanes_vert = "lines final";

const char* projMatFilename = "/home/ottocar/visLoc_TopviewCalib_ProjMat.txt";
const char* configFilename = "/home/ottocar/visLoc_TopviewCalib_Config.txt";

bool resizeTopviewOutput = false;
bool visCarPos = false;

// Lane Mark Detection
int NEIGHBOR_KERNEL_SIZE = 20;
cv::Scalar TOPVIEW_BACKGROUND = CV_RGB(180,180,180);
int TOPVIEW_BACKGROUND_GRAY = (int) (TOPVIEW_BACKGROUND[0] + TOPVIEW_BACKGROUND[1] + TOPVIEW_BACKGROUND[2])/ 3;
int LINE_DETECTION_THRESHOLD = 30;
float LINECHAIN_MAX_APPEND_DIST = 15.0;
cv::Size roiSize(300, 300); //(width, height)
cv::Point roiPos(100, 100);
cv::Rect processRoi(roiPos.x, roiPos.y, roiSize.width, roiSize.height);
bool useProcessingRoi = true;

// Lane Model

int LANE_DETECTION_LANE_WIDTH;// = 57; // lane width in pixel of topview
int LANE_DETECTION_POS_DELTA = 10; // position delta in +-pixel of topview
float LANE_DETECTION_ANGLE_DELTA = 10.0; // angle delta in +-degree

int LANE_STATE_CANDIDATE_POS_DELTA = 20; //position delta in +-pixel of topview for setting lane state candidates
float LANE_STATE_CANDIDATE_ANGLE_DELTA = 20.0; //angle delta in +-degree between candidate and last lane state





enum LineType {LANE_BORDER_CONTINOUS=0, LANE_BORDER_DASHED=1, FALSE_POSITIVE=2};
    

class OttoCarVisLoc{

     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_rgb;
     image_transport::Publisher image_pub_bw;
     image_transport::Publisher image_pub_rgb;
     ros::Publisher perfPub;
     ros::Publisher laneStatePub;
     ros::Publisher laneLeftPub;
     ros::Publisher laneMidPub;
     ros::Publisher laneRightPub;
     std_msgs::Float64MultiArray perfArray;
     std_msgs::Float64MultiArray laneStateArray;

     std_msgs::Float64MultiArray laneLeftArray;
     std_msgs::Float64MultiArray laneMidArray;
     std_msgs::Float64MultiArray laneRightArray;

     cv::Mat rgb_img, topviewRoi_img, topview_img, H, lines_img, linesMorph_img, lanes_img_hor,lanes_img_vert, meanQuadraticNeighborKernel, meanHorizontalNeighborKernel;
     cv::Size topviewSize;
     cv::Point carPos;
     float PIXEL_TO_CENTIMETER;

     ros::Time inputTimestamp;

     //detectLaneMarksV2
     std::vector<std::vector<cv::Point> > lineChainList, polylineList, polylineListDashed, polylineListFinal ;
     std::vector<std::pair<float, float> > polyLinesParamList; // <lineLength, angleChangeConf>
     std::vector<LineType> polyLineTypeList;

     typedef struct StateStruct_
     {
         cv::Point projectedToBaseLine;
         cv::Point2f direction;
         float confidence;
         LineType linetype;
         std::vector<cv::Point> points;
     } StateStruct;
     StateStruct CurrMidLaneState;
     typedef std::vector<StateStruct> ListOfStateStructs;
     ListOfStateStructs PolyLineStateStructList, BorderLeftCandidateList, BorderMidCandidateList, BorderRightCandidateList, LaneStateStructCandidate, LaneStateStruct_current, LaneStateStruct_last; // Laneborder_left ; LaneBorder_mid; Laneborder_right
     // point; vec; confidence ; linetype

    public:
    OttoCarVisLoc(): it_(nh_)
    {

        // load CalibConfig and TopviewProjMat
        H.create(3,3,CV_64F);
        loadDoubleMatFromFile(H, projMatFilename, 3, 3);
        loadCalibConfigFromFile(topviewSize, carPos, configFilename);

        LANE_DETECTION_LANE_WIDTH = (int)round((double)(41.0 / PIXEL_TO_CENTIMETER));

        if(useProcessingRoi)
        {
            carPos.x = carPos.x - roiPos.x;
            carPos.y = carPos.y - roiPos.y;
        }

        LaneStateStruct_current.clear();
        LaneStateStruct_last.clear();

        // Subscrive to input video feed and publish output
        image_sub_rgb = it_.subscribe("/usb_cam/image_raw", 1, &OttoCarVisLoc::imageMsgConvert_rgb, this);

        image_pub_bw = it_.advertise("ottocar_perception/laneStateViz/bw", 1);
        image_pub_rgb = it_.advertise("ottocar_perception/laneStateViz/rgb", 1);
        perfPub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/perf", 1);
        laneStatePub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/laneState", 1);
        laneLeftPub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/lanes/left", 1);
        laneMidPub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/lanes/mid", 1);
        laneRightPub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/lanes/right", 1);

        if(debugMode)
        {
            cv::namedWindow(OPENCV_WINDOW_rgb);
            cv::namedWindow(OPENCV_WINDOW_topview);
            cv::namedWindow(OPENCV_WINDOW_topview_roi);
            cv::namedWindow(OPENCV_WINDOW_lines);
            cv::namedWindow(OPENCV_WINDOW_lines_morph);
            cv::namedWindow(OPENCV_WINDOW_lanes_hor);
            cv::namedWindow(OPENCV_WINDOW_lanes_vert);
        }
    }

    ~OttoCarVisLoc()
    {
        if(debugMode)
        {
            cv::destroyWindow(OPENCV_WINDOW_rgb);
            cv::destroyWindow(OPENCV_WINDOW_topview);
            cv::destroyWindow(OPENCV_WINDOW_topview_roi);
            cv::destroyWindow(OPENCV_WINDOW_lines);
            cv::destroyWindow(OPENCV_WINDOW_lines_morph);
            cv::destroyWindow(OPENCV_WINDOW_lanes_hor);
            cv::destroyWindow(OPENCV_WINDOW_lanes_vert);
        }
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

        inputTimestamp = ros::Time::now();


        //process on input
        processImage(rgb_img);


        // Output modified video streams
        cv_ptr->image = lanes_img_hor.clone();
        // Output modified video stream
        image_pub_rgb.publish(cv_ptr->toImageMsg());//cv::FileStorage file("some_name.ext", cv::FileStorage::WRITE);


        
        cv_ptr->image = linesMorph_img.clone();
        cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
        image_pub_bw.publish(cv_ptr->toImageMsg());//cv::FileStorage file("some_name.ext", cv::FileStorage::WRITE);
    }

    void processImage(cv::Mat& inputImg)
    {
        //init images
        if(topview_img.empty())
        {
            topview_img = inputImg.clone();
            lines_img = inputImg.clone();
            linesMorph_img = inputImg.clone();
            lanes_img_hor = inputImg.clone();
            lanes_img_vert = inputImg.clone();
        }

        clock_t startProcess= clock();

        // process Topview projection
        clock_t startBirdsEye = clock();
        birdsEyeTransform(inputImg, topview_img, H, topviewSize);
        postprocessTopview(topview_img, resizeTopviewOutput, carPos, visCarPos);
        clock_t endBirdsEye = clock();
        double TimeTopview = diffclock(startBirdsEye, endBirdsEye);

        //set region of process
        if(useProcessingRoi)
        {
            topviewRoi_img = cv::Mat(roiSize, CV_8UC3);
            topview_img(processRoi).copyTo(topviewRoi_img);
            cv::rectangle(topview_img, processRoi, CV_RGB(255,255,0), 2);
        }
        else
        {
            topviewRoi_img = topview_img.clone();
        }

        // Lane Mark Detection
        // init kernel
        if(meanQuadraticNeighborKernel.empty())
        {            
            createQuadraticMeanNeighborKernel(meanQuadraticNeighborKernel, NEIGHBOR_KERNEL_SIZE);
        }
        if(meanHorizontalNeighborKernel.empty())
        {
            createHorizontalMeanNeighborKernel(meanHorizontalNeighborKernel, 5);
            cv::transpose(meanHorizontalNeighborKernel, meanHorizontalNeighborKernel);
        }
        clock_t startLaneMarks = clock();        
        detectLaneMarksV2(topviewRoi_img, /*meanHorizontalNeighborKernel*/ meanQuadraticNeighborKernel, lines_img, linesMorph_img, lanes_img_vert);
        // viz max_append_dist of line chain creation
        cv::line(topview_img, cv::Point(0,50), cv::Point((int)LINECHAIN_MAX_APPEND_DIST-1,50), CV_RGB(0,255,255));

        clock_t endLaneMarks = clock();
        double TimeLaneMarks = diffclock(startLaneMarks, endLaneMarks);

        clock_t endProcess = clock();
        double TimeProcess = diffclock(startProcess, endProcess);

        // visualize kernel size        
        cv::rectangle(topview_img, cv::Point(0,0), cv::Point(NEIGHBOR_KERNEL_SIZE, NEIGHBOR_KERNEL_SIZE), CV_RGB(255,0,0), 2);

        // ------
        // OUTPUT
        // ------        
        //ROS_INFO_STREAM("pos angle: " << calcAngleBetweenVecs(cv::Point(1,0),cv::Point(0,1)));
        //ROS_INFO_STREAM("neg angle: " << calcAngleBetweenVecs(cv::Point(0,1),cv::Point(1,0)));
//        ROS_INFO_STREAM("===========================================");
//       ROS_INFO_STREAM("Time Topview: " << TimeTopview << "millisec" );
//        ROS_INFO_STREAM("Time LaneMarks: " << TimeLaneMarks << "millisec" );
//        ROS_INFO_STREAM("Time Process: " << TimeProcess << "millisec" );

        //set array data and publish
        perfArray.data.clear();
        perfArray.data.push_back(TimeTopview);
        perfArray.data.push_back(TimeLaneMarks);
        perfArray.data.push_back(TimeProcess);
        perfPub.publish(perfArray);

        //publish lane State
        laneStatePub.publish(laneStateArray);

        //publish lane data
        laneLeftPub.publish(laneLeftArray);
        laneRightPub.publish(laneRightArray);
        laneMidPub.publish(laneMidArray);


        // show images
        if(debugMode)
        {
            //cv::imshow(OPENCV_WINDOW_rgb, inputImg);
            cv::imshow(OPENCV_WINDOW_topview, topview_img);
            cv::imshow(OPENCV_WINDOW_topview_roi, topviewRoi_img);
            cv::imshow(OPENCV_WINDOW_lines, lines_img);
            cv::imshow(OPENCV_WINDOW_lines_morph, linesMorph_img);
            cv::imshow(OPENCV_WINDOW_lanes_hor, lanes_img_hor);
            cv::imshow(OPENCV_WINDOW_lanes_vert, lanes_img_vert);
            cv::waitKey(1);
        }

        //  Save images
//        char k;
//        k = cv::waitKey(3);  // wait 3 sec before next image
//        //printf("%d\n", k);
//        if(k == 10) // save if "Enter" is clicked
//        {
//            // save images + H + CalibConfig
//            //cv::imwrite("presentation_topview.jpg", topview_img);
//            cv::imwrite("presentation_centerLines_hor.jpg", lines_img);
//            cv::imwrite("presentation_centerLines_vert.jpg", linesMorph_img);
//            cv::imwrite("presentation_PolyLines_hor.jpg", lanes_img_hor);
//            cv::imwrite("presentation_PolyLines_vert.jpg", lanes_img_vert);
//        }

    }


    void detectLaneMarksV2(cv::Mat &inputTopview, cv::Mat &neighborKernel, cv::Mat &dstTopviewThreshold, cv::Mat &dstTopviewThresholdMorph, cv::Mat & dstLanes)
    {        
        lineChainList.clear();
        polylineList.clear();
        polylineListDashed.clear();
//        polylineListFinal.clear();
        polyLineTypeList.clear();
        polyLinesParamList.clear();
        PolyLineStateStructList.clear();

        cv::Mat topviewGrayImg, topviewGrayImgSmall, meanNeighborImgSmall, topviewThresholdImg, topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, polyLineImg, houghImg, lineChainInputImg;
        cv::cvtColor(inputTopview, topviewGrayImg, CV_RGB2GRAY);        

        polyLineImg = cv::Mat::zeros(inputTopview.rows, inputTopview.cols, inputTopview.type());

        // threshold gray image
        double threshold = LINE_DETECTION_THRESHOLD;//190.0;
        //float maxColorDiffThreshold = 100.0;
        //cv::threshold(topviewGrayImg, topviewThresholdImg, threshold, 255.0, cv::THRESH_BINARY);
        float resizeFactor = 1.;
        //cv::resize(topviewGrayImg, topviewGrayImgSmall, cv::Size(), 1/resizeFactor, 1/resizeFactor);
        topviewGrayImgSmall = topviewGrayImg.clone();
        //cv::line(topviewGrayImgSmall, cv::Point(120,100), cv::Point(175,180), CV_RGB(255,255,255), 3, CV_AA);
        cv::filter2D(topviewGrayImgSmall, meanNeighborImgSmall, -1, neighborKernel);

        topviewThresholdImgSmall = cv::Mat::zeros(topviewGrayImgSmall.rows, topviewGrayImgSmall.cols, topviewGrayImgSmall.type());

        //Local thresholding
        //float colorDiff_0, colorDiff_1, maxColorDiff;
        for (int x = 0; x < topviewGrayImgSmall.cols; x++)
        {
            for (int y = 0; y < topviewGrayImgSmall.rows; y++)
            {
                uchar& pixVal = topviewThresholdImgSmall.at<uchar>(y, x);
                uchar pixVal_input = topviewGrayImgSmall.at<uchar>(y, x);
                uchar meanNeighborVal = meanNeighborImgSmall.at<uchar>(y, x);

                // get max difference of color channels
//                cv::Vec3b pixValColor_input = inputTopview.at<cv::Vec3b>(y, x);
//                maxColorDiff = std::abs((float)((float)pixValColor_input.val[0] - (float)pixValColor_input.val[1]));
//                colorDiff_0 = std::abs((float)((float)pixValColor_input.val[0] - (float)pixValColor_input.val[2]));
//                if(colorDiff_0 > maxColorDiff) maxColorDiff = colorDiff_0;
//                colorDiff_1 = std::abs((float)((float)pixValColor_input.val[1] - (float)pixValColor_input.val[2]));
//                if(colorDiff_1 > maxColorDiff) maxColorDiff = colorDiff_1;

                //TODO: control color value of input image --> check if it is an gray value --> check channel difference

                // if pixel is outside of topview
                if(TOPVIEW_BACKGROUND_GRAY != (int)pixVal_input)
                {
                    //ROS_INFO_STREAM("colordiff:" << maxColorDiff);
                    if((int)pixVal_input > threshold + (int)meanNeighborVal/* &&
                            maxColorDiff < maxColorDiffThreshold*/)
                    {
                        //ROS_INFO_STREAM("I:" << (int)pixVal_hor << " M:" << (int)meanNeighborVal_hor);
                        pixVal = 255;
                    }
                }
            }
        }        

        // BINARY IMAGE POSTPROCESSING
        //-------------------------

        // Morphological open --> detecting big white areas
        cv::Mat bigWhiteObjectsImg;
        cv::Mat bigOpingKernel = cv::getStructuringElement(cv::MORPH_CROSS , cv::Size(3, 3));
        cv::erode(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, bigOpingKernel, cv::Point(-1,-1), 3);
        cv::dilate(tmpTopviewThresholdImgSmall, bigWhiteObjectsImg, bigOpingKernel, cv::Point(-1,-1), 3);
        if(debugMode)   cv::imshow("big white objects", bigWhiteObjectsImg);

        // subtract big white objects from binary image
        cv::Mat diffBigObjectsImg = topviewThresholdImgSmall - bigWhiteObjectsImg;
        if(debugMode)   cv::imshow("DIFF big white objects", diffBigObjectsImg);
        topviewThresholdImgSmall = diffBigObjectsImg.clone();


        // Morphological open --> reduce noise
        cv::Mat tmpThresh = topviewThresholdImgSmall.clone();
        cv::Mat opingKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(3, 3));
        cv::erode(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, opingKernel);
        cv::dilate(tmpTopviewThresholdImgSmall, topviewThresholdImgSmall, opingKernel);


        // use vertical kernel to detect horizontal lines


//        // moprhological close in vertical direction
//        cv::Mat tmpThreshClosed = topviewThresholdImgSmall.clone();
//        cv::Mat closingKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(3, 15));
//        cv::dilate(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, closingKernel);
//        cv::erode(tmpTopviewThresholdImgSmall, topviewThresholdImgSmall, closingKernel);



        // thinning
//        cv::Mat tmpThreshThinned = topviewThresholdImgSmall.clone();
//        MorphThinning(topviewThresholdImgSmall, topviewThresholdImgSmall);

        // Morphological open
//        cv::Mat tmpThreshVert = topviewThresholdImgSmall.clone();
//        cv::Mat opingKernelVert = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(1, 3));
//        cv::erode(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, opingKernelVert);
//        cv::dilate(tmpTopviewThresholdImgSmall, topviewThresholdImgSmall, opingKernelVert);

        // test hough transform
        //cv::cvtColor(topviewThresholdImgSmall, houghImg, CV_GRAY2BGR);
        //houghImg = inputTopview.clone();
        //doHoughTransform(topviewThresholdImgSmall, houghImg);
        //lanes_img_hor = houghImg;

        //make line chains of a smaller version of the binary image                
        //cv::resize(topviewThresholdImg, lineChainInputImg,cv::Size(), 1/resizeFactor, 1/resizeFactor);
        makeLineChains(topviewThresholdImgSmall/*lineChainInputImg*/, lineChainList, resizeFactor);

        lanes_img_hor = inputTopview.clone();
        dstLanes = inputTopview.clone();

        // approximate those line chains into Polylines and resize
        //approxCenterLines(lineChainList, polylineList, 5.0);
        classifyAndPostprocessPolylineList(lineChainList, 5.0, resizeFactor, inputTopview);
        //resizePolylinePointPositions(polylineList, resizeFactor);

        combineDashedLines();
        //dstLanes = inputTopview.clone();
        if(vizMode)   drawPolyLinesFinal(dstLanes);

        if(LaneStateStruct_current.empty() && LaneStateStruct_last.empty()) initCurrLaneState();

        //project polylines to base line
        projectPolylinesToBaseline();
        //lane detection
        laneDetection();
        //set candidate Struct
        setLaneStateCandidates();
        //update lane state Struct
        updateLaneState();

        if(vizMode)   vizCurrLanesState(lanes_img_hor);
        if(vizMode)   vizLaneStateList(lanes_img_hor, PolyLineStateStructList);

        //calc return values for module interface
        setLaneStateArray();

        setLanesArrays();

        if(vizMode)   vizCurrMidLaneState(lanes_img_hor);

        // resize for output visualisation
        cv::resize(topviewThresholdImgSmall, topviewThresholdImg, cv::Size(), resizeFactor, resizeFactor);



        dstTopviewThreshold = tmpThresh/*Closed*/ /*tmpThreshVert*/;
        dstTopviewThresholdMorph = topviewThresholdImg;//polyLineImg;
        //dstLanes
    }

    void setLanesArrays()
    {
        laneLeftArray.data.clear();
        laneMidArray.data.clear();
        laneRightArray.data.clear();

        for(int k=0; k<LaneStateStructCandidate[0].points.size(); k++)
        {
            laneLeftArray.data.push_back(LaneStateStructCandidate[0].points[k].x);
            laneLeftArray.data.push_back(LaneStateStructCandidate[0].points[k].y);
        }
        for(int k=0; k<LaneStateStructCandidate[1].points.size(); k++)
        {
            laneMidArray.data.push_back(LaneStateStructCandidate[1].points[k].x);
            laneMidArray.data.push_back(LaneStateStructCandidate[1].points[k].y);
        }
        for(int k=0; k<LaneStateStructCandidate[2].points.size(); k++)
        {
            laneRightArray.data.push_back(LaneStateStructCandidate[2].points[k].x);
            laneRightArray.data.push_back(LaneStateStructCandidate[2].points[k].y);
        }
    }

    void setLaneStateArray()
    {
        float confSum = LaneStateStruct_current[0].confidence + LaneStateStruct_current[1].confidence + LaneStateStruct_current[2].confidence;
        float w_left = LaneStateStruct_current[0].confidence / confSum;
        float w_mid = LaneStateStruct_current[1].confidence / confSum;
        float w_right = LaneStateStruct_current[2].confidence / confSum;

        //set x position of averaged mid lane
        double posX = (double)(w_left*(LaneStateStruct_current[0].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH) +
                                w_mid*LaneStateStruct_current[1].projectedToBaseLine.x +
                                w_right*(LaneStateStruct_current[2].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH));
        //set vec
        cv::Point2f laneDir;
        laneDir.x = (float)(w_left*LaneStateStruct_current[0].direction.x + w_mid*LaneStateStruct_current[1].direction.x + w_right*LaneStateStruct_current[2].direction.x);
        laneDir.y = (float)(w_left*LaneStateStruct_current[0].direction.y + w_mid*LaneStateStruct_current[1].direction.y + w_right*LaneStateStruct_current[2].direction.y);
        //set confidence
        double confidence = (double)(w_left*LaneStateStruct_current[0].confidence + w_mid*LaneStateStruct_current[1].confidence + w_right*LaneStateStruct_current[2].confidence);

        // calc deltas
        double deltaPosX_CarPosX = (double)(carPos.x - posX);
        // conversion in centimeter
        deltaPosX_CarPosX = deltaPosX_CarPosX * PIXEL_TO_CENTIMETER;

        cv::Point2f CarDir(0.0, 1.0);
        double deltaLaneAngle = (double)calcAngleBetweenVecs_float(CarDir, laneDir);        
        //conversion in radiant
        // deltaLaneAngle = (CV_PI/ 180.0) * deltaLaneAngle;


        laneStateArray.data.clear();
        laneStateArray.data.push_back(inputTimestamp.toSec());
        laneStateArray.data.push_back(deltaPosX_CarPosX);
        laneStateArray.data.push_back(deltaLaneAngle);
        laneStateArray.data.push_back(confidence);

        //position
        CurrMidLaneState.projectedToBaseLine = cv::Point((int)posX, 0);
        CurrMidLaneState.direction = laneDir;
        CurrMidLaneState.confidence = (float)confidence;
        CurrMidLaneState.linetype = LANE_BORDER_DASHED;

    }

    void MorphThinning(cv::Mat &img, cv::Mat &dst)
    {
        cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat temp;
        cv::Mat eroded;

        cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

        bool done;
        do
        {
          cv::erode(img, eroded, element);
          cv::dilate(eroded, temp, element); // temp = open(img)
          cv::subtract(img, temp, temp);
          cv::bitwise_or(skel, temp, skel);
          eroded.copyTo(img);

          done = (cv::countNonZero(img) == 0);
        } while (!done);

        cv::cvtColor(skel, dst, CV_GRAY2RGB);
        dst = skel;
    }


    void vizCurrLanesState(cv::Mat &dstImg)
    {
        int vizLine = 230;

        cv::Scalar color;
        for(int i=0; i<LaneStateStruct_current.size(); i++)
        {

            if(i == 0)       color = CV_RGB(255,0,0);
            else if(i == 1)  color = CV_RGB(0,255,0);
            else if(i == 2)  color = CV_RGB(0,0,255);
            cv::line( dstImg, cv::Point(LaneStateStruct_current[i].projectedToBaseLine.x, vizLine),
                    cv::Point(LaneStateStruct_current[i].projectedToBaseLine.x - (int)(40*LaneStateStruct_current[i].direction.x),
                              vizLine - (int)(40*LaneStateStruct_current[i].direction.y)), color, 1, CV_AA);
            vizNumber(dstImg, (float)LaneStateStruct_current[i].confidence, cv::Point(LaneStateStruct_current[i].projectedToBaseLine.x, vizLine+10));

            // viz construction range for current lane state
            cv::line( dstImg, cv::Point(LaneStateStruct_last[i].projectedToBaseLine.x - LANE_STATE_CANDIDATE_POS_DELTA, vizLine+5),
                    cv::Point(LaneStateStruct_last[i].projectedToBaseLine.x - LANE_STATE_CANDIDATE_POS_DELTA,
                              vizLine-5),color, 1, CV_AA);

            cv::line( dstImg, cv::Point(LaneStateStruct_last[i].projectedToBaseLine.x + LANE_STATE_CANDIDATE_POS_DELTA, vizLine+5),
                    cv::Point(LaneStateStruct_last[i].projectedToBaseLine.x + LANE_STATE_CANDIDATE_POS_DELTA,
                              vizLine-5),color, 1, CV_AA);

        }
    }

    void vizCurrMidLaneState(cv::Mat &dstImg)
    {
        int vizLine = 170;

        cv::Scalar color = CV_RGB(255,255,0);


        // viz pos, dir and conf
        cv::line( dstImg, cv::Point(CurrMidLaneState.projectedToBaseLine.x, vizLine),
                cv::Point(CurrMidLaneState.projectedToBaseLine.x - (int)(40*CurrMidLaneState.direction.x),
                          vizLine - (int)(40*CurrMidLaneState.direction.y)), color, 1, CV_AA);
        vizNumber(dstImg, (float)CurrMidLaneState.confidence, cv::Point(CurrMidLaneState.projectedToBaseLine.x, vizLine+10));

        // viz delta pos to carpos
        cv::line( dstImg, cv::Point(CurrMidLaneState.projectedToBaseLine.x, vizLine),
                  cv::Point(carPos.x, vizLine),color, 1, CV_AA);

        cv::line( dstImg, cv::Point(carPos.x, vizLine-5),
                  cv::Point(carPos.x, vizLine+5),color, 1, CV_AA);
        vizNumber(dstImg, (float)(carPos.x - CurrMidLaneState.projectedToBaseLine.x), cv::Point(carPos.x, vizLine-10));

        // viz angle diff
        cv::Point2f CarDir(0.0, 1.0);
        float deltaLaneAngle = calcAngleBetweenVecs_float(CarDir, CurrMidLaneState.direction);
        vizNumber(dstImg, deltaLaneAngle, cv::Point(CurrMidLaneState.projectedToBaseLine.x, vizLine - 30));

    }


    void vizLaneStateList(cv::Mat &dstImg, ListOfStateStructs &inputList )
    {
        cv::Scalar color;
        for(int i=0; i<inputList.size(); i++)
        {
            if(inputList[i].linetype == LANE_BORDER_CONTINOUS)
            {
                color = CV_RGB(255,0,255);

                if(inputList[i].confidence > 2.0)     vizNumber_color(dstImg, (float)inputList[i].confidence, inputList[i].projectedToBaseLine, CV_RGB(0,200,0));
                else if(inputList[i].confidence > 1.0)     vizNumber_color(dstImg, (float)inputList[i].confidence, inputList[i].projectedToBaseLine, CV_RGB(0,0,200));
                else                                    vizNumber(dstImg, (float)inputList[i].confidence, inputList[i].projectedToBaseLine);
            }
            if(inputList[i].linetype == LANE_BORDER_DASHED)
            {
                color = CV_RGB(0,255,255);

                if(inputList[i].confidence > 2.0)     vizNumber_color(dstImg, (float)inputList[i].confidence, cv::Point(inputList[i].projectedToBaseLine.x, inputList[i].projectedToBaseLine.y - 10), CV_RGB(0,200,0));
                else if(inputList[i].confidence > 1.0)     vizNumber_color(dstImg, (float)inputList[i].confidence, cv::Point(inputList[i].projectedToBaseLine.x, inputList[i].projectedToBaseLine.y - 10), CV_RGB(0,0,200));
                else                                    vizNumber(dstImg, (float)inputList[i].confidence, cv::Point(inputList[i].projectedToBaseLine.x, inputList[i].projectedToBaseLine.y - 10));
            }
            cv::line( dstImg, inputList[i].projectedToBaseLine,
                    cv::Point(inputList[i].projectedToBaseLine.x - (int)(40*inputList[i].direction.x),
                              inputList[i].projectedToBaseLine.y - (int)(40*inputList[i].direction.y)),
                    color, 1, CV_AA);

        }
    }


    void laneDetection()
    {
        for(int i=0; i<PolyLineStateStructList.size(); i++)
        {
            //for each dashed line
            if(PolyLineStateStructList[i].linetype == LANE_BORDER_DASHED)
            {
                for(int k=0; k<PolyLineStateStructList.size(); k++)
                {
                    if(k==i) continue;
                    if(PolyLineStateStructList[k].linetype == LANE_BORDER_DASHED) continue;
                    if(!checkVecsForSameDir(PolyLineStateStructList[k].direction, PolyLineStateStructList[i].direction)) continue;

                    //left corresponding line found
                    if(PolyLineStateStructList[k].projectedToBaseLine.x < PolyLineStateStructList[i].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                       PolyLineStateStructList[k].projectedToBaseLine.x > PolyLineStateStructList[i].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                    {
                        //increase conf
                        PolyLineStateStructList[k].confidence = PolyLineStateStructList[k].confidence + 0.5;
                        PolyLineStateStructList[i].confidence = PolyLineStateStructList[i].confidence + 0.5;
                    }
                    //right corresponding line found
                    if(PolyLineStateStructList[k].projectedToBaseLine.x < PolyLineStateStructList[i].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                       PolyLineStateStructList[k].projectedToBaseLine.x > PolyLineStateStructList[i].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                    {
                        //increase conf
                        PolyLineStateStructList[k].confidence = PolyLineStateStructList[k].confidence + 0.5;
                        PolyLineStateStructList[i].confidence = PolyLineStateStructList[i].confidence + 0.5;
                    }
                }
            }
            //for each continous line
            else if(PolyLineStateStructList[i].linetype == LANE_BORDER_CONTINOUS)
            {
                for(int k=0; k<PolyLineStateStructList.size(); k++)
                {
                    if(k==i) continue;
                    if(!checkVecsForSameDir(PolyLineStateStructList[k].direction, PolyLineStateStructList[i].direction)) continue;

                    // current is dashed
                    if(PolyLineStateStructList[k].linetype == LANE_BORDER_DASHED)
                    {
                        //left corresponding mid line found
                        if(PolyLineStateStructList[k].projectedToBaseLine.x < PolyLineStateStructList[i].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateStructList[k].projectedToBaseLine.x > PolyLineStateStructList[i].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateStructList[k].confidence = PolyLineStateStructList[k].confidence + 0.5;
                            PolyLineStateStructList[i].confidence = PolyLineStateStructList[i].confidence + 0.5;
                        }
                        //right corresponding mid line found
                        if(PolyLineStateStructList[k].projectedToBaseLine.x < PolyLineStateStructList[i].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateStructList[k].projectedToBaseLine.x > PolyLineStateStructList[i].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateStructList[k].confidence = PolyLineStateStructList[k].confidence + 0.5;
                            PolyLineStateStructList[i].confidence = PolyLineStateStructList[i].confidence + 0.5;
                        }
                    }
                    // current is continous
                    else if(PolyLineStateStructList[k].linetype == LANE_BORDER_CONTINOUS)
                    {
                        //left corresponding line found
                        if(PolyLineStateStructList[k].projectedToBaseLine.x < PolyLineStateStructList[i].projectedToBaseLine.x - 2*LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateStructList[k].projectedToBaseLine.x > PolyLineStateStructList[i].projectedToBaseLine.x - 2*LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateStructList[k].confidence = PolyLineStateStructList[k].confidence + 0.5;
                            PolyLineStateStructList[i].confidence = PolyLineStateStructList[i].confidence + 0.5;
                        }
                        //right corresponding line found
                        if(PolyLineStateStructList[k].projectedToBaseLine.x < PolyLineStateStructList[i].projectedToBaseLine.x + 2*LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateStructList[k].projectedToBaseLine.x > PolyLineStateStructList[i].projectedToBaseLine.x + 2*LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateStructList[k].confidence = PolyLineStateStructList[k].confidence + 0.5;
                            PolyLineStateStructList[i].confidence = PolyLineStateStructList[i].confidence + 0.5;
                        }
                    }
                }
            }
        }
    }

    bool checkVecsForSameDir(cv::Point2f &vecA, cv::Point2f &vecB)
    {
        float angle = calcAngleBetweenVecs(vecA, vecB);
        if(std::abs((double)angle) <= LANE_DETECTION_ANGLE_DELTA)
        {
            return true;
        }
        else return false;
    }


    void initCurrLaneState()
    {
        //init or reset lanestate to middle of rigth lane
        StateStruct leftLane;
        //set point
        leftLane.projectedToBaseLine.x = (int)(carPos.x - 1.5 *((float)LANE_DETECTION_LANE_WIDTH));
        leftLane.projectedToBaseLine.y = 290;
        //set vec
        leftLane.direction.x = (float)(0);
        leftLane.direction.y = (float)(1);
        //normalize vec
        leftLane.direction.x = leftLane.direction.x / std::sqrt(std::pow(leftLane.direction.x,2)
                                                                            + std::pow(leftLane.direction.y,2));
        leftLane.direction.y = leftLane.direction.y / std::sqrt(std::pow(leftLane.direction.x,2)
                                                                            + std::pow(leftLane.direction.y,2));
        //set confidence
        leftLane.confidence = 0.5;
        //set linetype
        leftLane.linetype = LANE_BORDER_CONTINOUS;

        LaneStateStruct_current.push_back(leftLane);
        LaneStateStruct_last.push_back(leftLane);


        StateStruct midLane;
        //set point
        midLane.projectedToBaseLine.x = (int)(carPos.x - 0.5 *((float)LANE_DETECTION_LANE_WIDTH));
        midLane.projectedToBaseLine.y = 290;
        //set vec
        midLane.direction.x = (float)(0);
        midLane.direction.y = (float)(1);
        //normalize vec
        midLane.direction.x = midLane.direction.x / std::sqrt(std::pow(midLane.direction.x,2)
                                                                            + std::pow(midLane.direction.y,2));
        midLane.direction.y = midLane.direction.y / std::sqrt(std::pow(midLane.direction.x,2)
                                                                            + std::pow(midLane.direction.y,2));
        //set confidence
        midLane.confidence = 0.5;
        //set linetype
        midLane.linetype = LANE_BORDER_DASHED;

        LaneStateStruct_current.push_back(midLane);
        LaneStateStruct_last.push_back(midLane);


        StateStruct rightLane;
        //set point
        rightLane.projectedToBaseLine.x = (int)(carPos.x + 0.5 *((float)LANE_DETECTION_LANE_WIDTH));
        rightLane.projectedToBaseLine.y = 290;
        //set vec
        rightLane.direction.x = (float)(0);
        rightLane.direction.y = (float)(1);
        //normalize vec
        rightLane.direction.x = rightLane.direction.x / std::sqrt(std::pow(rightLane.direction.x,2)
                                                                            + std::pow(rightLane.direction.y,2));
        rightLane.direction.y = rightLane.direction.y / std::sqrt(std::pow(rightLane.direction.x,2)
                                                                            + std::pow(rightLane.direction.y,2));
        //set confidence
        rightLane.confidence = 0.5;
        //set linetype
        rightLane.linetype = LANE_BORDER_CONTINOUS;

        LaneStateStruct_current.push_back(rightLane);
        LaneStateStruct_last.push_back(rightLane);
    }


    void projectPolylinesToBaseline()
    {
        // set polyLineStructList
        for(int i=0; i<polylineList.size(); i++)
        {
            StateStruct stateStruct;
            //todo: project point instead of just copying x value + use car pos for base line
            //set point
            stateStruct.projectedToBaseLine.x = polylineList[i][0].x;
            stateStruct.projectedToBaseLine.y = 290;
            //set vec
            //ERROR in HERE??? target - src?
            stateStruct.direction.x = (float)(polylineList[i][0].x - polylineList[i][1].x);
            stateStruct.direction.y = (float)(polylineList[i][0].y - polylineList[i][1].y);
            //normalize vec
            stateStruct.direction.x = stateStruct.direction.x / std::sqrt(std::pow(stateStruct.direction.x,2)
                                                                                + std::pow(stateStruct.direction.y,2));
            stateStruct.direction.y = stateStruct.direction.y / std::sqrt(std::pow(stateStruct.direction.x,2)
                                                                                + std::pow(stateStruct.direction.y,2));
            //set confidence
            stateStruct.confidence = polyLinesParamList[i].second;

            //set linetype
            stateStruct.linetype = polyLineTypeList[i];

            for(int k=0;k<polylineList[i].size();k++){
                stateStruct.points.push_back(polylineList[i][k]);
            }

            PolyLineStateStructList.push_back(stateStruct);
        }
    }


    void setLaneStateCandidates()
    {
        LaneStateStructCandidate.clear();
        // also possible without folowing lists
        BorderLeftCandidateList.clear();
        BorderMidCandidateList.clear();
        BorderRightCandidateList.clear();

        //todo: limit ranges to images ranges
        int leftRange[2] = {LaneStateStruct_last[0].projectedToBaseLine.x - LANE_STATE_CANDIDATE_POS_DELTA, LaneStateStruct_last[0].projectedToBaseLine.x + LANE_STATE_CANDIDATE_POS_DELTA};
        int midRange[2] = {LaneStateStruct_last[1].projectedToBaseLine.x - LANE_STATE_CANDIDATE_POS_DELTA, LaneStateStruct_last[1].projectedToBaseLine.x + LANE_STATE_CANDIDATE_POS_DELTA};
        int rightRange[2] = {LaneStateStruct_last[2].projectedToBaseLine.x - LANE_STATE_CANDIDATE_POS_DELTA, LaneStateStruct_last[2].projectedToBaseLine.x + LANE_STATE_CANDIDATE_POS_DELTA};

        float bestLeftConf[2] = {-1.0, -1.0}; // val; id in border candidate list
        float bestMidConf[2] = {-1.0, -1.0};
        float bestRightConf[2] = {-1.0, -1.0};
        int leftCounter = -1;
        int midCounter = -1;
        int rightCounter = -1;


        // SEARCH THE BEST FITTING LINE
        // ----------------------------
        for(int i=0; i<PolyLineStateStructList.size(); i++)
        {
            StateStruct currStateStruct = PolyLineStateStructList[i];
            // test on left angle difference
            //last timestep is still current till it will be updated later in updateLaneState()!!!
            if(std::abs(calcAngleBetweenVecs_float(currStateStruct.direction, LaneStateStruct_current[0].direction)) < LANE_STATE_CANDIDATE_ANGLE_DELTA)
            {
                //test on left range
                if(currStateStruct.projectedToBaseLine.x > leftRange[0] && currStateStruct.projectedToBaseLine.x < leftRange[1] &&
                        currStateStruct.linetype == LANE_BORDER_CONTINOUS)
                {
                    BorderLeftCandidateList.push_back(currStateStruct);
                    leftCounter++;
                    if(bestLeftConf[0] < currStateStruct.confidence)
                    {
                        bestLeftConf[0] = currStateStruct.confidence;
                        bestLeftConf[1] = (float)leftCounter;
                    }
                }
            }

            // test on mid angle difference
            if(std::abs(calcAngleBetweenVecs_float(currStateStruct.direction, LaneStateStruct_current[1].direction)) < LANE_STATE_CANDIDATE_ANGLE_DELTA)
            {
                // test on mid range
                if(currStateStruct.projectedToBaseLine.x > midRange[0] && currStateStruct.projectedToBaseLine.x < midRange[1] &&
                        currStateStruct.linetype == LANE_BORDER_DASHED)
                {
                    BorderMidCandidateList.push_back(currStateStruct);
                    midCounter++;
                    if(bestMidConf[0] < currStateStruct.confidence)
                    {
                        bestMidConf[0] = currStateStruct.confidence;
                        bestMidConf[1] = (float)midCounter;
                    }
                }
            }
            // test on mid angle difference
            if(std::abs(calcAngleBetweenVecs_float(currStateStruct.direction, LaneStateStruct_current[2].direction)) < LANE_STATE_CANDIDATE_ANGLE_DELTA)
            {
                // test on right range
                if(currStateStruct.projectedToBaseLine.x > rightRange[0] && currStateStruct.projectedToBaseLine.x < rightRange[1] &&
                        currStateStruct.linetype == LANE_BORDER_CONTINOUS)
                {
                    BorderRightCandidateList.push_back(currStateStruct);
                    rightCounter++;
                    if(bestRightConf[0] < currStateStruct.confidence)
                    {
                        bestRightConf[0] = currStateStruct.confidence;
                        bestRightConf[1] = (float)rightCounter;
                    }
                }
            }

        }


        // SET LANE STATE CANDIDATES
        // -------------------------
        StateStruct dummy;
        int lineFound[3] = {-1,-1,-1};
        int foundSum = 0;
        if((int)bestLeftConf[1] == -1)
        {
            lineFound[0] = 0;
            LaneStateStructCandidate.push_back(dummy);
        }
        else
        {
            lineFound[0] = 1;
            foundSum++;
            LaneStateStructCandidate.push_back(BorderLeftCandidateList[(int)bestLeftConf[1]]);
        }

        if((int)bestMidConf[1] == -1)
        {
            lineFound[1] = 0;
            LaneStateStructCandidate.push_back(dummy);
        }
        else
        {
            lineFound[1] = 1;
            foundSum++;
            LaneStateStructCandidate.push_back(BorderMidCandidateList[(int)bestMidConf[1]]);
        }

        if((int)bestRightConf[1] == -1)
        {
            lineFound[2] = 0;
            LaneStateStructCandidate.push_back(dummy);
        }
        else
        {
            lineFound[2] = 1;
            foundSum++;
            LaneStateStructCandidate.push_back(BorderRightCandidateList[(int)bestRightConf[1]]);
        }


        // SIMULATE NOT FOUND CANDIDATES
        // -----------------------------
        if(foundSum != 3 && foundSum > 0)
        {
            // simulate one line
            if(foundSum == 2)
            {
                if(lineFound[0] == 0)   //left
                {
                    //update point
                    LaneStateStructCandidate[0].projectedToBaseLine.x = LaneStateStructCandidate[1].projectedToBaseLine.x - (LaneStateStructCandidate[2].projectedToBaseLine.x - LaneStateStructCandidate[1].projectedToBaseLine.x);
                    LaneStateStructCandidate[0].projectedToBaseLine.y = LaneStateStructCandidate[1].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[0].direction.x = 0.5 * LaneStateStructCandidate[1].direction.x + 0.5 * LaneStateStructCandidate[2].direction.x;
                    LaneStateStructCandidate[0].direction.y = 0.5 * LaneStateStructCandidate[1].direction.y + 0.5 * LaneStateStructCandidate[2].direction.y;
                    //update confidence
                    LaneStateStructCandidate[0].confidence = 0.5 * LaneStateStructCandidate[1].confidence + 0.5 * LaneStateStructCandidate[2].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[0].linetype = LANE_BORDER_CONTINOUS;
                    // simulate points
                    LaneStateStructCandidate[0].points = simulatePoints(LaneStateStructCandidate[0]);
                }
                else if(lineFound[1] == 0)  //mid
                {
                    //update point
                    LaneStateStructCandidate[1].projectedToBaseLine.x = LaneStateStructCandidate[0].projectedToBaseLine.x + (int)(0.5 * (float)(LaneStateStructCandidate[2].projectedToBaseLine.x - LaneStateStructCandidate[0].projectedToBaseLine.x));
                    LaneStateStructCandidate[1].projectedToBaseLine.y = LaneStateStructCandidate[0].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[1].direction.x = 0.5 * LaneStateStructCandidate[0].direction.x + 0.5 * LaneStateStructCandidate[2].direction.x;
                    LaneStateStructCandidate[1].direction.y = 0.5 * LaneStateStructCandidate[0].direction.y + 0.5 * LaneStateStructCandidate[2].direction.y;
                    //update confidence
                    LaneStateStructCandidate[1].confidence = 0.5 * LaneStateStructCandidate[0].confidence + 0.5 * LaneStateStructCandidate[2].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[1].linetype = LANE_BORDER_DASHED;
                    // simulate points
                    LaneStateStructCandidate[1].points = simulatePoints(LaneStateStructCandidate[1]);
                }
                else if(lineFound[2] == 0)  //rigth
                {
                    //update point
                    LaneStateStructCandidate[2].projectedToBaseLine.x = LaneStateStructCandidate[1].projectedToBaseLine.x + (LaneStateStructCandidate[1].projectedToBaseLine.x - LaneStateStructCandidate[0].projectedToBaseLine.x);
                    LaneStateStructCandidate[2].projectedToBaseLine.y = LaneStateStructCandidate[0].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[2].direction.x = 0.5 * LaneStateStructCandidate[0].direction.x + 0.5 * LaneStateStructCandidate[1].direction.x;
                    LaneStateStructCandidate[2].direction.y = 0.5 * LaneStateStructCandidate[0].direction.y + 0.5 * LaneStateStructCandidate[1].direction.y;
                    //update confidence
                    LaneStateStructCandidate[2].confidence = 0.5 * LaneStateStructCandidate[0].confidence + 0.5 * LaneStateStructCandidate[1].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[2].linetype = LANE_BORDER_CONTINOUS;
                    // simulate points
                    LaneStateStructCandidate[2].points = simulatePoints(LaneStateStructCandidate[2]);
                }
            }
            // simulate two lines
            else if(foundSum == 1)
            {
                if(lineFound[0] == 1)   //left
                {
                    //update point
                    LaneStateStructCandidate[1].projectedToBaseLine.x = LaneStateStructCandidate[0].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH;
                    LaneStateStructCandidate[1].projectedToBaseLine.y = LaneStateStructCandidate[0].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[1].direction.x = LaneStateStructCandidate[0].direction.x;
                    LaneStateStructCandidate[1].direction.y = LaneStateStructCandidate[0].direction.y;
                    //update confidence
                    LaneStateStructCandidate[1].confidence = LaneStateStructCandidate[0].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[1].linetype = LANE_BORDER_DASHED;
                    // simulate points
                    LaneStateStructCandidate[1].points = simulatePoints(LaneStateStructCandidate[1]);

                    //update point
                    LaneStateStructCandidate[2].projectedToBaseLine.x = LaneStateStructCandidate[1].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH;
                    LaneStateStructCandidate[2].projectedToBaseLine.y = LaneStateStructCandidate[0].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[2].direction.x = LaneStateStructCandidate[0].direction.x;
                    LaneStateStructCandidate[2].direction.y = LaneStateStructCandidate[0].direction.y;
                    //update confidence
                    LaneStateStructCandidate[2].confidence = LaneStateStructCandidate[0].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[2].linetype = LANE_BORDER_CONTINOUS;
                    // simulate points
                    LaneStateStructCandidate[2].points = simulatePoints(LaneStateStructCandidate[2]);
                }
                else if(lineFound[1] == 1)  //mid
                {
                    //update point
                    LaneStateStructCandidate[0].projectedToBaseLine.x = LaneStateStructCandidate[1].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH;
                    LaneStateStructCandidate[0].projectedToBaseLine.y = LaneStateStructCandidate[1].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[0].direction.x = LaneStateStructCandidate[1].direction.x;
                    LaneStateStructCandidate[0].direction.y = LaneStateStructCandidate[1].direction.y;
                    //update confidence
                    LaneStateStructCandidate[0].confidence = LaneStateStructCandidate[1].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[0].linetype = LANE_BORDER_CONTINOUS;
                    // simulate points
                    LaneStateStructCandidate[0].points = simulatePoints(LaneStateStructCandidate[0]);

                    //update point
                    LaneStateStructCandidate[2].projectedToBaseLine.x = LaneStateStructCandidate[1].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH;
                    LaneStateStructCandidate[2].projectedToBaseLine.y = LaneStateStructCandidate[1].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[2].direction.x = LaneStateStructCandidate[1].direction.x;
                    LaneStateStructCandidate[2].direction.y = LaneStateStructCandidate[1].direction.y;
                    //update confidence
                    LaneStateStructCandidate[2].confidence = LaneStateStructCandidate[1].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[2].linetype = LANE_BORDER_CONTINOUS;
                    // simulate points
                    LaneStateStructCandidate[2].points = simulatePoints(LaneStateStructCandidate[2]);
                }
                else if(lineFound[2] == 1)  //rigth
                {
                    //update point
                    LaneStateStructCandidate[1].projectedToBaseLine.x = LaneStateStructCandidate[2].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH;
                    LaneStateStructCandidate[1].projectedToBaseLine.y = LaneStateStructCandidate[2].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[1].direction.x = LaneStateStructCandidate[2].direction.x;
                    LaneStateStructCandidate[1].direction.y = LaneStateStructCandidate[2].direction.y;
                    //update confidence
                    LaneStateStructCandidate[1].confidence = LaneStateStructCandidate[2].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[1].linetype = LANE_BORDER_DASHED;
                    // simulate points
                    LaneStateStructCandidate[1].points = simulatePoints(LaneStateStructCandidate[1]);

                    //update point
                    LaneStateStructCandidate[0].projectedToBaseLine.x = LaneStateStructCandidate[1].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH;
                    LaneStateStructCandidate[0].projectedToBaseLine.y = LaneStateStructCandidate[2].projectedToBaseLine.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateStructCandidate[0].direction.x = LaneStateStructCandidate[2].direction.x;
                    LaneStateStructCandidate[0].direction.y = LaneStateStructCandidate[2].direction.y;
                    //update confidence
                    LaneStateStructCandidate[0].confidence = LaneStateStructCandidate[2].confidence;
                    //lane type keeps the same
                    LaneStateStructCandidate[0].linetype = LANE_BORDER_CONTINOUS;
                    // simulate points
                    LaneStateStructCandidate[0].points = simulatePoints(LaneStateStructCandidate[0]);
                }
            }
        }

        // copy last timestep if no line was found--> last timestep is still current till it will be updated later in updateLaneState()
        // todo: predict timestep instead just copying
        else if(foundSum != 3 && foundSum == 0)
        {
            for(int i=0; i < 3; i++)
            {
                //update point
                LaneStateStructCandidate[i].projectedToBaseLine.x = LaneStateStruct_current[i].projectedToBaseLine.x;
                LaneStateStructCandidate[i].projectedToBaseLine.y = LaneStateStruct_current[i].projectedToBaseLine.y;
                //update vec
                LaneStateStructCandidate[i].direction.x = LaneStateStruct_current[i].direction.x;
                LaneStateStructCandidate[i].direction.y = LaneStateStruct_current[i].direction.y;
                //update confidence
                LaneStateStructCandidate[i].confidence = LaneStateStruct_current[i].confidence;
                //lane type keeps the same
                LaneStateStructCandidate[i].linetype = LaneStateStruct_current[i].linetype;

                // simulate points
                LaneStateStructCandidate[i].points = simulatePoints(LaneStateStructCandidate[i]);
            }
        } 
    }

    std::vector<cv::Point> simulatePoints(StateStruct stateStruct) 
    {
        std::vector<cv::Point> points;

        cv::Point a(stateStruct.projectedToBaseLine.x,stateStruct.projectedToBaseLine.y);
        cv::Point b(stateStruct.projectedToBaseLine.x+stateStruct.direction.x*200,stateStruct.projectedToBaseLine.y+stateStruct.direction.y*200);
        points.push_back(a);
        points.push_back(b);

        return points;
    }


    void updateLaneState()
    {
        //ROS_INFO_STREAM("a");
        // set time step
        LaneStateStruct_last.clear();
        StateStruct laneBorder;
        laneBorder = LaneStateStruct_current[0];
        LaneStateStruct_last.push_back(laneBorder);
        laneBorder = LaneStateStruct_current[1];
        LaneStateStruct_last.push_back(laneBorder);
        laneBorder = LaneStateStruct_current[2];
        LaneStateStruct_last.push_back(laneBorder);
        LaneStateStruct_current.clear();

        
        //todo: if variance to last step is too big, let it out


//        float k = 0.5;
//        // a(t) = k * a(t) + (1-k) * a(t-1)
//        // go to lanes
//        for(int i=0; i<LaneStateStructCandidate.size(); i++)
//        {
//            StateStruct laneBorder;
//            //update point
//            laneBorder.projectedToBaseLine.x = (int)round((double) k * LaneStateStructCandidate[i].projectedToBaseLine.x + (1.0-k) * LaneStateStruct_last[i].projectedToBaseLine.x);
//            laneBorder.projectedToBaseLine.y = (int)round((double) k * LaneStateStructCandidate[i].projectedToBaseLine.y + (1.0-k) * LaneStateStruct_last[i].projectedToBaseLine.y);
//            //update vec
//            laneBorder.direction.x = k * LaneStateStructCandidate[i].direction.x + (1.0-k) * LaneStateStruct_last[i].direction.x;
//            laneBorder.direction.y = k * LaneStateStructCandidate[i].direction.y + (1.0-k) * LaneStateStruct_last[i].direction.y;
//            //update confidence
//            laneBorder.confidence = k * LaneStateStructCandidate[i].confidence + (1.0-k) * LaneStateStruct_last[i].confidence;
//            //lane type keeps the same

//            LaneStateStruct_current.push_back(laneBorder);
//        }

        float w_curr =  3./9.;
        float w_last =  4./9.;
        float w_other =  1./9.; // 2x

        // LEFT
        StateStruct laneBorderLeft;
        //update point 
        laneBorderLeft.projectedToBaseLine.x = (int)round((double) w_curr * LaneStateStructCandidate[0].projectedToBaseLine.x + w_last * LaneStateStruct_last[0].projectedToBaseLine.x +
                                                        w_other * (LaneStateStructCandidate[1].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH) +
                                                        w_other * (LaneStateStructCandidate[2].projectedToBaseLine.x - (2*LANE_DETECTION_LANE_WIDTH)));
        laneBorderLeft.projectedToBaseLine.y = 0.0;
        //update vec
        laneBorderLeft.direction.x = w_curr * LaneStateStructCandidate[0].direction.x + w_last * LaneStateStruct_last[0].direction.x +
                                    w_other * LaneStateStructCandidate[1].direction.x + w_other * LaneStateStructCandidate[2].direction.x;
        laneBorderLeft.direction.y = w_curr * LaneStateStructCandidate[0].direction.y + w_last * LaneStateStruct_last[0].direction.y +
                                    w_other * LaneStateStructCandidate[1].direction.y + w_other * LaneStateStructCandidate[2].direction.y;
        //update confidence
        laneBorderLeft.confidence = w_curr * LaneStateStructCandidate[0].confidence + w_last * LaneStateStruct_last[0].confidence +
                                    w_other * LaneStateStructCandidate[1].confidence + w_other * LaneStateStruct_last[2].confidence;
        //lane type keeps the same

        LaneStateStruct_current.push_back(laneBorderLeft);

        // MID
        StateStruct laneBorderMid;
        //update point
        laneBorderMid.projectedToBaseLine.x = (int)round((double) w_curr * LaneStateStructCandidate[1].projectedToBaseLine.x + w_last * LaneStateStruct_last[1].projectedToBaseLine.x +
                                                        w_other * (LaneStateStructCandidate[0].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH) +
                                                        w_other * (LaneStateStructCandidate[2].projectedToBaseLine.x - LANE_DETECTION_LANE_WIDTH));
        laneBorderMid.projectedToBaseLine.y = 0.0;
        //update vec
        laneBorderMid.direction.x = w_curr * LaneStateStructCandidate[1].direction.x + w_last * LaneStateStruct_last[1].direction.x +
                                        w_other * LaneStateStructCandidate[0].direction.x + w_other * LaneStateStructCandidate[2].direction.x;
        laneBorderMid.direction.y = w_curr * LaneStateStructCandidate[1].direction.y + w_last * LaneStateStruct_last[1].direction.y +
                                        w_other * LaneStateStructCandidate[0].direction.y + w_other * LaneStateStructCandidate[2].direction.y;
        //update confidence
        laneBorderMid.confidence = w_curr * LaneStateStructCandidate[1].confidence + w_last * LaneStateStruct_last[1].confidence +
                                        w_other * LaneStateStructCandidate[0].confidence + w_other * LaneStateStruct_last[2].confidence;
        //lane type keeps the same

        LaneStateStruct_current.push_back(laneBorderMid);

        // RIGHT
        StateStruct laneBorderRight;
        //update point
        laneBorderRight.projectedToBaseLine.x = (int)round((double) w_curr * LaneStateStructCandidate[2].projectedToBaseLine.x + w_last * LaneStateStruct_last[2].projectedToBaseLine.x +
                                                        w_other * (LaneStateStructCandidate[1].projectedToBaseLine.x + LANE_DETECTION_LANE_WIDTH) +
                                                        w_other * (LaneStateStructCandidate[0].projectedToBaseLine.x + (2*LANE_DETECTION_LANE_WIDTH)));
        laneBorderRight.projectedToBaseLine.y = 0.0;
        //update vec
        laneBorderRight.direction.x = w_curr * LaneStateStructCandidate[2].direction.x + w_last * LaneStateStruct_last[2].direction.x +
                                    w_other * LaneStateStructCandidate[0].direction.x + w_other * LaneStateStructCandidate[1].direction.x;
        laneBorderRight.direction.y = w_curr * LaneStateStructCandidate[2].direction.y + w_last * LaneStateStruct_last[2].direction.y +
                                    w_other * LaneStateStructCandidate[0].direction.y + w_other * LaneStateStructCandidate[1].direction.y;
        //update confidence
        laneBorderRight.confidence = w_curr * LaneStateStructCandidate[2].confidence + w_last * LaneStateStruct_last[2].confidence +
                                    w_other * LaneStateStructCandidate[0].confidence + w_other * LaneStateStruct_last[1].confidence;
        //lane type keeps the same

        LaneStateStruct_current.push_back(laneBorderRight);
    }


    void combineDashedLines()
    {
        // Todo: semantic error in here! dashed lines are not found sometimes (look at belongingFound)
        std::vector<int> processed;
        processed.clear();
        for(int i=0; i<polylineListDashed.size(); i++)
        {
            processed.push_back(0);
        }

        for(int i=0; i<polylineListDashed.size(); i++)
        {
            if(processed[i] != 0)   continue;
            // for each polyline check all next lines in list
            //std::vector<cv::Point> polyLine = polylineListDashed[i];
            //check if polylines correspond
            if(i+1 < polylineListDashed.size())
            {
                std::vector<cv::Point> finalPolyLine = polylineListDashed[i];
                bool belongingFound = false;
                cv::Point vecA(polylineListDashed[i][polylineListDashed[i].size()-1].x - polylineListDashed[i][0].x, polylineListDashed[i][polylineListDashed[i].size()-1].y - polylineListDashed[i][0].y);
                for(int k=i+1; k<polylineListDashed.size(); k++)
                {
                    if(processed[k] != 0)   continue;
                    belongingFound = checkDashedLineBelonging(finalPolyLine, vecA, i, processed);
                }

                processed[i] = 1;

                if(belongingFound || !belongingFound)
                {
                    // calc and append line params
                    double lineLength = cv::arcLength(finalPolyLine, false);

                    // calc changement of angles between finalPolyLine segments
                    // --> gives confidence val
                    float angleChangeConf = 0.0;
                    int counter = 0;
                    if(finalPolyLine.size() > 3)
                    {
                        for(int k=0; k<finalPolyLine.size()-3; k++)
                        {
                            counter= counter + 1;
                            angleChangeConf = angleChangeConf + (std::abs(
                                                        calcAngleBetweenVecs(cv::Point(finalPolyLine[k+1].x - finalPolyLine[k].x, finalPolyLine[k+1].y - finalPolyLine[k].y),
                                                                                cv::Point(finalPolyLine[k+2].x - finalPolyLine[k+1].x, finalPolyLine[k+2].y - finalPolyLine[k+1].y))
                                                        - calcAngleBetweenVecs(cv::Point(finalPolyLine[k+2].x - finalPolyLine[k+1].x, finalPolyLine[k+2].y - finalPolyLine[k+1].y),
                                                                                cv::Point(finalPolyLine[k+3].x - finalPolyLine[k+2].x, finalPolyLine[k+3].y - finalPolyLine[k+2].y))
                                                            ) / 360.0);
                        }
                        angleChangeConf =  angleChangeConf / (float)counter;
                    }
                    angleChangeConf = 1.0 - angleChangeConf;

                    // adjust confidence when dashed line is combined
                    if(belongingFound)
                    {
                        angleChangeConf = angleChangeConf + 1.0;
                    }

                    polylineList.push_back(finalPolyLine);
                    polyLineTypeList.push_back(LANE_BORDER_DASHED);
                    std::pair<float, float> polyParam((float)lineLength, angleChangeConf);
                    polyLinesParamList.push_back(polyParam);
                }
            }
        }
    }


    bool checkDashedLineBelonging(std::vector<cv::Point> &finalPolyLine, cv::Point &vecA, int &vecA_ID, std::vector<int> &processed)
    {
        bool belongingFound = false;
        for(int k=vecA_ID+1; k<polylineListDashed.size(); k++)
        {
            if(processed[k] != 0)   continue;

            cv::Point vec2Next(polylineListDashed[k][0].x - polylineListDashed[vecA_ID][polylineListDashed[vecA_ID].size()-1].x, polylineListDashed[k][0].y - polylineListDashed[vecA_ID][polylineListDashed[vecA_ID].size()-1].y);
            // line b is in range
            float vecLength = std::sqrt(std::pow(vec2Next.x,2) + std::pow(vec2Next.y,2));
            if(vecLength > 40.0)    continue;


            // line b lies in orientation direction of line a
            float angle2Next = std::abs(calcAngleBetweenVecs(vecA, vec2Next));
            if(angle2Next > 15.0) continue;

            // orientation of line b must be in +-20 degree of line a orientation
            cv::Point vecB(polylineListDashed[k][polylineListDashed[k].size()-1].x - polylineListDashed[k][0].x, polylineListDashed[k][polylineListDashed[k].size()-1].y - polylineListDashed[k][0].y);
            float angleBetween = std::abs(calcAngleBetweenVecs(vecA, vecB));
            if(angleBetween > 20.0) continue;

            // add line b to final line
            for(int l=0; l<polylineListDashed[k].size(); l++)
            {
                finalPolyLine.push_back(polylineListDashed[k][l]);
            }
            processed[k] = 1;
            belongingFound = true;

            // go on with line b checking
            if(k+1 < polylineListDashed.size())
            {
            checkDashedLineBelonging(finalPolyLine, vecB, k, processed);
            }
        }

        return belongingFound;
    }


    void classifyAndPostprocessPolylineList(std::vector<std::vector<cv::Point> > &inputCenterLines, float epsilon, float &resizeFactor, cv::Mat &dstPolylineImg)
    {
        // go through lines
        for(int i=0; i<inputCenterLines.size(); i++)
        {
            std::vector<cv::Point> polyLine;
            cv::approxPolyDP(inputCenterLines[i], polyLine, epsilon, false);


            // RESIZING
            // --------
            if(resizeFactor != 1.0)
            {
                for(int k=0; k<polyLine.size(); k++)
                {
                    // resize line point position
                    polyLine[k].x = resizeFactor * polyLine[k].x;
                    polyLine[k].y = resizeFactor * polyLine[k].y;
                }
            }


            // CLASSIFY
            // --------
            LineType currentLineType;

            double lineLength = cv::arcLength(polyLine, false);

            // calc changement of angles between polyline segments
            // --> gives confidence val
            float angleChangeConf = 0.0;
            int counter = 0;
            if(polyLine.size() > 3)
            {
                for(int k=0; k<polyLine.size()-3; k++)
                {
                    counter= counter + 1;
                    angleChangeConf = angleChangeConf + (std::abs(
                                                calcAngleBetweenVecs(cv::Point(polyLine[k+1].x - polyLine[k].x, polyLine[k+1].y - polyLine[k].y),
                                                                        cv::Point(polyLine[k+2].x - polyLine[k+1].x, polyLine[k+2].y - polyLine[k+1].y))
                                                - calcAngleBetweenVecs(cv::Point(polyLine[k+2].x - polyLine[k+1].x, polyLine[k+2].y - polyLine[k+1].y),
                                                                        cv::Point(polyLine[k+3].x - polyLine[k+2].x, polyLine[k+3].y - polyLine[k+2].y))
                                                    ) / 360.0);
                }
                angleChangeConf =  angleChangeConf / (float)counter;
            }
            angleChangeConf = 1.0 - angleChangeConf;

            // set classes
            if(lineLength <= 30.0)
            {
                currentLineType = LANE_BORDER_DASHED;

            }
            else    currentLineType = LANE_BORDER_CONTINOUS;
            if(lineLength < 10.0)       currentLineType = FALSE_POSITIVE;
            if(angleChangeConf < 0.5)       currentLineType = FALSE_POSITIVE;


            if(currentLineType == LANE_BORDER_CONTINOUS)
            {
                polylineList.push_back(polyLine);
                polyLineTypeList.push_back(currentLineType);
                std::pair<float, float> polyParam((float)lineLength, angleChangeConf);
                polyLinesParamList.push_back(polyParam);
            }
            if(currentLineType == LANE_BORDER_DASHED)       polylineListDashed.push_back(polyLine);


            // VIZUALISATION
            // -------------
            if(vizMode)
            {
                vizNumber(dstPolylineImg, angleChangeConf, polyLine[0]);
                for(int k=0; k<polyLine.size()-1; k++)
                {
                    // draw current Polyline
                    if(currentLineType == LANE_BORDER_CONTINOUS)      cv::line(dstPolylineImg, polyLine[k], polyLine[k+1], CV_RGB(0,255,0));
                    else if(currentLineType == LANE_BORDER_DASHED)    cv::line(dstPolylineImg, polyLine[k], polyLine[k+1], CV_RGB(0,0,255));
                    else if(currentLineType == FALSE_POSITIVE)        cv::line(dstPolylineImg, polyLine[k], polyLine[k+1], CV_RGB(255,0,0));
                }
            }
        }

    }


    void makeLineChains(cv::Mat &inputBinaryImg, std::vector<std::vector<cv::Point> > &dstLineChains, float &resizeFactor)
    {
        //float maxAppendDist = 25.0; // in topview coords
        // go horizontal over the image from the bottom to the top
        for(int y=inputBinaryImg.rows-1; y>=0; y--)
        {
            for(int x=0; x<=inputBinaryImg.cols-1; x++)
            {
                uchar pixValBin = inputBinaryImg.at<uchar>(y, x);

                // process if pixel is white
                if(0!=(int)pixValBin)
                {
                    cv::Point currPoint(x, y);
                    appendPointToNearestLineChain(currPoint, dstLineChains, LINECHAIN_MAX_APPEND_DIST / resizeFactor);//200.0
                }
            }
        }
    }


    void doHoughTransform(cv::Mat &srcImg, cv::Mat &dstImg)
    {
        std::vector<cv::Vec4i> lines;
          cv::HoughLinesP(srcImg, lines, 1, CV_PI/180, 40, 10, 40 );
          for( size_t i = 0; i < lines.size(); i++ )
          {
            cv::Vec4i l = lines[i];
            cv::line( dstImg, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,255,0), 1, CV_AA);
          }
    }

    //not needed any more
    void resizePolylinePointPositions(std::vector<std::vector<cv::Point> > &inputLineList, float factor)
    {
        for(int i=0; i<inputLineList.size(); i++)
        {
            for(int k=0; k<inputLineList[i].size(); k++)
            {
                inputLineList[i][k].x = factor * inputLineList[i][k].x;
                inputLineList[i][k].y = factor * inputLineList[i][k].y;
            }
        }
    }

    // replaced with cv function!!!
//    float getPolylineLength(std::vector<cv::Point> &inputLine)
//    {
//        float lineLength = 0.0;
//        for(int i=0; i<inputLine.size()-1; i++)
//        {
//            lineLength = lineLength +
//                            std::sqrt(std::pow(inputLine[i+1].x - inputLine[i].x, 2)
//                                    + std::pow(inputLine[i+1].y - inputLine[i].y, 2));
//        }

//        return lineLength;
//    }

    void appendPointToNearestLineChain(cv::Point &inputPos, std::vector<std::vector<cv::Point> > &inputLineChains, float maxAppendDist)
    {
        int nearestIndex = -1;
        float nearestDist = -1;

        float currDist;
        //cv::Point currPoint;
        for(int i = 0; i < inputLineChains.size(); i++)
        {
            // distance to last line chain element
            currDist = std::sqrt(std::pow(inputLineChains[i][inputLineChains[i].size()-1].x - inputPos.x, 2)
                                + std::pow(inputLineChains[i][inputLineChains[i].size()-1].y - inputPos.y, 2));
            // first line chain
            if(i==0)
            {
                nearestDist = currDist;
                nearestIndex = i;
            }
            // other line chains
            else
            {
                if(currDist < nearestDist)
                {
                    nearestDist = currDist;
                    nearestIndex = i;
                }
            }
        }

        //make new chain if distance is to large or line chains is empty
        if(nearestIndex == -1 || nearestDist > maxAppendDist)
        {
            std::vector<cv::Point> NewLineChain;
            NewLineChain.push_back(inputPos);
            inputLineChains.push_back(NewLineChain);
        }
        // append to nearest line chain
        else
        {
            inputLineChains[nearestIndex].push_back(inputPos);
        }

    }


    void loadDoubleMatFromFile(cv::Mat& m, const char* filename, int rows, int cols)
    {
        double a;
        int rowCount = 1;
        int colCount = 0;
        std::ifstream myfile (filename);
        if (myfile.is_open())
        {
        while ( myfile >> a)
        {
            colCount++;
            if(colCount > cols)
            {
                colCount = 1;
                rowCount++;
            }
            if(rowCount > rows) break;

            m.at<double>(rowCount-1,colCount-1) = a;
        }
        myfile.close();
        }
        else std::cout << "Unable to open file";
    }


    void loadCalibConfigFromFile(cv::Size& topviewSize, cv::Point& carPos, const char* filename)
    {
        std::ifstream myfile (filename);
        if (myfile.is_open())
        {
        myfile >> topviewSize.width;
        myfile >> topviewSize.height;
        myfile >> carPos.x;
        myfile >> carPos.y;
        myfile >> PIXEL_TO_CENTIMETER;
        myfile.close();
        }
        else std::cout << "Unable to open file";
    }


    void birdsEyeTransform(cv::Mat& inputImg, cv::Mat& outputImg, cv::Mat& projMat, cv::Size& topviewSize)
    {
        // transformate image
        cv::warpPerspective(inputImg, outputImg, projMat, topviewSize, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS, cv::BORDER_CONSTANT, TOPVIEW_BACKGROUND ) ;
    }

    void postprocessTopview(cv::Mat& dstImg, bool& resizeOutput, cv::Point& carPos, bool& visCarPos)
    {
        if(visCarPos)
        {
            cv::circle(dstImg, carPos, 5, CV_RGB(0,255,0), -1);
            cv::line(dstImg,cv::Point(0,0),cv::Point(NEIGHBOR_KERNEL_SIZE,0),cv::Scalar(0,0,255,0),3);
        }
        if(resizeOutput)
        {
            cv::resize(dstImg, dstImg, cv::Size(), .5, .5 );
        }
    }


    void createQuadraticMeanNeighborKernel(cv::Mat& dstKernel, int kernelSize)
    {
        float kernElem = (float) 1/((kernelSize * kernelSize)-1);
        int Myanchor = (int)(kernelSize / 2);
        cv::Mat neighborKern;
        neighborKern.create(kernelSize,kernelSize,CV_32FC1);

        for(int i = 0; i<kernelSize; i++)
        {
            for(int k = 0; k<kernelSize; k++)
            {
                float& pixVal = neighborKern.at<float>(i,k);
                if(i == Myanchor && k == Myanchor)
                {
                    pixVal = 0.0;
                }
                else    pixVal = kernElem;
            }
        }

        dstKernel = neighborKern;
    }


    void createHorizontalMeanNeighborKernel(cv::Mat& dstKernel, int kernelSize)
        {
            float kernElem = (float) 1/(kernelSize-1);
            int Myanchor = (int)(kernelSize-1) / 2;
            cv::Mat neighborKern;
            neighborKern.create(1,kernelSize,CV_32FC1);

            for(int i = 0; i<kernelSize; i++)
            {
                float& pixVal = neighborKern.at<float>(0,i);
                if(i == Myanchor)
                {
                    pixVal = 0.0;
                }
                pixVal = kernElem;
            }

            dstKernel = neighborKern;
        }


    void drawPolyLines(std::vector<std::vector<cv::Point> > &inputPolyLines, cv::Mat &dstImg, cv::Scalar color )
    {
        for(int i=0; i<inputPolyLines.size(); i++)
        {
            for(int k=0; k<inputPolyLines[i].size()-1; k++)
            {
                cv::line(dstImg, inputPolyLines[i][k], inputPolyLines[i][k+1], color);
            }
        }
    }


    void drawPolyLinesFinal(cv::Mat &dstImg)
    {
        cv::Scalar color;
        for(int i=0; i<polylineList.size(); i++)
        {
            for(int k=0; k<polylineList[i].size()-1; k++)
            {
                if(polyLineTypeList[i] == LANE_BORDER_CONTINOUS)    color = CV_RGB(0,255,0);
                if(polyLineTypeList[i] == LANE_BORDER_DASHED)       color = CV_RGB(0,0,255);
                cv::line(dstImg, polylineList[i][k], polylineList[i][k+1], color);
                // viz conf
                vizNumber(dstImg, (float)polyLinesParamList[i].second, polylineList[i][0]);
            }
        }
    }


    //TODO: could be combine with the classification for loop???
    void approxCenterLines(std::vector<std::vector<cv::Point> > &inputCenterLines, std::vector<std::vector<cv::Point> > &dstPolyLines, float epsilon)
    {
        for(int i=0; i<inputCenterLines.size(); i++)
        {
            std::vector<cv::Point> polyLine;
            cv::approxPolyDP(inputCenterLines[i], polyLine, epsilon, false);

            dstPolyLines.push_back(polyLine);
        }
    }       


    float calcAngleBetweenVecs(cv::Point src_vec, cv::Point trgt_vec)
    {// left direction is neg angle
     // rigth direction is pos angle
        float angle = (std::atan2((float)src_vec.y, (float)src_vec.x) - std::atan2((float)trgt_vec.y, (float)trgt_vec.x)) * 180.0 / CV_PI;
        return (-1.0*angle);
    }

    float calcAngleBetweenVecs_float(cv::Point2f src_vec, cv::Point2f trgt_vec)
    {// left direction is neg angle
     // rigth direction is pos angle
        //ERROR in HERE??? vec = target-src
        float angle = (std::atan2(src_vec.y, src_vec.x) - std::atan2(trgt_vec.y, trgt_vec.x)) * 180.0 / CV_PI;
        return (-1.0*angle);
    }

    void vizNumber(cv::Mat &dstImg, float number, cv::Point pos)
    {
        char text[255];
        sprintf(text, "%.2f", number);
        cv::putText(dstImg, text, pos, CV_FONT_HERSHEY_SIMPLEX, 0.25, cvScalar(0,0,0));
    }

    void vizNumber_color(cv::Mat &dstImg, float number, cv::Point pos, cv::Scalar color)
    {
        char text[255];
        sprintf(text, "%.2f", number);
        cv::putText(dstImg, text, pos, CV_FONT_HERSHEY_SIMPLEX, 0.25, color);
    }

    double diffclock(clock_t clock1,clock_t clock2)
    {
        double diffticks=clock1-clock2;
        double diffms=(diffticks)/(CLOCKS_PER_SEC/1000);
        return diffms;
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "visLoc");
    OttoCarVisLoc visLoc;
    ros::spin();
    return 0;
}

