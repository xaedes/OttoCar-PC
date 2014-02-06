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

bool debugMode = true;
static const std::string OPENCV_WINDOW_rgb = "rgb input";
static const std::string OPENCV_WINDOW_topview = "topview";
static const std::string OPENCV_WINDOW_topview_roi = "topview ROI";
static const std::string OPENCV_WINDOW_lines = "line detection ";
static const std::string OPENCV_WINDOW_lines_morph = "line detection morphed";
static const std::string OPENCV_WINDOW_lanes_hor = "test lane state";
static const std::string OPENCV_WINDOW_lanes_vert = "lines final";

const char* projMatFilename = "visLoc_TopviewCalib_ProjMat.txt";
const char* configFilename = "visLoc_TopviewCalib_Config.txt";

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

int LANE_DETECTION_LANE_WIDTH = 57; // lane width in pixel of topview
int LANE_DETECTION_POS_DELTA = 10; // position delta in +-pixel of topview
float LANE_DETECTION_ANGLE_DELTA = 10.0; // angle delta in +-degree

int LANE_STATE_CANDIDATE_POS_DELTA = 20; //position delta in +-pixel of topview for setting lane state candidates
float LANE_STATE_CANDIDATE_ANGLE_DELTA = 20.0; //angle delta in +-degree between candidate and last lane state





enum LineType {LANE_BORDER_CONTINOUS=0, LANE_BORDER_DASHED=1, FALSE_POSITIVE=2};
    

class OttoCarVisLoc{

     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;
     image_transport::Subscriber image_sub_rgb;
     ros::Publisher perfPub;
     ros::Publisher laneStatePub;
     std_msgs::Float64MultiArray perfArray;
     std_msgs::Float64MultiArray laneStateArray;

     cv::Mat rgb_img, topviewRoi_img, topview_img, H, lines_img, linesMorph_img, lanes_img_hor,lanes_img_vert, meanQuadraticNeighborKernel, meanHorizontalNeighborKernel;
     cv::Size topviewSize;
     cv::Point carPos;

     ros::Time inputTimestamp;

     //detectLaneMarksV2
     std::vector<std::vector<cv::Point> > lineChainList, polylineList, polylineListDashed, polylineListFinal ;
     std::vector<std::pair<float, float> > polyLinesParamList; // <lineLength, angleChangeConf>
     std::vector<LineType> polyLineTypeList;

     typedef std::pair<std::pair<cv::Point, cv::Point2f>, std::pair<float, LineType> > StateTriple;
     typedef std::vector<StateTriple> ListOfStateTriples;
     ListOfStateTriples PolyLineStateTripleList, BorderLeftCandidateList, BorderMidCandidateList, BorderRightCandidateList, LaneStateTripleCandidate, LaneStateTriple_current, LaneStateTriple_last; // Laneborder_left ; LaneBorder_mid; Laneborder_right
     // point; vec; confidence ; linetype

    public:
    OttoCarVisLoc(): it_(nh_)
    {

        // load CalibConfig and TopviewProjMat
        H.create(3,3,CV_64F);
        loadDoubleMatFromFile(H, projMatFilename, 3, 3);
        loadCalibConfigFromFile(topviewSize, carPos, configFilename);

        LaneStateTriple_current.clear();
        LaneStateTriple_last.clear();

        // Subscrive to input video feed and publish output
        image_sub_rgb = it_.subscribe("/usb_cam/image_raw", 1, &OttoCarVisLoc::imageMsgConvert_rgb, this);
        perfPub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/perf", 100);
        laneStatePub= nh_.advertise<std_msgs::Float64MultiArray>("ottocar_perception/laneState", 100);

        if(debugMode)
        {
            //cv::namedWindow(OPENCV_WINDOW_rgb);
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
            //cv::destroyWindow(OPENCV_WINDOW_rgb);
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
            createHorizontalMeanNeighborKernel(meanHorizontalNeighborKernel, NEIGHBOR_KERNEL_SIZE);
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
        PolyLineStateTripleList.clear();

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

        //Lines Image preprocessing
        // Morphological open
        cv::Mat tmpThresh = topviewThresholdImgSmall.clone();
        cv::Mat opingKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(3, 3));
        cv::erode(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, opingKernel);
        cv::dilate(tmpTopviewThresholdImgSmall, topviewThresholdImgSmall, opingKernel);

        // moprhological close in vertical direction
        cv::Mat tmpThreshClosed = topviewThresholdImgSmall.clone();
        cv::Mat closingKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(3, 15));
        cv::dilate(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, closingKernel);
        cv::erode(tmpTopviewThresholdImgSmall, topviewThresholdImgSmall, closingKernel);

        cv::line(topviewThresholdImgSmall, cv::Point(120,180), cv::Point(175,180), CV_RGB(255,255,255), 3, CV_AA);

        // thinning
        cv::Mat tmpThreshThinned = topviewThresholdImgSmall.clone();
        MorphThinning(topviewThresholdImgSmall, topviewThresholdImgSmall);

        // Morphological open
        cv::Mat tmpThreshVert = topviewThresholdImgSmall.clone();
        cv::Mat opingKernelVert = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(1, 3));
        cv::erode(topviewThresholdImgSmall, tmpTopviewThresholdImgSmall, opingKernelVert);
        cv::dilate(tmpTopviewThresholdImgSmall, topviewThresholdImgSmall, opingKernelVert);

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
        if(debugMode)   drawPolyLinesFinal(dstLanes);

        if(LaneStateTriple_current.empty() && LaneStateTriple_last.empty()) initCurrLaneState();

        //project polylines to base line
        projectPolylinesToBaseline();
        //lane detection
        laneDetection();
        //set candidate Triple
        setLaneStateCandidates();
        //update lane state triple
        updateLaneState();

        if(debugMode)   vizCurrLaneState(lanes_img_hor);
        if(debugMode)   vizLaneStateList(lanes_img_hor, PolyLineStateTripleList);

        //calc return values for module interface
        setLaneStateArray();

        // resize for output visualisation
        cv::resize(topviewThresholdImgSmall, topviewThresholdImg, cv::Size(), resizeFactor, resizeFactor);



        dstTopviewThreshold = /*tmpThreshClosed*/ tmpThreshVert;
        dstTopviewThresholdMorph = topviewThresholdImg;//polyLineImg;
        //dstLanes
    }

    void setLaneStateArray()
    {
        float confSum = LaneStateTriple_current[0].second.first + LaneStateTriple_current[1].second.first + LaneStateTriple_current[2].second.first;
        float w_left = LaneStateTriple_current[0].second.first / confSum;
        float w_mid = LaneStateTriple_current[1].second.first / confSum;
        float w_right = LaneStateTriple_current[2].second.first / confSum;

        //set x position of averaged mid lane
        double posX = (double)(w_left*(LaneStateTriple_current[0].first.first.x + LANE_DETECTION_LANE_WIDTH) +
                                w_mid*LaneStateTriple_current[1].first.first.x +
                                w_right*(LaneStateTriple_current[2].first.first.x - LANE_DETECTION_LANE_WIDTH));
        //set vec
        cv::Point2f laneDir;
        laneDir.x = (float)(w_left*LaneStateTriple_current[0].first.second.x + w_mid*LaneStateTriple_current[1].first.second.x + w_right*LaneStateTriple_current[2].first.second.x);
        laneDir.x = (float)(w_left*LaneStateTriple_current[0].first.second.y + w_mid*LaneStateTriple_current[1].first.second.y + w_right*LaneStateTriple_current[2].first.second.y);
        //set confidence
        double confidence = (double)(w_left*LaneStateTriple_current[0].second.first + w_mid*LaneStateTriple_current[1].second.first + w_right*LaneStateTriple_current[2].second.first);

        // calc deltas
        double deltaPosX_CarPosX = (double)(carPos.x - posX);
        cv::Point2f CarDir(1.0, 0.0);
        double deltaLaneAngle = (double)calcAngleBetweenVecs_float(CarDir, laneDir);


        laneStateArray.data.clear();
        laneStateArray.data.push_back(inputTimestamp.toSec());
        laneStateArray.data.push_back(deltaPosX_CarPosX);
        laneStateArray.data.push_back(deltaLaneAngle);
        laneStateArray.data.push_back(confidence);
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


    void vizCurrLaneState(cv::Mat &dstImg)
    {
        int vizLine = 200;

        cv::Scalar color;
        for(int i=0; i<LaneStateTriple_current.size(); i++)
        {

            if(i == 0)       color = CV_RGB(255,0,0);
            else if(i == 1)  color = CV_RGB(0,255,0);
            else if(i == 2)  color = CV_RGB(0,0,255);
            cv::line( dstImg, cv::Point(LaneStateTriple_current[i].first.first.x, vizLine),
                    cv::Point(LaneStateTriple_current[i].first.first.x - (int)(40*LaneStateTriple_current[i].first.second.x),
                              vizLine - (int)(40*LaneStateTriple_current[i].first.second.y)), color, 1, CV_AA);
            vizNumber(dstImg, (float)LaneStateTriple_current[i].second.first, cv::Point(LaneStateTriple_current[i].first.first.x, vizLine-30));

            // viz construction range for current lane state
            cv::line( dstImg, cv::Point(LaneStateTriple_last[i].first.first.x - LANE_STATE_CANDIDATE_POS_DELTA, vizLine),
                    cv::Point(LaneStateTriple_last[i].first.first.x - LANE_STATE_CANDIDATE_POS_DELTA,
                              vizLine-10),color, 1, CV_AA);

            cv::line( dstImg, cv::Point(LaneStateTriple_last[i].first.first.x + LANE_STATE_CANDIDATE_POS_DELTA, vizLine),
                    cv::Point(LaneStateTriple_last[i].first.first.x + LANE_STATE_CANDIDATE_POS_DELTA,
                              vizLine-10),color, 1, CV_AA);

        }
    }


    void vizLaneStateList(cv::Mat &dstImg, ListOfStateTriples &inputList )
    {
        cv::Scalar color;
        for(int i=0; i<inputList.size(); i++)
        {
            if(inputList[i].second.second == LANE_BORDER_CONTINOUS)
            {
                color = CV_RGB(255,0,255);
                vizNumber(dstImg, (float)inputList[i].second.first, inputList[i].first.first);
            }
            if(inputList[i].second.second == LANE_BORDER_DASHED)
            {
                color = CV_RGB(0,255,255);
                vizNumber(dstImg, (float)inputList[i].second.first, cv::Point(inputList[i].first.first.x, inputList[i].first.first.y - 10));
            }
            cv::line( dstImg, inputList[i].first.first,
                    cv::Point(inputList[i].first.first.x - (int)(40*inputList[i].first.second.x),
                              inputList[i].first.first.y - (int)(40*inputList[i].first.second.y)),
                    color, 1, CV_AA);

        }
    }


    void laneDetection()
    {
        for(int i=0; i<PolyLineStateTripleList.size(); i++)
        {
            //for each dashed line
            if(PolyLineStateTripleList[i].second.second == LANE_BORDER_DASHED)
            {
                for(int k=0; k<PolyLineStateTripleList.size(); k++)
                {
                    if(k==i) continue;
                    if(PolyLineStateTripleList[k].second.second == LANE_BORDER_DASHED) continue;
                    if(!checkVecsForSameDir(PolyLineStateTripleList[k].first.second, PolyLineStateTripleList[i].first.second)) continue;

                    //left corresponding line found
                    if(PolyLineStateTripleList[k].first.first.x < PolyLineStateTripleList[i].first.first.x - LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                       PolyLineStateTripleList[k].first.first.x > PolyLineStateTripleList[i].first.first.x - LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                    {
                        //increase conf
                        PolyLineStateTripleList[k].second.first = PolyLineStateTripleList[k].second.first + 0.5;
                        PolyLineStateTripleList[i].second.first = PolyLineStateTripleList[i].second.first + 0.5;
                    }
                    //right corresponding line found
                    if(PolyLineStateTripleList[k].first.first.x < PolyLineStateTripleList[i].first.first.x + LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                       PolyLineStateTripleList[k].first.first.x > PolyLineStateTripleList[i].first.first.x + LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                    {
                        //increase conf
                        PolyLineStateTripleList[k].second.first = PolyLineStateTripleList[k].second.first + 0.5;
                        PolyLineStateTripleList[i].second.first = PolyLineStateTripleList[i].second.first + 0.5;
                    }
                }
            }
            //for each continous line
            else if(PolyLineStateTripleList[i].second.second == LANE_BORDER_CONTINOUS)
            {
                for(int k=0; k<PolyLineStateTripleList.size(); k++)
                {
                    if(k==i) continue;
                    if(!checkVecsForSameDir(PolyLineStateTripleList[k].first.second, PolyLineStateTripleList[i].first.second)) continue;

                    // current is dashed
                    if(PolyLineStateTripleList[k].second.second == LANE_BORDER_DASHED)
                    {
                        //left corresponding mid line found
                        if(PolyLineStateTripleList[k].first.first.x < PolyLineStateTripleList[i].first.first.x - LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateTripleList[k].first.first.x > PolyLineStateTripleList[i].first.first.x - LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateTripleList[k].second.first = PolyLineStateTripleList[k].second.first + 0.5;
                            PolyLineStateTripleList[i].second.first = PolyLineStateTripleList[i].second.first + 0.5;
                        }
                        //right corresponding mid line found
                        if(PolyLineStateTripleList[k].first.first.x < PolyLineStateTripleList[i].first.first.x + LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateTripleList[k].first.first.x > PolyLineStateTripleList[i].first.first.x + LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateTripleList[k].second.first = PolyLineStateTripleList[k].second.first + 0.5;
                            PolyLineStateTripleList[i].second.first = PolyLineStateTripleList[i].second.first + 0.5;
                        }
                    }
                    // current is continous
                    else if(PolyLineStateTripleList[k].second.second == LANE_BORDER_CONTINOUS)
                    {
                        //left corresponding line found
                        if(PolyLineStateTripleList[k].first.first.x < PolyLineStateTripleList[i].first.first.x - 2*LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateTripleList[k].first.first.x > PolyLineStateTripleList[i].first.first.x - 2*LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateTripleList[k].second.first = PolyLineStateTripleList[k].second.first + 0.5;
                            PolyLineStateTripleList[i].second.first = PolyLineStateTripleList[i].second.first + 0.5;
                        }
                        //right corresponding line found
                        if(PolyLineStateTripleList[k].first.first.x < PolyLineStateTripleList[i].first.first.x + 2*LANE_DETECTION_LANE_WIDTH + LANE_DETECTION_POS_DELTA &&
                           PolyLineStateTripleList[k].first.first.x > PolyLineStateTripleList[i].first.first.x + 2*LANE_DETECTION_LANE_WIDTH - LANE_DETECTION_POS_DELTA)
                        {
                            //increase conf
                            PolyLineStateTripleList[k].second.first = PolyLineStateTripleList[k].second.first + 0.5;
                            PolyLineStateTripleList[i].second.first = PolyLineStateTripleList[i].second.first + 0.5;
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
        StateTriple leftLane;
        //set point
        //todo: do it with car pos
        leftLane.first.first.x = 63;
        leftLane.first.first.y = 290;
        //set vec
        leftLane.first.second.x = (float)(0);
        leftLane.first.second.y = (float)(1);
        //normalize vec
        leftLane.first.second.x = leftLane.first.second.x / std::sqrt(std::pow(leftLane.first.second.x,2)
                                                                            + std::pow(leftLane.first.second.y,2));
        leftLane.first.second.y = leftLane.first.second.y / std::sqrt(std::pow(leftLane.first.second.x,2)
                                                                            + std::pow(leftLane.first.second.y,2));
        //set confidence
        leftLane.second.first = 0.5;
        //set linetype
        leftLane.second.second = LANE_BORDER_CONTINOUS;

        LaneStateTriple_current.push_back(leftLane);
        LaneStateTriple_last.push_back(leftLane);


        StateTriple midLane;
        //set point
        midLane.first.first.x = 120;
        midLane.first.first.y = 290;
        //set vec
        midLane.first.second.x = (float)(0);
        midLane.first.second.y = (float)(1);
        //normalize vec
        midLane.first.second.x = midLane.first.second.x / std::sqrt(std::pow(midLane.first.second.x,2)
                                                                            + std::pow(midLane.first.second.y,2));
        midLane.first.second.y = midLane.first.second.y / std::sqrt(std::pow(midLane.first.second.x,2)
                                                                            + std::pow(midLane.first.second.y,2));
        //set confidence
        midLane.second.first = 0.5;
        //set linetype
        midLane.second.second = LANE_BORDER_DASHED;

        LaneStateTriple_current.push_back(midLane);
        LaneStateTriple_last.push_back(midLane);


        StateTriple rightLane;
        //set point
        rightLane.first.first.x = 177;
        rightLane.first.first.y = 290;
        //set vec
        rightLane.first.second.x = (float)(0);
        rightLane.first.second.y = (float)(1);
        //normalize vec
        rightLane.first.second.x = rightLane.first.second.x / std::sqrt(std::pow(rightLane.first.second.x,2)
                                                                            + std::pow(rightLane.first.second.y,2));
        rightLane.first.second.y = rightLane.first.second.y / std::sqrt(std::pow(rightLane.first.second.x,2)
                                                                            + std::pow(rightLane.first.second.y,2));
        //set confidence
        rightLane.second.first = 0.5;
        //set linetype
        rightLane.second.second = LANE_BORDER_CONTINOUS;

        LaneStateTriple_current.push_back(rightLane);
        LaneStateTriple_last.push_back(rightLane);
    }


    void projectPolylinesToBaseline()
    {
        // set polyLineTripleList
        for(int i=0; i<polylineList.size(); i++)
        {
            StateTriple stateTriple;
            //todo: project point instead of just copying x value + use car pos for base line
            //set point
            stateTriple.first.first.x = polylineList[i][0].x;
            stateTriple.first.first.y = 290;
            //set vec
            stateTriple.first.second.x = (float)(polylineList[i][0].x - polylineList[i][1].x);
            stateTriple.first.second.y = (float)(polylineList[i][0].y - polylineList[i][1].y);
            //normalize vec
            stateTriple.first.second.x = stateTriple.first.second.x / std::sqrt(std::pow(stateTriple.first.second.x,2)
                                                                                + std::pow(stateTriple.first.second.y,2));
            stateTriple.first.second.y = stateTriple.first.second.y / std::sqrt(std::pow(stateTriple.first.second.x,2)
                                                                                + std::pow(stateTriple.first.second.y,2));
            //set confidence
            stateTriple.second.first = polyLinesParamList[i].second;

            //set linetype
            stateTriple.second.second = polyLineTypeList[i];

            PolyLineStateTripleList.push_back(stateTriple);
        }
    }


    void setLaneStateCandidates()
    {
        LaneStateTripleCandidate.clear();
        // also possible without folowing lists
        BorderLeftCandidateList.clear();
        BorderMidCandidateList.clear();
        BorderRightCandidateList.clear();

        //todo: limit ranges to images ranges
        int leftRange[2] = {LaneStateTriple_last[0].first.first.x - LANE_STATE_CANDIDATE_POS_DELTA, LaneStateTriple_last[0].first.first.x + LANE_STATE_CANDIDATE_POS_DELTA};
        int midRange[2] = {LaneStateTriple_last[1].first.first.x - LANE_STATE_CANDIDATE_POS_DELTA, LaneStateTriple_last[1].first.first.x + LANE_STATE_CANDIDATE_POS_DELTA};
        int rightRange[2] = {LaneStateTriple_last[2].first.first.x - LANE_STATE_CANDIDATE_POS_DELTA, LaneStateTriple_last[2].first.first.x + LANE_STATE_CANDIDATE_POS_DELTA};

        float bestLeftConf[2] = {-1.0, -1.0}; // val; id in border candidate list
        float bestMidConf[2] = {-1.0, -1.0};
        float bestRightConf[2] = {-1.0, -1.0};
        int leftCounter = -1;
        int midCounter = -1;
        int rightCounter = -1;


        // SEARCH THE BEST FITTING LINE
        // ----------------------------
        for(int i=0; i<PolyLineStateTripleList.size(); i++)
        {
            StateTriple currStateTriple = PolyLineStateTripleList[i];
            // test on left angle difference
            //last timestep is still current till it will be updated later in updateLaneState()!!!
            if(std::abs(calcAngleBetweenVecs_float(currStateTriple.first.second, LaneStateTriple_current[0].first.second)) < LANE_STATE_CANDIDATE_ANGLE_DELTA)
            {
                //test on left range
                if(currStateTriple.first.first.x > leftRange[0] && currStateTriple.first.first.x < leftRange[1] &&
                        currStateTriple.second.second == LANE_BORDER_CONTINOUS)
                {
                    BorderLeftCandidateList.push_back(currStateTriple);
                    leftCounter++;
                    if(bestLeftConf[0] < currStateTriple.second.first)
                    {
                        bestLeftConf[0] = currStateTriple.second.first;
                        bestLeftConf[1] = (float)leftCounter;
                    }
                }
            }

            // test on mid angle difference
            if(std::abs(calcAngleBetweenVecs_float(currStateTriple.first.second, LaneStateTriple_current[1].first.second)) < LANE_STATE_CANDIDATE_ANGLE_DELTA)
            {
                // test on mid range
                if(currStateTriple.first.first.x > midRange[0] && currStateTriple.first.first.x < midRange[1] &&
                        currStateTriple.second.second == LANE_BORDER_DASHED)
                {
                    BorderMidCandidateList.push_back(currStateTriple);
                    midCounter++;
                    if(bestMidConf[0] < currStateTriple.second.first)
                    {
                        bestMidConf[0] = currStateTriple.second.first;
                        bestMidConf[1] = (float)midCounter;
                    }
                }
            }
            // test on mid angle difference
            if(std::abs(calcAngleBetweenVecs_float(currStateTriple.first.second, LaneStateTriple_current[2].first.second)) < LANE_STATE_CANDIDATE_ANGLE_DELTA)
            {
                // test on right range
                if(currStateTriple.first.first.x > rightRange[0] && currStateTriple.first.first.x < rightRange[1] &&
                        currStateTriple.second.second == LANE_BORDER_CONTINOUS)
                {
                    BorderRightCandidateList.push_back(currStateTriple);
                    rightCounter++;
                    if(bestRightConf[0] < currStateTriple.second.first)
                    {
                        bestRightConf[0] = currStateTriple.second.first;
                        bestRightConf[1] = (float)rightCounter;
                    }
                }
            }

        }


        // SET LANE STATE CANDIDATES
        // -------------------------
        StateTriple dummy;
        int lineFound[3] = {-1,-1,-1};
        int foundSum = 0;
        if((int)bestLeftConf[1] == -1)
        {
            lineFound[0] = 0;
            LaneStateTripleCandidate.push_back(dummy);
        }
        else
        {
            lineFound[0] = 1;
            foundSum++;
            LaneStateTripleCandidate.push_back(BorderLeftCandidateList[(int)bestLeftConf[1]]);
        }

        if((int)bestMidConf[1] == -1)
        {
            lineFound[1] = 0;
            LaneStateTripleCandidate.push_back(dummy);
        }
        else
        {
            lineFound[1] = 1;
            foundSum++;
            LaneStateTripleCandidate.push_back(BorderMidCandidateList[(int)bestMidConf[1]]);
        }

        if((int)bestRightConf[1] == -1)
        {
            lineFound[2] = 0;
            LaneStateTripleCandidate.push_back(dummy);
        }
        else
        {
            lineFound[2] = 1;
            foundSum++;
            LaneStateTripleCandidate.push_back(BorderRightCandidateList[(int)bestRightConf[1]]);
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
                    LaneStateTripleCandidate[0].first.first.x = LaneStateTripleCandidate[1].first.first.x - (LaneStateTripleCandidate[2].first.first.x - LaneStateTripleCandidate[1].first.first.x);
                    LaneStateTripleCandidate[0].first.first.y = LaneStateTripleCandidate[1].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[0].first.second.x = 0.5 * LaneStateTripleCandidate[1].first.second.x + 0.5 * LaneStateTripleCandidate[2].first.second.x;
                    LaneStateTripleCandidate[0].first.second.y = 0.5 * LaneStateTripleCandidate[1].first.second.y + 0.5 * LaneStateTripleCandidate[2].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[0].second.first = 0.5 * LaneStateTripleCandidate[1].second.first + 0.5 * LaneStateTripleCandidate[2].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[0].second.second = LANE_BORDER_CONTINOUS;
                }
                else if(lineFound[1] == 0)  //mid
                {
                    //update point
                    LaneStateTripleCandidate[1].first.first.x = LaneStateTripleCandidate[0].first.first.x + (int)(0.5 * (float)(LaneStateTripleCandidate[2].first.first.x - LaneStateTripleCandidate[0].first.first.x));
                    LaneStateTripleCandidate[1].first.first.y = LaneStateTripleCandidate[0].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[1].first.second.x = 0.5 * LaneStateTripleCandidate[0].first.second.x + 0.5 * LaneStateTripleCandidate[2].first.second.x;
                    LaneStateTripleCandidate[1].first.second.y = 0.5 * LaneStateTripleCandidate[0].first.second.y + 0.5 * LaneStateTripleCandidate[2].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[1].second.first = 0.5 * LaneStateTripleCandidate[0].second.first + 0.5 * LaneStateTripleCandidate[2].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[1].second.second = LANE_BORDER_DASHED;
                }
                else if(lineFound[2] == 0)  //rigth
                {
                    //update point
                    LaneStateTripleCandidate[2].first.first.x = LaneStateTripleCandidate[1].first.first.x + (LaneStateTripleCandidate[1].first.first.x - LaneStateTripleCandidate[0].first.first.x);
                    LaneStateTripleCandidate[2].first.first.y = LaneStateTripleCandidate[0].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[2].first.second.x = 0.5 * LaneStateTripleCandidate[0].first.second.x + 0.5 * LaneStateTripleCandidate[1].first.second.x;
                    LaneStateTripleCandidate[2].first.second.y = 0.5 * LaneStateTripleCandidate[0].first.second.y + 0.5 * LaneStateTripleCandidate[1].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[2].second.first = 0.5 * LaneStateTripleCandidate[0].second.first + 0.5 * LaneStateTripleCandidate[1].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[2].second.second = LANE_BORDER_CONTINOUS;
                }
            }
            // simulate two lines
            else if(foundSum == 1)
            {
                if(lineFound[0] == 1)   //left
                {
                    //update point
                    LaneStateTripleCandidate[1].first.first.x = LaneStateTripleCandidate[0].first.first.x + LANE_DETECTION_LANE_WIDTH;
                    LaneStateTripleCandidate[1].first.first.y = LaneStateTripleCandidate[0].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[1].first.second.x = LaneStateTripleCandidate[0].first.second.x;
                    LaneStateTripleCandidate[1].first.second.y = LaneStateTripleCandidate[0].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[1].second.first = LaneStateTripleCandidate[0].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[1].second.second = LANE_BORDER_DASHED;

                    //update point
                    LaneStateTripleCandidate[2].first.first.x = LaneStateTripleCandidate[1].first.first.x + LANE_DETECTION_LANE_WIDTH;
                    LaneStateTripleCandidate[2].first.first.y = LaneStateTripleCandidate[0].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[2].first.second.x = LaneStateTripleCandidate[0].first.second.x;
                    LaneStateTripleCandidate[2].first.second.y = LaneStateTripleCandidate[0].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[2].second.first = LaneStateTripleCandidate[0].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[2].second.second = LANE_BORDER_CONTINOUS;
                }
                else if(lineFound[1] == 1)  //mid
                {
                    //update point
                    LaneStateTripleCandidate[0].first.first.x = LaneStateTripleCandidate[1].first.first.x - LANE_DETECTION_LANE_WIDTH;
                    LaneStateTripleCandidate[0].first.first.y = LaneStateTripleCandidate[1].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[0].first.second.x = LaneStateTripleCandidate[1].first.second.x;
                    LaneStateTripleCandidate[0].first.second.y = LaneStateTripleCandidate[1].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[0].second.first = LaneStateTripleCandidate[1].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[0].second.second = LANE_BORDER_CONTINOUS;

                    //update point
                    LaneStateTripleCandidate[2].first.first.x = LaneStateTripleCandidate[1].first.first.x + LANE_DETECTION_LANE_WIDTH;
                    LaneStateTripleCandidate[2].first.first.y = LaneStateTripleCandidate[1].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[2].first.second.x = LaneStateTripleCandidate[1].first.second.x;
                    LaneStateTripleCandidate[2].first.second.y = LaneStateTripleCandidate[1].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[2].second.first = LaneStateTripleCandidate[1].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[2].second.second = LANE_BORDER_CONTINOUS;
                }
                else if(lineFound[2] == 1)  //rigth
                {
                    //update point
                    LaneStateTripleCandidate[1].first.first.x = LaneStateTripleCandidate[2].first.first.x - LANE_DETECTION_LANE_WIDTH;
                    LaneStateTripleCandidate[1].first.first.y = LaneStateTripleCandidate[2].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[1].first.second.x = LaneStateTripleCandidate[2].first.second.x;
                    LaneStateTripleCandidate[1].first.second.y = LaneStateTripleCandidate[2].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[1].second.first = LaneStateTripleCandidate[2].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[1].second.second = LANE_BORDER_DASHED;

                    //update point
                    LaneStateTripleCandidate[0].first.first.x = LaneStateTripleCandidate[1].first.first.x - LANE_DETECTION_LANE_WIDTH;
                    LaneStateTripleCandidate[0].first.first.y = LaneStateTripleCandidate[2].first.first.y; // baseline is everytime in the same height
                    //update vec
                    LaneStateTripleCandidate[0].first.second.x = LaneStateTripleCandidate[2].first.second.x;
                    LaneStateTripleCandidate[0].first.second.y = LaneStateTripleCandidate[2].first.second.y;
                    //update confidence
                    LaneStateTripleCandidate[0].second.first = LaneStateTripleCandidate[2].second.first;
                    //lane type keeps the same
                    LaneStateTripleCandidate[0].second.second = LANE_BORDER_CONTINOUS;
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
                LaneStateTripleCandidate[i].first.first.x = LaneStateTriple_current[i].first.first.x;
                LaneStateTripleCandidate[i].first.first.y = LaneStateTriple_current[i].first.first.y;
                //update vec
                LaneStateTripleCandidate[i].first.second.x = LaneStateTriple_current[i].first.second.x;
                LaneStateTripleCandidate[i].first.second.y = LaneStateTriple_current[i].first.second.y;
                //update confidence
                LaneStateTripleCandidate[i].second.first = LaneStateTriple_current[i].second.first;
                //lane type keeps the same
                LaneStateTripleCandidate[i].second.second = LaneStateTriple_current[i].second.second;

            }
        }
    }


    void updateLaneState()
    {
        //ROS_INFO_STREAM("a");
        // set time step
        LaneStateTriple_last.clear();
        StateTriple laneBorder;
        laneBorder = LaneStateTriple_current[0];
        LaneStateTriple_last.push_back(laneBorder);
        laneBorder = LaneStateTriple_current[1];
        LaneStateTriple_last.push_back(laneBorder);
        laneBorder = LaneStateTriple_current[2];
        LaneStateTriple_last.push_back(laneBorder);
        LaneStateTriple_current.clear();

        //ROS_INFO_STREAM("b");

        float k = 0.5;

        //todo: if variance to last step is too big, let it out



        // a(t) = k * a(t) + (1-k) * a(t-1)
        // go to lanes
        for(int i=0; i<LaneStateTripleCandidate.size(); i++)
        {
            //ROS_INFO_STREAM("it:" << i);
            StateTriple laneBorder;
            //update point
            laneBorder.first.first.x = (int)round((double) k * LaneStateTripleCandidate[i].first.first.x + (1.0-k) * LaneStateTriple_last[i].first.first.x);
            laneBorder.first.first.y = (int)round((double) k * LaneStateTripleCandidate[i].first.first.y + (1.0-k) * LaneStateTriple_last[i].first.first.y);
            //update vec
            laneBorder.first.second.x = k * LaneStateTripleCandidate[i].first.second.x + (1.0-k) * LaneStateTriple_last[i].first.second.x;
            laneBorder.first.second.y = k * LaneStateTripleCandidate[i].first.second.y + (1.0-k) * LaneStateTriple_last[i].first.second.y;
            //update confidence
            laneBorder.second.first = k * LaneStateTripleCandidate[i].second.first + (1.0-k) * LaneStateTriple_last[i].second.first;
            //lane type keeps the same

            LaneStateTriple_current.push_back(laneBorder);
        }
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
            if(angle2Next > 10.0) continue;

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
            if(debugMode)
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
    {// left direction is positiv angle
     // rigth direction is negativ angle
        float angle = (std::atan2((float)src_vec.y, (float)src_vec.x) - std::atan2((float)trgt_vec.y, (float)trgt_vec.x)) * 180.0 / CV_PI;
        return (-1.0*angle);
    }

    float calcAngleBetweenVecs_float(cv::Point2f src_vec, cv::Point2f trgt_vec)
    {// left direction is positiv angle
     // rigth direction is negativ angle
        float angle = (std::atan2(src_vec.y, src_vec.x) - std::atan2(trgt_vec.y, trgt_vec.x)) * 180.0 / CV_PI;
        return (-1.0*angle);
    }

    void vizNumber(cv::Mat &dstImg, float number, cv::Point pos)
    {
        char text[255];
        sprintf(text, "%.2f", number);
        cv::putText(dstImg, text, pos, CV_FONT_HERSHEY_SIMPLEX, 0.25, cvScalar(0,0,0));
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

