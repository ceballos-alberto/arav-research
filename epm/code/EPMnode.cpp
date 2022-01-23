
/* ----------------------------------------------------------------------------
   ---------------- Environment Perception Module (EPM) - ARAV ----------------
   ----------------------------------------------------------------------------
   -------------------- Author : Alberto Ceballos Gonzalez --------------------
   -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
   --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved --------
   ---------------------------------------------------------------------------- */

/* Import required libraries */

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/publisher.h>
#include <tf/transform_listener.h>

/* Namespaces */

using namespace std;
using namespace cv;
using namespace cv_bridge;
using namespace pcl;
using namespace sensor_msgs;
using namespace std_msgs;
using namespace ros;
using namespace pcl_ros;
using namespace image_transport;
using namespace tf;

/* Topic names - (Global variables) */

/* ---------- INPUTS ---------- */

static const std::string INPUT_TOPIC_RIGHT = "/arav/sensors/frontRightCamera/image";
static const std::string INPUT_TOPIC_LEFT = "/arav/sensors/frontLeftCamera/image";
static const std::string INPUT_FROM_FILTER = "/arav/EPM/AIFilterOutput";
static const std::string STAT_TOPIC_IN = "/arav/EPM/AIFilterStatus";

static const std::string INPUT_TOPIC_ULT_1 = "/arav/sensors/frontUltrasonic";
static const std::string INPUT_TOPIC_ULT_2 = "/arav/sensors/leftUltrasonic";
static const std::string INPUT_TOPIC_ULT_3 = "/arav/sensors/rightUltrasonic";

/* ---------- OUTPUTS --------- */

static const std::string OUTPUT_TOPIC = "/arav/EPM/output/cloud";
static const std::string OUTPUT_TOPIC_MODE = "/arav/EPM/output/mode";
static const std::string OUTPUT_TOPIC_ULT = "/arav/EPM/output/cloudUltrasonic";
static const std::string OUTPUT_FOR_FILTER = "/arav/EPM/AIFilterInput";
static const std::string STAT_TOPIC_OUT = "/arav/EPM/Status";

static const std::string VISUAL_DEPTH = "/arav/visual/depth";
static const std::string VISUAL_CLOUD = "/arav/visual/cloud";


/* Image Size - (Global variables) --> HD 720p */

static const int WIDTH = 1280;	/* This line can be modified */
static const int HEIGHT = 720;	/* This line can be modified */

/* Constants */

static const float PI = 3.14159;
static const int FINAL_FRAME = 3;

/* Listener Class definition */

class Listener {

	private:	/* Class Attributes */

		cv::Mat image;
		std::vector<short int> boundingBoxes;
		float range;
		int statusAI = 0;

	public:	/* Class Methods */

		void statusCallback (const std_msgs::Int8::ConstPtr& msg) {

			/* To be executed when a status message is received */

			statusAI = msg->data;

		}

		void imageCallback (const sensor_msgs::ImageConstPtr& msg) {

			/* To be executed when an image is received */

			try {

				/* Convert from ROS message to OpenCV image */

				cv_bridge::CvImagePtr imagePtr = cv_bridge::toCvCopy (msg, "bgr8");
				image = imagePtr->image;

			}

			catch (cv_bridge::Exception& e) {

			}

		}

		void boundingBoxCallback (const std_msgs::Int16MultiArray::ConstPtr& msg) {

			/* To be executed when a bounding box is received */

			boundingBoxes = msg->data;

		}

		void rangeCallback (const sensor_msgs::RangeConstPtr& msg) {

			/* To be executed when data from the ultrasonic sensor is received */

			range = msg->range;

		}

		/* Getters for class attributes */

		cv::Mat getImage () {

			return image;

		}

		std::vector<short int> getBoundingBoxes () {

			return boundingBoxes;

		}

		float getRange () {

			return range;

		}

		int getStatusAI () {

			return statusAI;

		}

};

/* Depth Map Class Definition */

class DepthMap {

	private:	/* Class Attributes */

		pcl::PointCloud<pcl::PointXYZ> pclPointCloud, ultPointCloud;

		sensor_msgs::PointCloud2 outputCloud, outputCloudUlt;

		cv::Mat image_right_rect, image_left_rect;

		cv::Mat mapRightX, mapRightY, mapLeftX, mapLeftY, Q;

		cv::Mat depthMap, depthMap_vis;

		cv::Ptr<cv::StereoSGBM> left_matcher;

		cv::Ptr<cv::StereoMatcher> right_matcher;

		cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

		std::vector < std::vector<int> > pointCoord;

		sensor_msgs::ImagePtr msgForFilter;
		
		sensor_msgs::ImagePtr msgForVisual1;
		
		sensor_msgs::ImagePtr msgForVisual2;

	public:	/* Class Methods */

		void setParams () {

			/* Disparity Parameters */

			int minDisparity = -64;		/* This line can be modified */
			int numDisparities = 192;		/* This line can be modified */
			int blockSize = 5;			/* This line can be modified */
			int preFilterCap = 4;			/* This line can be modified */
			int uniquenessRatio = 1;		/* This line can be modified */
			int speckleWindowSize = 150;		/* This line can be modified */
			int speckleRange = 2;			/* This line can be modified */
			int disp12MaxDiff = 10;		/* This line can be modified */
			int p1 = 600;				/* This line can be modified */
			int p2 = 2400;				/* This line can be modified */

			/* WLS Filter Parameters */

			double lambda = 8000.0;		/* This line can be modified */
			double sigma = 4;			/* This line can be modified */

			/* Point Cloud Parameters */

			pclPointCloud.height = 1;			/* This line can be modified */
        		pclPointCloud.header.frame_id = "odom";	/* This line can be modified */

        		ultPointCloud.height = 1;			/* This line can be modified */
        		ultPointCloud.header.frame_id = "odom";	/* This line can be modified */

			/* Agorithm Configuration */

			left_matcher = cv::StereoSGBM::create ();
			left_matcher->setMinDisparity (minDisparity);
			left_matcher->setNumDisparities (numDisparities);
			left_matcher->setBlockSize (blockSize);
			left_matcher->setPreFilterCap (preFilterCap);
			left_matcher->setUniquenessRatio (uniquenessRatio);
			left_matcher->setSpeckleWindowSize (speckleWindowSize);
			left_matcher->setSpeckleRange (speckleRange);
			left_matcher->setDisp12MaxDiff (disp12MaxDiff);
			left_matcher->setP1(p1);
            		left_matcher->setP2(p2);
            		left_matcher->setMode(cv::StereoSGBM::MODE_HH);

        		right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

        		wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        		wls_filter->setLambda(lambda);
        		wls_filter->setSigmaColor(sigma);

		}

		void loadCalibrationData (std::string DATA_PATH) {

			/* Calibration Parameters */

			cv::Size img_size = cv::Size (WIDTH, HEIGHT);
			cv::Mat CM1 = cv::Mat(3, 3, CV_64FC1);
			cv::Mat CM2 = cv::Mat(3, 3, CV_64FC1);
			cv::Mat D1, D2, R, T, E, F, R1, R2, P1, P2;
			Q = cv::Mat(4, 4, CV_64FC1);

			/* Read DATA File */

			cv::FileStorage file (DATA_PATH, cv::FileStorage::READ);

				file ["CM1"] >> CM1;
			      	file ["CM2"] >> CM2;
			    	file ["D1"] >> D1;
			    	file ["D2"] >> D2;
			    	file ["R"] >> R;
			    	file ["T"] >> T;
			    	file ["E"] >> E;
			    	file ["F"] >> F;
	    			file ["R1"] >> R1;
	    			file ["R2"] >> R2;
	    			file ["P1"] >> P1;
	    			file ["P2"] >> P2;
	    			file ["Q"] >> Q;

    			file.release ();

    			/* Compute Rectification Maps */

    			cv::initUndistortRectifyMap (CM1, D1, R1, P1, img_size, CV_32FC1, mapRightX, mapRightY);
    			cv::initUndistortRectifyMap (CM2, D2, R2, P2, img_size, CV_32FC1, mapLeftX, mapLeftY);

		}

		void preProcessing (cv::Mat imgR, cv::Mat imgL, bool display, bool save, std::string SAVE_PATH, clock_t init_time) {

        		/* Image rectification */

        		cv::remap(imgR, image_right_rect, mapRightX, mapRightY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        		cv::remap(imgL, image_left_rect, mapLeftX, mapLeftY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

        		/* Convert to ROS message --> Input for AI Filter */

        		msgForFilter = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_left_rect).toImageMsg();

        		/* Color conversion --> To grayscale */

			cv::cvtColor(image_right_rect, image_right_rect, CV_BGR2GRAY, 0);
        		cv::cvtColor(image_left_rect, image_left_rect, CV_BGR2GRAY, 0);

        		/* Display rectification output */

        		if (display) {

        			cv::imshow ("Image rectification - left", image_left_rect);
        			cv::imshow ("Image rectification - right", image_right_rect);
        			cv::waitKey (15);

        		}

        		/* Save Output */

        		if (save) {

        			cv::imwrite (SAVE_PATH + "rect_left_" + std::to_string((clock()-init_time)/CLOCKS_PER_SEC).substr(0,4) + "_segs.png", image_left_rect);
        			cv::imwrite (SAVE_PATH + "rect_right_" + std::to_string((clock()-init_time)/CLOCKS_PER_SEC).substr(0,4) + "_segs.png", image_right_rect);

        		}

		}

		void compute (bool display, bool save, std::string SAVE_PATH, clock_t init_time) {

			/* Parameters */

			double minValue, maxValue;

			/* Matrices to store disparity maps */

			cv::Mat left_disparity, right_disparity;

			/* Matrix to store colorMAP */

			cv::Mat depthMap_color;

			/* Compute disparity Maps */

			left_matcher->compute(image_left_rect, image_right_rect, left_disparity);
			right_matcher->compute(image_right_rect, image_left_rect, right_disparity);

			/* Apply WLS filter */

			wls_filter->filter(left_disparity, image_left_rect, depthMap, right_disparity);

			/* Transform depth Map */

			cv::minMaxLoc (depthMap, &minValue, &maxValue);

			depthMap.convertTo(depthMap_vis, CV_8U, 255/(maxValue - minValue));

			depthMap.convertTo(depthMap, CV_32F, 1./16);

			cv::applyColorMap (depthMap_vis, depthMap_color, cv::COLORMAP_MAGMA);

			/* Display depthMap output */

			if (display) {

				imshow("Depth MAP", depthMap_color);
        			cv::waitKey (15);

        		}

        		/* Save Output */

        		if (save) {

        			cv::imwrite (SAVE_PATH + "depthMap_" + std::to_string((clock()-init_time)/CLOCKS_PER_SEC).substr(0,4) + "_segs.png", depthMap_color);

        		}
        		
        		msgForVisual1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthMap_color).toImageMsg();

		}

		void postFiltering (std::vector<short int> boundingBoxes, bool display, bool save, std::string SAVE_PATH, clock_t init_time) {

			/* Post-filtering parameters */

			int Xmin, Xmax, Ymin, Ymax, Xmean, Ymean;
			int nbBoxes;
			bool importantPixel;
			int bottom = 25;				/* This line can be modified */
			int pointReduction = 10;			/* This line can be modified */
			std::vector <uchar> referenceValues;
			std::vector <float> referenceDepth;

			/* Matrix to store colorMAP */

			cv::Mat depthMap_color;

			/* Delete data from previous frames */

			pointCoord.clear ();

			/* Compute reference point of each obstacle */

			nbBoxes = boundingBoxes.size()/4;

			for (int i=0; i<nbBoxes; i++) {

				Xmin = boundingBoxes[1+4*i];
				Xmax = boundingBoxes[3+4*i];
				Ymin = boundingBoxes[0+4*i];
				Ymax = boundingBoxes[2+4*i];

				Xmean = static_cast <int> ((Xmin+Xmax)/2);
				Ymean = static_cast <int> ((Ymin+Ymax)/2);

				referenceValues.push_back (depthMap_vis.at<uchar>(Ymean, Xmean));
				referenceDepth.push_back (depthMap.at<float>(Ymean, Xmean));

			}

			/* Iterate over the image (i,j) */

			for (int i=0; i<depthMap.rows; i++) {

				for (int j=0; j<depthMap.cols; j++) {

					importantPixel = false;

					/* Check if the pixel is inside a bounding box */

					for (int k = 0; k<nbBoxes; k++) {

						Xmin = boundingBoxes[1+4*k];
						Xmax = boundingBoxes[3+4*k];
						Ymin = boundingBoxes[0+4*k];
						Ymax = boundingBoxes[2+4*k];

						if ((i>Ymin) && (i<Ymax)) {

							if ((j>Xmin) && (j<Xmax)) {

								if ((j%pointReduction==0) && (i%pointReduction==0)) {

									std::vector <int> points = {i,j};
									pointCoord.push_back(points);

								}

								depthMap_vis.at<uchar>(i,j) = referenceValues[k];
								depthMap.at<float>(i,j) = referenceDepth[k];

								importantPixel = true;

								break;

							}

						}

					}

					if (!importantPixel) {

						depthMap_vis.at<uchar>(i,j) = bottom;

					}

				}

			}

			cv::applyColorMap (depthMap_vis, depthMap_color, cv::COLORMAP_MAGMA);

			/* Display post-filtering output */

			if (display) {

				imshow("Depth MAP with post-filtering", depthMap_color);
        			cv::waitKey (15);

        		}

        		/* Save Output */

        		if (save) {

        			cv::imwrite (SAVE_PATH + "depthFilter_" + std::to_string((clock()-init_time)/CLOCKS_PER_SEC).substr(0,4) + "_segs.png", depthMap_color);

        		}
        		
        		msgForVisual2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthMap_color).toImageMsg();

		}

		void toPointCloud (bool save, std::string SAVE_PATH, clock_t init_time, std::vector <float> position) {

			/* Parameters */

			float zOffset = 0.1;			/* This line can be modified */

			/* compute openCV Point Cloud */

			cv::Mat opencvPointCloud;
			cv::reprojectImageTo3D (depthMap, opencvPointCloud, Q, false, -1);

			/* Point Cloud configuration */

        		pclPointCloud.points.clear();
        		pclPointCloud.width = pointCoord.size();

        		/* Vector to store point coordinates */

        		std::vector <int> points (2);

			/* Convert CV to PCL Point Cloud */

			while (pointCoord.size() > 0) {

				points = pointCoord[pointCoord.size()-1];
				pointCoord.pop_back();

				cv::Point3f opencvPoint = opencvPointCloud.at<cv::Point3f> (points[0], points[1]);
				pcl::PointXYZ pclPoint;

				pclPoint.x = -opencvPoint.z;
		        	pclPoint.y = opencvPoint.x;
		        	pclPoint.z = opencvPoint.y;

		        	/* Reference Output Cloud to a fixed frame --> odom */

		        	pclPoint.x = pclPoint.x + position [0];

		        	pclPoint.y = pclPoint.y + position [1] - pclPoint.x * tan(position[2]);

		        	pclPoint.z = pclPoint.z + zOffset;

		        	/* Point converted */

		        	pclPointCloud.points.push_back(pclPoint);

			}

			/* save Point Cloud into a file */

			if (save) {

				pcl::io::savePCDFileASCII (SAVE_PATH + "cloud_" + std::to_string((clock()-init_time)/CLOCKS_PER_SEC).substr(0,4) + "_segs.pcd", pclPointCloud);

			}

			/* Transform to ROS Point Cloud message */

			pcl::toROSMsg(pclPointCloud, outputCloud);

		}

		std::string computeUltPointCloud (std::vector <float> position, float front, float right, float left) {

			/* Parameters */

			float zoffset = 0.2; 							/* This line can be modified */
			float xPosition, yPosition;
			std::string mode;
			float changeMode = 2.5;						/* This line can be modified */
			float maxRange = 5.0;							/* This line can be modified */
			std::vector <bool> insideRange = {true, true, true};
			int counter = 3;

			pcl::PointXYZ rightPoint;
        		pcl::PointXYZ leftPoint;
        		pcl::PointXYZ frontPoint;

			/* Point Cloud configuration */

        		ultPointCloud.points.clear();

			/* Max range threshold & number of points */

			if (right >= maxRange) {

			insideRange[0] = false;
			counter--;

			}

			if (left >= maxRange) {

			insideRange[1] = false;
			counter--;

			}

			if (front >= maxRange) {

			insideRange[2] = false;
			counter--;

			}

			if (counter == 0) {		/* There is no data */

				/* Important to refresh the Point Cloud when no data is available */

				ultPointCloud.width = 1;

		   		frontPoint.x = 0;
		   		frontPoint.y = 0;
		   		frontPoint.z = 0;

		   		ultPointCloud.points.push_back(frontPoint);

			}

			else {

				ultPointCloud.width = counter;

			}

        		/* convert range data to Point Cloud */

        		rightPoint.x = right * cos (PI/6);
		      	rightPoint.y = right * sin (PI/6);
		      	rightPoint.z = zoffset;

		      	leftPoint.x = left * cos (PI/6);
		      	leftPoint.y = - left * sin (PI/6);
		      	leftPoint.z = zoffset;

		      	frontPoint.x = front;
		      	frontPoint.y = 0.0;
		      	frontPoint.z = zoffset;

		      	/* Select mode --> Avoiding / Detection */

		      	if (((frontPoint.x+frontPoint.y)<changeMode) || ((leftPoint.x+leftPoint.y)<changeMode) || ((rightPoint.x+rightPoint.y)<changeMode)) {

		      		mode = "avoiding";

		      	}

		      	else {

		      		mode = "detection";

		      	}

		      	/* Reference Output Cloud to a fixed frame --> odom */

		      	/* 1 -- TRANSLATION */

		      	rightPoint.x = rightPoint.x + position [0];
		   	rightPoint.y = rightPoint.y + position [1];

		   	/* 2 -- ROTATION */

		      	xPosition = cos (-position [2]) * (rightPoint.x-position [0]) - sin (-position [2]) * (rightPoint.y-position [1]) + position [0];
		      	yPosition = sin (-position [2]) * (rightPoint.x-position [0]) + cos (-position [2]) * (rightPoint.y-position [1]) + position [1];
		      	rightPoint.x = xPosition;
		      	rightPoint.y = yPosition;

		      	/* 1 -- TRANSLATION */

		      	leftPoint.x = leftPoint.x + position [0];
		   	leftPoint.y = leftPoint.y + position [1];

		   	/* 2 -- ROTATION */

		      	xPosition = cos (-position [2]) * (leftPoint.x-position [0]) - sin (-position [2]) * (leftPoint.y-position [1]) + position [0];
		      	yPosition = sin (-position [2]) * (leftPoint.x-position [0]) + cos (-position [2]) * (leftPoint.y-position [1]) + position [1];
		      	leftPoint.x = xPosition;
		      	leftPoint.y = yPosition;

		      	/* 1 -- TRANSLATION */

		      	frontPoint.x = frontPoint.x + position [0];
		   	frontPoint.y = frontPoint.y + position [1];

		   	/* 2 -- ROTATION */

		      	xPosition = cos (-position [2]) * (frontPoint.x-position [0]) - sin (-position [2]) * (frontPoint.y-position [1]) + position [0];
		      	yPosition = sin (-position [2]) * (frontPoint.x-position [0]) + cos (-position [2]) * (frontPoint.y-position [1]) + position [1];
		      	frontPoint.x = xPosition;
		      	frontPoint.y = yPosition;

		   	/* create Point cloud */

		   	if (insideRange[0]) {

		   		ultPointCloud.points.push_back(rightPoint);

		   	}

		   	if (insideRange[1]) {

		   		ultPointCloud.points.push_back(leftPoint);

		   	}

		   	if (insideRange[2]) {

		   		ultPointCloud.points.push_back(frontPoint);

		   	}

		   	/* Transform to ROS Point Cloud message */

			pcl::toROSMsg(ultPointCloud, outputCloudUlt);

			/* Return mode */

			return mode;

		}

		/* Getters for class attributes */

		sensor_msgs::PointCloud2 getOutputCloud () {

			return outputCloud;

		}

		sensor_msgs::PointCloud2 getOutputCloudUlt () {

			return outputCloudUlt;

		}

		sensor_msgs::ImagePtr getMsgForFilter () {

			return msgForFilter;

		}
		
		sensor_msgs::ImagePtr getMsgForVisual1 () {

			return msgForVisual1;

		}
		
		sensor_msgs::ImagePtr getMsgForVisual2 () {

			return msgForVisual2;

		}

};

/* Main function */

int main (int argc, char** argv) {

	/* Paths */

	std::string DATA_PATH (argv[1]);
	std::string SAVE_1_PATH;
	std::string SAVE_2_PATH;
	std::string SAVE_3_PATH;
	std::string SAVE_4_PATH;

	int level = 0;

	for (int i=0; i<DATA_PATH.size(); i++) {

		if (DATA_PATH[i] == '/') {

			level++;

		}

		if (level == 3) {

			SAVE_1_PATH = "../" + DATA_PATH.substr(i+1,100) + "/output/rectify/";
			SAVE_2_PATH = "../" + DATA_PATH.substr(i+1,100) + "/output/depthMap/";
			SAVE_3_PATH = "../" + DATA_PATH.substr(i+1,100) + "/output/depthFilter/";
			SAVE_4_PATH = "../" + DATA_PATH.substr(i+1,100) + "/output/cloud/";
			DATA_PATH = "../" + DATA_PATH.substr(i+1,100) + "/data/calibrationData.yml";
			break;

		}

	}

	/* Error detector */

	int error = 0;

	/* Script modes */

	std::string MODE_SELECTOR_DISPLAY_1 (argv[2]);
	bool display_1;
	std::string MODE_SELECTOR_DISPLAY_2 (argv[3]);
	bool display_2;
	std::string MODE_SELECTOR_DISPLAY_3 (argv[4]);
	bool display_3;
	std::string MODE_SELECTOR_SAVE_1 (argv[5]);
	bool save_1;
	std::string MODE_SELECTOR_SAVE_2 (argv[6]);
	bool save_2;
	std::string MODE_SELECTOR_SAVE_3 (argv[7]);
	bool save_3;
	std::string MODE_SELECTOR_SAVE_4 (argv[8]);
	bool save_4;

	if (MODE_SELECTOR_DISPLAY_1 == "true") {

		display_1 = true;

	}

	else {

		display_1 = false;

	}

	if (MODE_SELECTOR_DISPLAY_2 == "true") {

		display_2 = true;

	}

	else {

		display_2 = false;

	}

	if (MODE_SELECTOR_DISPLAY_3 == "true") {

		display_3 = true;

	}

	else {

		display_3 = false;

	}

	if (MODE_SELECTOR_SAVE_1 == "true") {

		save_1 = true;

	}

	else {

		save_1 = false;

	}

	if (MODE_SELECTOR_SAVE_2 == "true") {

		save_2 = true;

	}

	else {

		save_2 = false;

	}

	if (MODE_SELECTOR_SAVE_3 == "true") {

		save_3 = true;

	}

	else {

		save_3 = false;

	}

	if (MODE_SELECTOR_SAVE_4 == "true") {

		save_4 = true;

	}

	else {

		save_4 = false;

	}

	/* Parameters */

	std::string rate_string (argv[9]);

	float updateRate = std::stof (rate_string);

	/* Time measurement */

	clock_t init_time;
	clock_t start_time;
	clock_t end_time;
	float elapsed_time;

	/* OpenCV images */

	cv::Mat image_right;
	cv::Mat image_left;

	/* Ultrasonic sensors */

	float ultrasonic_front;
	float ultrasonic_left;
	float ultrasonic_right;

	/* Bounding Boxes */

	std::vector<short int> boundingBoxes;

	/* Mode & Status message */

	std_msgs::String mode_msg;
	std_msgs::Int8 status_msg;

	/* Listener declaration and initialization */

	Listener listener_right;
	Listener listener_left;
	Listener listener_front;

	/* Depth Map declaration and initialization */

	DepthMap depthMap;
	depthMap.setParams ();
	depthMap.loadCalibrationData (DATA_PATH);

	/* Initializing ROS node */

	ros::init (argc, argv, "environment_perception_module");
	ros::NodeHandle nh;

	/* ROS Publishers & Subscribers --> Queue size = 1 */

	image_transport::ImageTransport it (nh);

	/* ---------- INPUTS ---------- */

	image_transport::Subscriber sub_right = it.subscribe (INPUT_TOPIC_RIGHT, 10, &Listener::imageCallback, &listener_right);
	image_transport::Subscriber sub_left = it.subscribe (INPUT_TOPIC_LEFT, 10, &Listener::imageCallback, &listener_left);
	ros::Subscriber sub_filter = nh.subscribe (INPUT_FROM_FILTER, 10, &Listener::boundingBoxCallback, &listener_left);

	ros::Subscriber sub_ult_left = nh.subscribe (INPUT_TOPIC_ULT_2, 10, &Listener::rangeCallback, &listener_left);
	ros::Subscriber sub_ult_right = nh.subscribe (INPUT_TOPIC_ULT_3, 10, &Listener::rangeCallback, &listener_right);
	ros::Subscriber sub_ult_front = nh.subscribe (INPUT_TOPIC_ULT_1, 10, &Listener::rangeCallback, &listener_front);

	ros::Subscriber sub_stat = nh.subscribe (STAT_TOPIC_IN, 10, &Listener::statusCallback, &listener_right);

	/* ---------- OUTPUTS ---------- */

	image_transport::Publisher pub_image = it.advertise (OUTPUT_FOR_FILTER, 10);
	image_transport::Publisher pub_depth = it.advertise (VISUAL_DEPTH, 10);
	image_transport::Publisher pub_filter = it.advertise (VISUAL_CLOUD, 10);

	pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_cloud;
	pub_cloud.advertise (nh, OUTPUT_TOPIC, 10);
	pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_cloud_ult;
	pub_cloud_ult.advertise (nh, OUTPUT_TOPIC_ULT, 10);

	ros::Publisher pub_mode = nh.advertise <std_msgs::String> (OUTPUT_TOPIC_MODE, 10);
	ros::Publisher pub_stat = nh.advertise <std_msgs::Int8> (STAT_TOPIC_OUT, 10);

	/* TF Data --> reference to fixed frame */

	tf::TransformListener listenerTF;

	float xPosition;
	float yPosition;
	float yawAngle;

	float xAngle;
	float yAngle;
	float zAngle;
	float wAngle;

	std::vector <float> position (3);

	/* Update rate */

	ros::Rate rate (updateRate);

	init_time = clock ();

	int currentFrame = 1;

	/* Publish status --> init */

	status_msg.data = 0;
	pub_stat.publish(status_msg);

	/* Main loop */

	while (ros::ok ()) {

		start_time = clock ();

		/* ------------------------------------------------------- */

		/* Get data from the vehicle */

		image_right = listener_right.getImage();
		image_left = listener_left.getImage();

		ultrasonic_front = listener_front.getRange();
		ultrasonic_left = listener_left.getRange();
		ultrasonic_right = listener_right.getRange();

		try {

			error = 0;

			/* Image Preprocessing */

			depthMap.preProcessing (image_right, image_left, display_1, save_1, SAVE_1_PATH, init_time);

			/* Send information to the AI Filter */

			pub_image.publish (depthMap.getMsgForFilter());

			/* Wait until the AI filter is ready */

			if (listener_right.getStatusAI() < 2) {

				error = 1;
				currentFrame--;

			}

			/* Compute Depth Map */

			depthMap.compute (display_2, save_2, SAVE_2_PATH, init_time);
			
			pub_depth.publish (depthMap.getMsgForVisual1());

			/* Receive information from the AI filter */

			boundingBoxes = listener_left.getBoundingBoxes();

			/* AI - Filtering */

			depthMap.postFiltering (boundingBoxes, display_3, save_3, SAVE_3_PATH, init_time);
			
			pub_filter.publish(depthMap.getMsgForVisual2());

			/* Obtain latest TF data available */

			tf::StampedTransform transformTF;

			listenerTF.lookupTransform ("/base_link", "/odom", ros::Time(0), transformTF);

			/* Convert from quaternions to euler angles --> yaw */

			xAngle = transformTF.getRotation().getX();
			yAngle = transformTF.getRotation().getY();
			zAngle = transformTF.getRotation().getZ();
			wAngle = transformTF.getRotation().getW();

			yawAngle = std::atan2 ((2*(wAngle*zAngle+xAngle*yAngle)),(1-2*(yAngle*yAngle+zAngle*zAngle)));

			/* Angle criteria --> start from 0 rad & clockwise positive */

			if (yawAngle > 0) {

				yawAngle = yawAngle - PI;

			}

			else {

				yawAngle = yawAngle + PI;

			}

			/* Rotate X and Y using the yaw angle */

			xPosition = transformTF.getOrigin().getX()*cos(-yawAngle) - transformTF.getOrigin().getY()*sin(-yawAngle);
			yPosition = transformTF.getOrigin().getX()*sin(-yawAngle) + transformTF.getOrigin().getY()*cos(-yawAngle);

			position[0] = xPosition;
			position[1] = yPosition;
			position[2] = yawAngle;

			/* Convert to Point Cloud */

			depthMap.toPointCloud (save_4, SAVE_4_PATH, init_time, position);

			/* Compute ultrasonic Point Cloud */

			mode_msg.data = depthMap.computeUltPointCloud (position, ultrasonic_front, ultrasonic_right, ultrasonic_left);

			/* Publish Point Clouds */

			pub_cloud.publish (depthMap.getOutputCloud());
			pub_cloud_ult.publish (depthMap.getOutputCloudUlt());

			/* Publich mode info */

			pub_mode.publish (mode_msg);

			/* Stop the node */

			if (currentFrame >= FINAL_FRAME && listener_right.getStatusAI() == 2) {

				std::cout << "[Depth-Vision] Final Frame Processed >> Killing node" << std::endl;

				/* Publish status --> Work finished */

				status_msg.data = 1;
				pub_stat.publish(status_msg);

				break;

			}

			else {

				currentFrame++;

			}

		}

		catch (cv::Exception& e) {

			error = 1;

		}

		catch (pcl::IOException e) {

			error = 1;

		}

		/* ------------------------------------------------------- */

		end_time = clock ();

		elapsed_time = (((float) (end_time - start_time))/CLOCKS_PER_SEC);

		if (error == 0) {
			std::cout << "[Depth-Vision] Frame Processed >> Time = " << std::setprecision (3) << elapsed_time << " segs" << std::endl;
		}

		/* Wait for a new pair of images */

		ros::spinOnce ();
		rate.sleep ();

	}

	/* Exit main function */

	return 0;

}

/* ----------------------------------------------------------------------------
   ---------------- Environment Perception Module (EPM) - ARAV ----------------
   ----------------------------------------------------------------------------
   -------------------- Author : Alberto Ceballos Gonzalez --------------------
   -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
   --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved --------
   ---------------------------------------------------------------------------- */
