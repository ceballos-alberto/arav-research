
/* ---------------------------------------------------------------------------- 
   --------------------- Stereo Camera Calibration - ARAV ---------------------
   ---------------------------------------------------------------------------- 
   ------------------- Author --> Alberto Ceballos Gonzalez ------------------- 
   ------- E-mail --> alberto.ceballos-gonzalez@student.isae-supaero.fr ------- 
   --------- (c) Copyright 2021. Alberto Ceballos. All Rights Reserved --------  
   ---------------------------------------------------------------------------- */

/* Import required libraries */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

/* Namespaces */

using namespace cv;
using namespace std;
using namespace cv_bridge;
using namespace ros;
using namespace image_transport;
using namespace sensor_msgs;

/* Topic names - (Global variables) */

static const std::string INPUT_TOPIC_NAME_RIGHT = "/ARAV/sensors/frontRightCamera/image";
static const std::string INPUT_TOPIC_NAME_LEFT = "/ARAV/sensors/frontLeftCamera/image";

/* Listener Class definition */

class Listener {

	private:	/* Class Attributes - Received Image */
		
		cv::Mat image;
		
	public:	/* Class Methods */
		
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
		
		/* Getters for class attributes */
		
		cv::Mat getImage () {
		
			return image;
		
		}

};

/* Calibrator Class definition */

class Calibrator {

	private:	/* Class Attributes */
							
		int boardRows = 8;		/* This line can be modified */			
		int boardColumns = 6;		/* This line can be modified */
		cv::Size boardSize;
		bool success = false;
		
		std::vector <cv::Point3f> chessBoard;
		std::vector <std::vector<cv::Point2f>> imagePointsRight;
		std::vector <std::vector<cv::Point2f>> imagePointsLeft;
		std::vector <std::vector<cv::Point3f>> chessBoardList;
		
		cv::Mat image_right;
		cv::Mat image_left;
		
		/* Calibration parameters --> matrices */
		
		cv::Mat CM1, CM2, D1, D2, R, T, E, F, R1, R2, P1, P2, Q; 
    		
	public:	/* Class Methods */
		
		void loadImages (cv::Mat imgR, cv::Mat imgL) {
		
			cv::cvtColor(imgR, image_right, CV_BGR2GRAY);
        		cv::cvtColor(imgL, image_left, CV_BGR2GRAY);
		
		}
		
		void computeNbPoints () {
		
			/* Parameters */
			
			int nbPoints = 0;
		
			nbPoints = boardRows*boardColumns;
			boardSize = cv::Size(boardColumns, boardRows);
			
			std::cout << "\n[Camera calibration] Number of rows --> " << boardRows << "\n";
			std::cout << "[Camera calibration] Number of columns --> " << boardColumns << "\n";
			std::cout << "[Camera calibration] Number of points --> " << nbPoints << "\n";
		
		}
		
		void computeChessBoard () {
		
			/* Parameters */
			
			float squareSize = 0.03f;			/* This line can be modified --> m */
		
			for (int i = 0; i < boardRows; i++) {
			
        			for (int j = 0; j < boardColumns; j++) {
        			
            				chessBoard.push_back (cv::Point3f(i*squareSize, j*squareSize, 0));
            				
        			}
        			
    			}
			
		}
		
		void cornerDetector (int i, bool display, bool save, std::string SAVE_PATH) {
		
			/* Parameters */
			
			std::vector <cv::Point2f> cornersRight;
			std::vector <cv::Point2f> cornersLeft;
		
			bool success_right = false;
			bool success_left = false;
		
			success_right = cv::findChessboardCorners (image_right, boardSize, cornersRight, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
			if (success_right) {
			
				cv::cornerSubPix (image_right, cornersRight, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria (CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				cv::drawChessboardCorners (image_right, boardSize, cornersRight, true);
			
			}
			
			success_left = cv::findChessboardCorners (image_left, boardSize, cornersLeft, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
			if (success_left) {
			
				cv::cornerSubPix (image_left, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria (CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
				cv::drawChessboardCorners (image_left, boardSize, cornersLeft, true);
			
			}
			
			if (success_right && success_left) {
			
				success = true;
				
				if (display) {
			
					cv::imshow ("Calibration - Right Camera", image_right);
					cv::imshow ("Calibration - Left Camera", image_left);
					cv::waitKey (15);
					
				}
				
				if (save) {
				
					cv::imwrite (SAVE_PATH + "imageRight_" + std::to_string(i) + ".png", image_right);
					cv::imwrite (SAVE_PATH + "imageLeft_" + std::to_string(i) + ".png", image_left);
				
				}
			
				imagePointsRight.push_back(cornersRight);
				imagePointsLeft.push_back(cornersLeft);
				chessBoardList.push_back(chessBoard);
			
			}
			
			else {
			
				success = false;
			
			}
			
		}
		
		void calibrate () {
		
			CM1 = cv::Mat(3, 3, CV_64FC1);
    			CM2 = cv::Mat(3, 3, CV_64FC1);
    			
    			cv::stereoCalibrate (
    			
    				chessBoardList,
    				imagePointsRight, 
    				imagePointsLeft, 
    				CM1, D1, CM2, D2, 
    				image_right.size(), 
    				R, T, E, F, 
    				cv::CALIB_SAME_FOCAL_LENGTH | cv::CALIB_ZERO_TANGENT_DIST, 
    				cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
    				
    			);
    			
    			cv::stereoRectify (CM1, D1, CM2, D2, image_right.size(), R, T, R1, R2, P1, P2, Q);
		
		}
		
		void saveData (std::string DATA_PATH) {
		
			cv::FileStorage file (DATA_PATH, cv::FileStorage::WRITE);
			
	    			file << "CM1" << CM1;
			    	file << "CM2" << CM2;
			    	file << "D1" << D1;
			    	file << "D2" << D2;
			    	file << "R" << R;
			    	file << "T" << T;
			    	file << "E" << E;
			    	file << "F" << F;
				file << "R1" << R1;
	    			file << "R2" << R2;
	    			file << "P1" << P1;
	    			file << "P2" << P2;
	    			file << "Q" << Q;
    			
    			file.release ();
    			
		}
		
		void testCalibrator (cv::Mat imgR, cv::Mat imgL, bool display) {
		
			/* Parameters */
		
			cv::Mat mapRightx, mapRighty, mapLeftx, mapLefty, imgRightrect, imgLeftrect, imgRectify;
			
			cv::initUndistortRectifyMap(CM1, D1, R1, P1, image_right.size(), CV_32FC1, mapRightx, mapRighty);
    			cv::initUndistortRectifyMap(CM2, D2, R2, P2, image_left.size(), CV_32FC1, mapLeftx, mapLefty);
    			
    			cv::cvtColor(imgR, imgR, cv::COLOR_BGR2RGB);
        		cv::cvtColor(imgL, imgL, cv::COLOR_BGR2RGB);
        		
        		cv::remap(imgR, imgRightrect, mapRightx, mapRighty, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        		cv::remap(imgL, imgLeftrect, mapLeftx, mapLefty, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
        		
    			imgRectify = cv::Mat::zeros (imgRightrect.rows, imgRightrect.cols*2+10, imgRightrect.type());
    			imgRightrect.copyTo(imgRectify(cv::Range::all(), cv::Range(0, imgLeftrect.cols)));
        		imgLeftrect.copyTo(imgRectify(cv::Range::all(), cv::Range(imgLeftrect.cols+10, imgLeftrect.cols*2+10)));
    			
    			for(int j = 0; j < imgRectify.rows; j += 16) {
    			
            			cv::Point p1 = cv::Point(0,j);
            			cv::Point p2 = cv::Point(imgRectify.cols*2,j);
            			cv::line (imgRectify, p1, p2, CV_RGB(255,0,0));
            			
        		}
        		
        		if (display) {
        		
				imshow("Rectified image", imgRectify);
				cv::waitKey (15);
				
			}
        		
		}
		
		/* Getters for class attributes */
		
		bool getSuccess () {
		
			return success;
		
		}
		
};

/* Main function */

int main (int argc, char** argv) {

	/* Paths */
	
	std::string DATA_PATH (argv[1]);
	std::string SAVE_PATH;
	
	int level = 0;
	
	for (int i=0; i<DATA_PATH.size(); i++) {
	
		if (DATA_PATH[i] == '/') {
		
			level++;
		
		}
		
		if (level == 3) {
	
			SAVE_PATH = "../" + DATA_PATH.substr(i+1,100) + "/output/";
			DATA_PATH = "../" + DATA_PATH.substr(i+1,100) + "/data/calibrationData.yml";
			break;
			
		}
	
	}
	
	/* Script modes */
	
	std::string MODE_SELECTOR_DISPLAY (argv[2]);
	bool display;
	
	std::string MODE_SELECTOR_SAVE (argv[3]);
	bool save; 
	
	if (MODE_SELECTOR_DISPLAY == "true") {
	
		display = true;
	
	}
	
	if (MODE_SELECTOR_SAVE == "true") {
	
		save = true;
	
	}

	/* Parameter declaration */
	
	std::string rate_string (argv[4]);
	
	float updateRate = std::stof (rate_string);
	
	int nbimages = 15;		/* This line can be modified (15/20 is a good value) */
	
	/* Iterators */
	
	int i = 0;
	
	/* Images from the cameras */
	
	cv::Mat image_right;
	cv::Mat image_left;
	
	/* Calibrator declaration and initialization */
	
	Calibrator calibrator;
	calibrator.computeNbPoints();
	calibrator.computeChessBoard();
	
	/* Listener declaration */
	
	Listener listener_right;
	Listener listener_left;

	/* Initializing ROS node */

	ros::init (argc, argv, "camera_calibration");
	ros::NodeHandle nh;
	
	/* ROS Publishers & Subscribers */
	
	image_transport::ImageTransport it (nh);
	image_transport::Subscriber sub_right = it.subscribe (INPUT_TOPIC_NAME_RIGHT, 1, &Listener::imageCallback, &listener_right);
	image_transport::Subscriber sub_left = it.subscribe (INPUT_TOPIC_NAME_LEFT, 1, &Listener::imageCallback, &listener_left);
	
	ros::Rate rate (updateRate);
	
	char start;
	
	std::cout << "[Camera calibration] Press s + enter to start calibration >> ";
	
	while (true) {
	
		std::cin >> start;
		
		if (start == 's') {
		
			break;
		
		}
		
		else { 
		
		std::cout << "\n[Camera calibration] Invalid character. Try again.\n"; 
		
		}
	
	}
	
	std::cout << "\n[Camera calibration] Starting process ...\n\n";
	
	/* Main loop */
	
	while (ros::ok () && i<=nbimages) {
	
		if (i>0) {
		
		/* Repeat The calibration several times */
		
			/* Get images */
		
			image_right = listener_right.getImage();
			image_left = listener_left.getImage();
			
			calibrator.loadImages (image_right, image_left);
			calibrator.cornerDetector (i, display, save, SAVE_PATH);
			
			if (calibrator.getSuccess()) {
			
				std::cout << "[Camera calibration] The frame [nb " << i << "] has been correctly processed\n";
			
			}
			
			else {
			
				i--;
				std::cout << "[Camera calibration] The frame has not been correctly processed\n";
			
			}
			
		}
		
		/* Wait for new images */
		
		i++;
		ros::spinOnce ();
		rate.sleep ();
		
	}
	
	calibrator.calibrate ();
	calibrator.saveData (DATA_PATH);
	cv::destroyAllWindows();
	
	std::cout << "\n[Camera calibration] Successful calibration !!\n";
	
	/* Show results */
	
	while (ros::ok ()) {
	
		image_right = listener_right.getImage();
		image_left = listener_left.getImage();
	
		calibrator.testCalibrator (image_right, image_left, display);
		ros::spinOnce ();
		rate.sleep ();
	
	}

	/* Exit main function */	
	
	return 0;
}

/* ---------------------------------------------------------------------------- 
   --------------------- Stereo Camera Calibration - ARAV ---------------------
   ---------------------------------------------------------------------------- 
   ------------------- Author --> Alberto Ceballos Gonzalez ------------------- 
   ------- E-mail --> alberto.ceballos-gonzalez@student.isae-supaero.fr ------- 
   --------- (c) Copyright 2021. Alberto Ceballos. All Rights Reserved --------  
   ---------------------------------------------------------------------------- */
   
