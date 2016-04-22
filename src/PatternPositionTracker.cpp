/*
 * PatternPositionTracker.cpp
 *
 *  Created on: 05/ott/2014
 *      Author: Sab
 */

#include "PatternPositionTracker.h"

/**
 *
 */
PatternPositionTracker::PatternPositionTracker(const string &filename) {
	if (!fileExists(filename.c_str())) {
		stringstream ss;
		ss << "ERROR: The settings file  " << filename << " does not exist!";
		throw std::runtime_error(ss.str());
	}

	setUpParameters(filename);

	WLOG("Started");

	m_CoordinatesFile.open(m_CoordinatesFileName.c_str());

	//cbSize = Size(boardHeight, boardWidth);
	//Zconst = 5 * DIM_MM_QUAD; //distanza costante del punto significativo dalla camera

	//set up matrices for storage
	rvec = Mat(Size(3, 1), CV_64F);
	tvec = Mat(Size(3, 1), CV_64F);
	setUpCameraModel(m_CalibratorFileName, cameraMatrix, distortionCoeff);

	//generate vectors for the points on the chessboard
	//boardPoints3D are measured in millimeters because calibration is done in mm also

	//initChessboard3DPoints(boardPoints3D, boardWidth, boardHeight);
	initPattern3DPoints(boardPoints3D, m_pattern_WidthReal,
			m_pattern_HeightReal);

	// generate points in the reference frame (gliassi cartesiani)

	//init3DPointsAxes(framePoints, 5 * DIM_MM_QUAD);//dim chessboard
	init3DPointsAxes(framePoints, Zconst);

	//punto fisso del quale voglio calcolare le coordinate 3D
	pFixedCalculated = false;

	img_pattern = imread(m_PatternFileName, CV_LOAD_IMAGE_GRAYSCALE);
}

PatternPositionTracker::PatternPositionTracker(const string &filename,
		const cv::Mat &webcamImage) {
	if (!fileExists(filename.c_str())) {
		stringstream ss;
		ss << "ERROR: The settings file  " << filename << " does not exist!";
		throw std::runtime_error(ss.str());
	}

	setUpParameters(filename);

	WLOG("Started");

	m_CoordinatesFile.open(m_CoordinatesFileName.c_str());

	//cbSize = Size(boardHeight, boardWidth);
	//Zconst = 5 * DIM_MM_QUAD; //distanza costante del punto significativo dalla camera

	//set up matrices for storage
	rvec = Mat(Size(3, 1), CV_64F);
	tvec = Mat(Size(3, 1), CV_64F);
	setUpCameraModel(m_CalibratorFileName, cameraMatrix, distortionCoeff);

	//init3DPointsAxes(framePoints, 5 * DIM_MM_QUAD);//dim chessboard
	init3DPointsAxes(framePoints, Zconst);

	//punto fisso del quale voglio calcolare le coordinate 3D
	pFixedCalculated = false;

	if (!pFixedCalculated) {
		//punto fisso del quale voglio calcolare le coordinate 3D
		pFixed.x = webcamImage.cols / 2;
		pFixed.y = webcamImage.rows / 2;
		uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
		uvPoint.at<double>(0, 0) = pFixed.x;
		uvPoint.at<double>(1, 0) = pFixed.y;
		pFixedCalculated = true;
	}

	prec_marker_quadrante = -1;
	prec_markerCalculated = false;

	// non mappo ancora l'origine 2D
	boardPoints2D.clear();

	//mappo l'origine 3D
	boardPoints3D.clear();
	initPattern3DPoints(boardPoints3D, m_pattern_WidthReal,
			m_pattern_HeightReal);

}

/**
 *
 */
PatternPositionTracker::~PatternPositionTracker() {
	if (m_enableLog) {
		m_logFile.close();
	}
	m_CoordinatesFile.close();
}

/**
 *
 */
void PatternPositionTracker::WLOG(const string& msg) {
	if (m_enableLog) {
		m_logFile << "[" << getNowTime() << "]PatternPositionTracker::" << msg
				<< endl;
	}
}

/**
 * No log!
 */
void PatternPositionTracker::setUpParameters(const string &filename) {
	//set up a FileStorage object to read camera params from file
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		stringstream ss;
		ss << "ERROR: Could not open the settings file: \"" << filename;
		throw std::runtime_error(ss.str());
	}

	fs["CalibratorOutFileName"] >> m_CalibratorFileName;
	fs["PatternFileName"] >> m_PatternFileName;
	//fs["pattern_WidthPixel"] >> m_pattern_WidthPixel;
	//fs["pattern_HeightPixel"] >> m_pattern_HeightPixel;
	fs["pattern_WidthReal"] >> m_pattern_WidthReal;
	fs["pattern_HeightReal"] >> m_pattern_HeightReal;
	fs["zConst"] >> Zconst;
	fs["EnableLogFile"] >> m_enableLog;
	fs["LogFileName"] >> m_LogFileName;
	fs["CoordinatesOutFileName"] >> m_CoordinatesFileName;
	fs.release();

	// controlo l'esistenza dei file di input ...
	if (!fileExists(m_CalibratorFileName.c_str())) {
		stringstream ss;
		ss << "ERROR: The calibrator file  " << m_CalibratorFileName
				<< " not exist!";
		throw std::runtime_error(ss.str());
	}

	if (!fileExists(m_PatternFileName.c_str())) {
		stringstream ss;
		ss << "ERROR: The pattern file " << m_PatternFileName << " not exist!";
		throw std::runtime_error(ss.str());
	}

	if (m_enableLog) {
		m_logFile.open(m_LogFileName.c_str());
	}

}

/**
 *
 */
void PatternPositionTracker::WLOG(const stringstream& msg) {
	WLOG(msg.str());
}

/**
 *
 */
void PatternPositionTracker::setUpCameraModel(const string &filename,
		cv::Mat& cameraMatrix, cv::Mat& distortionCoeff) {
	WLOG("setUpCameraModel");

	//set up a FileStorage object to read camera params from file
	FileStorage fs;
	fs.open(filename, FileStorage::READ);
	if (!fs.isOpened()) {
		stringstream ss;
		ss << "ERROR: Could not open the camera model file: \"" << filename;
		WLOG(ss);
		throw std::runtime_error(ss.str());
	}

	// read camera matrix and distortionCoeff coefficients from file
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distortionCoeff;
	// close the input file
	fs.release();
}

/**
 * Inizializza i punti 3D di cui conosco realmente le coordinate
 * ambientali rispetto all'origine (le dimensioni sono espresse in
 * millimetri)
 */
void PatternPositionTracker::initChessboard3DPoints(
		vector<Point3d> &boardPoints3D, const int boardWidth,
		const int boardHeight) {
	WLOG("initChessboard3DPoints");

	int count = 0;
	for (unsigned int i = 0; i < boardWidth; i++) {
		for (unsigned int j = 0; j < boardHeight; j++) {
			boardPoints3D.push_back(
					Point3d(double(i) * DIM_MM_QUAD, double(j) * DIM_MM_QUAD,
							0.0));
			count++;
			cout << "P[" << count << "]=" << double(i) * DIM_MM_QUAD << ","
					<< double(j) * DIM_MM_QUAD << endl;
		}
		cout << endl << endl;
	}

	/*
	 boardPoints3D.push_back(Point3d(0.0, 0.0, 0.0));
	 cout << "P[" << 0 << "]=" << 0 << "," << 0 << endl;

	 boardPoints3D.push_back(Point3d(0, (boardHeight - 1) * DIM_MM_QUAD, 0.0));
	 cout << "P[" << 1 << "]=" << 0 << "," << (boardHeight - 1) * DIM_MM_QUAD
	 << endl;

	 boardPoints3D.push_back(
	 Point3d((boardWidth - 1) * DIM_MM_QUAD,
	 (boardHeight - 1) * DIM_MM_QUAD, 0.0));
	 cout << "P[" << 2 << "]=" << (boardWidth - 1) * DIM_MM_QUAD << ","
	 << (boardHeight - 1) * DIM_MM_QUAD << endl;

	 boardPoints3D.push_back(
	 Point3d((boardHeight - 1) * DIM_MM_QUAD,
	 (boardWidth - 1) * DIM_MM_QUAD, 0.0));
	 cout << "P[" << 3 << "]=" << (boardHeight - 1) * DIM_MM_QUAD << ","
	 << (boardWidth - 1) * DIM_MM_QUAD << endl;
	 */
}

void PatternPositionTracker::initPattern3DPoints(vector<Point3d> &boardPoints3D,
		const int sizeWidth, const int sizeHeight) {
	WLOG("initPattern3DPoints");

	boardPoints3D.push_back(Point3d(0.0, 0.0, Zconst));
	//boardPoints3D.push_back(Point3d(0, sizeWidth, 0.0));
	boardPoints3D.push_back(Point3d(0, sizeHeight, Zconst));
	boardPoints3D.push_back(Point3d(sizeWidth, sizeHeight, Zconst));
	//boardPoints3D.push_back(Point3d(sizeHeight, sizeWidth, 0.0));
	boardPoints3D.push_back(Point3d(sizeWidth, 0, Zconst));
	/*
	 int dimSezWidth = sizeWidth/boardWidth;
	 int dimSezHeight = sizeHeight/boardHeight;

	 for (unsigned int i = 0; i < boardWidth; i++) {
	 for (unsigned int j = 0; j < boardHeight; j++) {
	 boardPoints3D.push_back(
	 Point3d(double(i) * dimSezWidth, double(j) * dimSezHeight,
	 0.0));
	 }
	 }
	 */
}

/**
 *
 */
void PatternPositionTracker::init3DPointsAxes(vector<Point3d> &framePoints,
		const int size) {
	WLOG("init3DPointsAxes");

	framePoints.push_back(Point3d(0.0, 0.0, 0.0));
	framePoints.push_back(Point3d(size, 0.0, 0.0));
	framePoints.push_back(Point3d(0.0, size, 0.0));
	framePoints.push_back(Point3d(0.0, 0.0, size));
}

/**
 *
 */
void PatternPositionTracker::drawSceneReference(cv::Mat &webcamImage,
		const Point2d& pFixed, const vector<Point2d>& boardPoints2D,
		const vector<Point2d>& imageFramePoints) {
	WLOG("drawSceneReference");

	// disegno l'origine ...
	//cv::circle(webcamImage, imageFramePoints[0], 8, CV_RGB(255, 50, 255));

	//... e gli angoli sul pattern
	cv::circle(webcamImage, boardPoints2D[0], 1, CV_RGB(255, 0, 0), 2);
	cv::circle(webcamImage, boardPoints2D[1], 1, CV_RGB(255, 0, 0), 2);
	cv::circle(webcamImage, boardPoints2D[2], 1, CV_RGB(255, 0, 0), 2);
	cv::circle(webcamImage, boardPoints2D[3], 1, CV_RGB(255, 0, 0), 2);

	//il contorno del pattern
	line(webcamImage, boardPoints2D[0], boardPoints2D[1], CV_RGB(0, 255, 0), 2);
	line(webcamImage, boardPoints2D[1], boardPoints2D[2], CV_RGB(0, 255, 0), 2);
	line(webcamImage, boardPoints2D[2], boardPoints2D[3], CV_RGB(0, 255, 0), 2);
	line(webcamImage, boardPoints2D[3], boardPoints2D[0], CV_RGB(0, 255, 0), 2);

	//disegno gli assi cartesiani
	//line(webcamImage, imageFramePoints[0], imageFramePoints[1],
	//		CV_RGB(0, 0, 255), 2);
	//putText(webcamImage, "x", imageFramePoints[1], FONT_HERSHEY_SIMPLEX, 1,
	//		Scalar::all(255), 1, 8);

	//line(webcamImage, imageFramePoints[0], imageFramePoints[2],
	//		CV_RGB(0, 0, 255), 2);
	//putText(webcamImage, "y", imageFramePoints[2], FONT_HERSHEY_SIMPLEX, 1,
	//		Scalar::all(255), 1, 8);

	//line(webcamImage, imageFramePoints[0], imageFramePoints[3],
	//		CV_RGB(0, 0, 255), 2);
	//putText(webcamImage, "z", imageFramePoints[3], FONT_HERSHEY_SIMPLEX, 1,
	//		Scalar::all(255), 1, 8);

	// ... e quelli (2D) sulla chessboard
	//for (unsigned int i = 0; i < boardPoints2D.size(); ++i) {
	//	circle(webcamImage, boardPoints2D[i], 3, CV_RGB(0, 255, 0));
	//}

	// disegno la linea dall'origine al punto fisso sullo schermo
	//line(webcamImage, imageFramePoints[0], pFixed, CV_RGB(255, 0, 255), 2);
	line(webcamImage, boardPoints2D[0], pFixed, CV_RGB(0, 0, 255), 2);
}

/**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) {
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1 * dx2 + dy1 * dy2)
			/ sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

/**
 *
 */
bool PatternPositionTracker::isQuadrilateral(
		const vector<Point2d>& boardPoints2D) {
	//WLOG("isQuadrilateral");

	bool res = false;
	try {

		int vtc = boardPoints2D.size();
		// Get the cosines of all corners
		std::vector<double> cos;
		cos.push_back(
				angle(boardPoints2D[1], boardPoints2D[3], boardPoints2D[0]));
		cos.push_back(
				angle(boardPoints2D[0], boardPoints2D[2], boardPoints2D[1]));
		cos.push_back(
				angle(boardPoints2D[0], boardPoints2D[2], boardPoints2D[3]));
		cos.push_back(
				angle(boardPoints2D[3], boardPoints2D[1], boardPoints2D[2]));

		// Sort ascending the cosine values
		std::sort(cos.begin(), cos.end());
		// Get the lowest and the highest cosine
		double mincos = cos.front();
		double maxcos = cos.back();
		// Use the degrees obtained above and the number of vertices
		// to determine the shape of the contour
		if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
			//cerr << " IS QUADRILATERAL!" << endl;
			res = true;
		}

	} catch (cv::Exception &cvError) {
		res = false;
	}

	return res;
}

/**
 *
 */
bool PatternPositionTracker::findPatternBorders(const cv::Mat & img_object,
		const cv::Mat & img_cam, vector<Point2d>& boardPoints2D) {
	WLOG("findPatternBorders");

	bool res = false;

	if (!img_object.data || !img_cam.data) {
		m_logFile << " --(!) Error reading images " << std::endl;
		return res;
	}

	//-- Step 1: Detect the keypoints using SURF Detector
	WLOG("findPatternBorders:: Detect the keypoints using SURF Detector");

	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> keypoints_object, keypoints_cam;

	detector.detect(img_object, keypoints_object);
	detector.detect(img_cam, keypoints_cam);

	//-- Step 2: Calculate descriptors (feature vectors)
	WLOG("findPatternBorders:: Calculate descriptors (feature vectors)");
	SurfDescriptorExtractor extractor;

	Mat descriptors_object, descriptors_cam;

	extractor.compute(img_object, keypoints_object, descriptors_object);
	extractor.compute(img_cam, keypoints_cam, descriptors_cam);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	WLOG(
			"findPatternBorders:: Matching descriptor vectors using FLANN matcher");
	FlannBasedMatcher matcher;
	//BFMatcher matcher(NORM_HAMMING);

	std::vector<DMatch> matches;
	matcher.match(descriptors_object, descriptors_cam, matches);

	double max_dist = 0;
	double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	WLOG(
			"findPatternBorders:: Quick calculation of max and min distances between keypoints");
	for (int i = 0; i < descriptors_object.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	WLOG("findPatternBorders:: Draw only good matches");
	std::vector<cv::DMatch> good_matches;
	for (int i = 0; i < descriptors_object.rows; i++) {
		//if (matches[i].distance < 3 * min_dist) {
		if (matches[i].distance < 2 * min_dist) {
			good_matches.push_back(matches[i]);
		}
	}
	//cout << "good_matches.size() = " << good_matches.size() << endl;

	if (good_matches.size() > 1) {
		Mat img_matches;
		cv::drawMatches(img_object, keypoints_object, img_cam, keypoints_cam,
				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//-- Localize the object
		WLOG("findPatternBorders:: Localize the object");
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (int i = 0; i < good_matches.size(); i++) {
			//-- Get the keypoints from the good matches
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_cam[good_matches[i].trainIdx].pt);
		}

		Mat H = findHomography(obj, scene, CV_RANSAC);

		//-- Get the corners from the image_1 ( the object to be "detected" )
		WLOG("findPatternBorders:: Get the corners from the image_1 ");

		// in senso antiorario....
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(0, img_object.rows);
		obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
		obj_corners[3] = cvPoint(img_object.cols, 0);
		std::vector<Point2f> scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);

		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		//cv::Mat img_obj_detected = Mat::zeros(img_matches.size(),
		//		img_matches.type());
		//line(img_obj_detected, scene_corners[0] + Point2f(img_object.cols, 0),
		//		scene_corners[1] + Point2f(img_object.cols, 0),
		//		Scalar(0, 255, 0), 4);
		//line(img_obj_detected, scene_corners[1] + Point2f(img_object.cols, 0),
		//		scene_corners[2] + Point2f(img_object.cols, 0),
		//		Scalar(0, 255, 0), 4);
		//line(img_obj_detected, scene_corners[2] + Point2f(img_object.cols, 0),
		//		scene_corners[3] + Point2f(img_object.cols, 0),
		//		Scalar(0, 255, 0), 4);
		//line(img_obj_detected, scene_corners[3] + Point2f(img_object.cols, 0),
		//		scene_corners[0] + Point2f(img_object.cols, 0),
		//		Scalar(0, 255, 0), 4);

		//-- Show detected matches
		WLOG("findPatternBorders:: Show detected matches");

		//disegno gli estremi
		boardPoints2D.clear();
		//cv::Point2f Point1(scene_corners[0] + Point2f(img_object.cols, 0));
		//cv::circle(img_obj_detected, Point1, 1, CV_RGB(255, 0, 0), 2);
		boardPoints2D.push_back(scene_corners[0]);

		//cv::Point2f Point2(scene_corners[1] + Point2f(img_object.cols, 0));
		//cv::circle(img_obj_detected, Point2, 1, CV_RGB(255, 0, 0), 2);
		boardPoints2D.push_back(scene_corners[1]);

		//cv::Point2f Point3(scene_corners[2] + Point2f(img_object.cols, 0));
		//cv::circle(img_obj_detected, Point3, 1, CV_RGB(255, 0, 0), 2);
		boardPoints2D.push_back(scene_corners[2]);

		//cv::Point2f Point4(scene_corners[3] + Point2f(img_object.cols, 0));
		//cv::circle(img_obj_detected, Point4, 1, CV_RGB(255, 0, 0), 2);
		boardPoints2D.push_back(scene_corners[3]);

		//cv::Point m_CentralPoint;
		//m_CentralPoint.x = (Point1.x + Point3.x) / 2;
		//m_CentralPoint.y = (Point1.y + Point3.y) / 2;
		//cv::circle(img_obj_detected, m_CentralPoint, 8, CV_RGB(0, 0, 255), 2);
		//imshow("img_obj_detected", img_obj_detected);

		res = isQuadrilateral(boardPoints2D);
	}
	return res;
}

bool PatternPositionTracker::calculateHomography(const cv::Mat & img_object,
		const cv::Mat & img_cam, cv::Mat &H) {
	cerr << "calculateHomography" << endl;

	//imshow("img_object", img_object);
	//imshow("img_cam", img_cam);
	//waitKey(1);

	bool res = false;

	if (!img_object.data || !img_cam.data) {
		m_logFile << " --(!) Error reading images " << std::endl;
		return res;
	}

	//-- Step 1: Detect the keypoints using SURF Detector
	WLOG("findPatternBorders:: Detect the keypoints using SURF Detector");

	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);

	std::vector<KeyPoint> keypoints_object, keypoints_cam;

	detector.detect(img_object, keypoints_object);
	detector.detect(img_cam, keypoints_cam);

	//-- Step 2: Calculate descriptors (feature vectors)
	WLOG("findPatternBorders:: Calculate descriptors (feature vectors)");
	SurfDescriptorExtractor extractor;

	Mat descriptors_object, descriptors_cam;

	extractor.compute(img_object, keypoints_object, descriptors_object);
	extractor.compute(img_cam, keypoints_cam, descriptors_cam);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	WLOG(
			"findPatternBorders:: Matching descriptor vectors using FLANN matcher");
	FlannBasedMatcher matcher;
	//BFMatcher matcher(NORM_HAMMING);

	std::vector<DMatch> matches;
	matcher.match(descriptors_object, descriptors_cam, matches);

	double max_dist = 0;
	double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	WLOG(
			"findPatternBorders:: Quick calculation of max and min distances between keypoints");
	for (int i = 0; i < descriptors_object.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	WLOG("findPatternBorders:: Draw only good matches");
	std::vector<cv::DMatch> good_matches;
	for (int i = 0; i < descriptors_object.rows; i++) {
		//if (matches[i].distance < 3 * min_dist) {
		if (matches[i].distance < 2 * min_dist) {
			good_matches.push_back(matches[i]);
		}
	}
	//cout << "good_matches.size() = " << good_matches.size() << endl;

	if (good_matches.size() > 1) {
		Mat img_matches;
		cv::drawMatches(img_object, keypoints_object, img_cam, keypoints_cam,
				good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//-- Localize the object
		WLOG("findPatternBorders:: Localize the object");
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (int i = 0; i < good_matches.size(); i++) {
			//-- Get the keypoints from the good matches
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_cam[good_matches[i].trainIdx].pt);
		}

		H = findHomography(obj, scene, CV_RANSAC);
		res = true;
	}
	cerr << "calculateHomography end" << endl;

	return res;
}

/**
 *
 */
bool PatternPositionTracker::mapKeyPoints(const cv::Mat & img_cam,
		vector<Point2d>& boardPoints2D, vector<Point3d> &boardPoints3D,
		std::vector<KeyPoint> &keypoints_cam) {
	WLOG("findKeyPoints");

	bool res = false;

	//-- Step 1: Detect the keypoints using SURF Detector
	WLOG("findKeyPoints:: Detect the keypoints using SURF Detector");

	int minHessian = 400;
	SurfFeatureDetector detector(minHessian);
	std::vector<KeyPoint> keypoints_cam_tmp;
	detector.detect(img_cam, keypoints_cam_tmp);

	for (int i = 0; i < 4/*keypoints_cam.size()*/; ++i) {
		keypoints_cam.push_back(keypoints_cam_tmp[i]);
	}

	//mappo i punti significati sulle coordinate della camera
	boardPoints2D.clear();
	boardPoints3D.clear();

	int dim_x_2D = img_cam.cols;
	int dim_y_2D = img_cam.rows;
	for (int i = 0; i < keypoints_cam.size(); ++i) {

		// --> 2D
		int X_2d = keypoints_cam[i].pt.x;
		int Y_2d = keypoints_cam[i].pt.y;
		boardPoints2D.push_back(Point2d(X_2d, Y_2d));

		// --> 3D
		int X_3d = (X_2d * m_pattern_WidthReal) / dim_x_2D;
		int Y_3d = (Y_2d * m_pattern_HeightReal) / dim_y_2D;
		boardPoints3D.push_back(Point3d(X_3d, Y_3d, Zconst));
	}
	// e mappo i punti 2D sulla scena 3D

	if (keypoints_cam.size() > 0) {
		//if (!pOriginCalculated) {
		//	pOrigin = boardPoints2D[0];
		//	pOriginX = boardPoints2D[1];
		//	pOriginY = boardPoints2D[2];
		//	pOriginZ = boardPoints2D[3];
		//	pOriginCalculated = true;
		//}
		res = true;
	}

	return res;
}

/**
 *
 */
void PatternPositionTracker::estimate(cv::Mat &webcamImage) {

	if (!pFixedCalculated) {
		//punto fisso del quale voglio calcolare le coordinate 3D
		pFixed.x = webcamImage.cols / 2;
		pFixed.y = webcamImage.rows / 2;
		uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
		uvPoint.at<double>(0, 0) = pFixed.x;
		uvPoint.at<double>(1, 0) = pFixed.y;
		pFixedCalculated = true;

		/*cv::Mat tmp = webcamImage.clone();//primo frame
		 int x_ = webcamImage.cols/6;
		 int y_ = webcamImage.rows/6;
		 Point point1(x_, y_);
		 Point point2(webcamImage.rows, webcamImage.rows );
		 img_pattern = tmp(cv::Rect(point1, point2));
		 imshow("img_pattern", img_pattern);
		 */
	}

	//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	vector<Point2d> boardPoints2D, imageFramePoints;
	bool found = false;
	try {
		// detect pattern corners
		WLOG("estimate:: detect pattern corners");
		Mat gray;
		cvtColor(webcamImage, gray, COLOR_BGR2GRAY);
		//bool found = findChessboardCorners(gray, cbSize, boardPoints2D,
		//		CALIB_CB_FAST_CHECK);
		WLOG("estimate:: call findPatternBorders");
		found = findPatternBorders(img_pattern, gray, boardPoints2D);
	} catch (cv::Exception &cvError) {
		found = false;
	}

	// find camera orientation if the chessboard corners have been found
	if (found) {

		// find the camera extrinsic parameters
		// do per certa la corrsipondenza fra in punti 3D (espressa in mm) e quelli in 2D
		WLOG("estimate:: solvePnP");
		solvePnP(Mat(boardPoints3D), Mat(boardPoints2D), cameraMatrix,
				distortionCoeff, rvec, tvec, false);

		WLOG("estimate:: Rodrigues");
		cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
		cv::Rodrigues(rvec, rotationMatrix);

		// calcolo le coordinate 3D del punto centrale della camera (u,v,1)
		WLOG("estimate:: calculating 3D Coordinates");
		cv::Mat tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
		cv::Mat tempMat2 = rotationMatrix.inv() * tvec;
		double s = Zconst + tempMat2.at<double>(2, 0);
		s /= tempMat.at<double>(2, 0);
		cerr << "s = " << s << endl;

		cv::Mat cam3DPoint = rotationMatrix.inv()
				* (s * cameraMatrix.inv() * uvPoint - tvec);

		// rifaccio l'operazione per calcolare solo lo spostamento su Z
		cv::Mat cam3DPointOnlyForZ = rotationMatrix.inv()
				* (cameraMatrix.inv() * uvPoint - tvec);
		cam3DPoint.at<double>(2, 0) = (cam3DPointOnlyForZ.at<double>(2, 0)) / 1;

		// scrivo l'output sul file di destinazione ...
//		m_CoordinatesFile << cam3DPoint << endl;//<-------------DEMO

		// ... e su video
		stringstream st_coordinates;
		st_coordinates << cam3DPoint;

		//project the reference frame onto the image
		WLOG("estimate:: project the reference frame onto the image");
		projectPoints(framePoints, rvec, tvec, cameraMatrix, distortionCoeff,
				imageFramePoints);

		WLOG("estimate:: drawing scene reference");
		drawSceneReference(webcamImage, pFixed, boardPoints2D,
				imageFramePoints);
		cv::putText(webcamImage, st_coordinates.str(),
				cv::Point(pFixed.x + 1, pFixed.y + 1),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
				cv::Scalar(0, 0, 255, 255));
		/*

		 //show the pose estimation data
		 cout << fixed << setprecision(2) << "rvec = ["
		 << rvec.at<double>(0, 0) << ", " << rvec.at<double>(1, 0)
		 << ", " << rvec.at<double>(2, 0) << "] \t" << "tvec = ["
		 << tvec.at<double>(0, 0) << ", " << tvec.at<double>(1, 0)
		 << ", " << tvec.at<double>(2, 0) << "]" << endl;
		 */
	}

	//disegno il punto della fotocamera
	cv::circle(webcamImage, pFixed, 8, CV_RGB(263, 83, 0), -1);

}

/**
 *
 */
void PatternPositionTracker::estimate2(cv::Mat &webcamImage) {

	if (!pFixedCalculated) {
		//punto fisso del quale voglio calcolare le coordinate 3D
		pFixed.x = webcamImage.cols / 2;
		pFixed.y = webcamImage.rows / 2;
		uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
		uvPoint.at<double>(0, 0) = pFixed.x;
		uvPoint.at<double>(1, 0) = pFixed.y;
		pFixedCalculated = true;

	}

	//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	vector<Point2d> boardPoints2D, imageFramePoints;
	std::vector<KeyPoint> keypoints_cam;
	bool found = false;
	try {
		// detect pattern corners
		WLOG("estimate:: detect pattern corners");
		Mat gray;
		cvtColor(webcamImage, gray, COLOR_BGR2GRAY);

		WLOG("estimate:: call findPatternBorders");
		found = mapKeyPoints(gray, boardPoints2D, boardPoints3D, keypoints_cam);
	} catch (cv::Exception &cvError) {
		found = false;
	} catch (...) {
		found = false;
	}

	// find camera orientation if the chessboard corners have been found
	if (found) {

		// find the camera extrinsic parameters
		// do per certa la corrsipondenza fra in punti 3D (espressa in mm) e quelli in 2D
		WLOG("estimate:: solvePnP");
		//
		//bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )¶
		//
		solvePnP(Mat(boardPoints3D), Mat(boardPoints2D), cameraMatrix,
				distortionCoeff, rvec, tvec, false);

		WLOG("estimate:: Rodrigues");
		cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
		cv::Rodrigues(rvec, rotationMatrix);

		// calcolo le coordinate 3D del punto centrale della camera (u,v,1)
		WLOG("estimate:: calculating 3D Coordinates");
		cv::Mat tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
		cv::Mat tempMat2 = rotationMatrix.inv() * tvec;
		double s = Zconst + tempMat2.at<double>(2, 0);
		s /= tempMat.at<double>(2, 0);

		cv::Mat cam3DPoint = rotationMatrix.inv()
				* (s * cameraMatrix.inv() * uvPoint - tvec);

		// rifaccio l'operazione per calcolare solo lo spostamento su Z
		cv::Mat cam3DPointOnlyForZ = rotationMatrix.inv()
				* (cameraMatrix.inv() * uvPoint - tvec);
		cam3DPoint.at<double>(2, 0) = (cam3DPointOnlyForZ.at<double>(2, 0)) / 1;

		// scrivo l'output sul file di destinazione ...
//		m_CoordinatesFile << cam3DPoint << endl;//<-------------DEMO

		// ... e su video
		stringstream st_coordinates;
		st_coordinates << cam3DPoint;

		//project the reference frame onto the image
		WLOG("estimate:: project the reference frame onto the image");
		projectPoints(framePoints, rvec, tvec, cameraMatrix, distortionCoeff,
				imageFramePoints);

		WLOG("estimate:: drawing scene reference");

		// disegno la linea dall'origine al punto fisso sullo schermo
		cv::line(webcamImage, boardPoints2D[0], pFixed, CV_RGB(0, 0, 255), 2);

		// disegno i punti significativi
		cv::drawKeypoints(webcamImage, keypoints_cam, webcamImage,
				Scalar::all(-1), DrawMatchesFlags::DEFAULT);

		// scrivo le coordinate a video
		cv::putText(webcamImage, st_coordinates.str(),
				cv::Point(pFixed.x + 1, pFixed.y + 1),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
				cv::Scalar(0, 0, 255, 255));
		/*
		 //disegno gli assi cartesiani dell'origine
		 line(webcamImage, pOrigin, pOriginX, CV_RGB(0, 0, 255), 2);
		 putText(webcamImage, "x", pOriginX, FONT_HERSHEY_SIMPLEX, 1,
		 Scalar::all(255), 1, 8);

		 line(webcamImage, pOrigin, pOriginY, CV_RGB(0, 0, 255), 2);
		 putText(webcamImage, "y", pOriginY, FONT_HERSHEY_SIMPLEX, 1,
		 Scalar::all(255), 1, 8);

		 line(webcamImage, pOrigin, pOriginZ, CV_RGB(0, 0, 255), 2);
		 putText(webcamImage, "z", pOriginZ, FONT_HERSHEY_SIMPLEX, 1,
		 Scalar::all(255), 1, 8);

		 //disegno gli assi cartesiani
		 line(webcamImage, boardPoints2D[0], boardPoints2D[1], CV_RGB(0, 255, 0),
		 2);
		 putText(webcamImage, "x", boardPoints2D[1], FONT_HERSHEY_SIMPLEX, 1,
		 Scalar::all(255), 1, 8);

		 line(webcamImage, boardPoints2D[0], boardPoints2D[2], CV_RGB(0, 255, 0),
		 2);
		 putText(webcamImage, "y", boardPoints2D[2], FONT_HERSHEY_SIMPLEX, 1,
		 Scalar::all(255), 1, 8);

		 //line(webcamImage, boardPoints2D[0], boardPoints2D[3], CV_RGB(0, 255, 0),
		 //		2);
		 //putText(webcamImage, "z", boardPoints2D[3], FONT_HERSHEY_SIMPLEX, 1,
		 //		Scalar::all(255), 1, 8);
		 */
		cerr << st_coordinates.str() << endl;

		/*

		 //show the pose estimation data
		 cout << fixed << setprecision(2) << "rvec = ["
		 << rvec.at<double>(0, 0) << ", " << rvec.at<double>(1, 0)
		 << ", " << rvec.at<double>(2, 0) << "] \t" << "tvec = ["
		 << tvec.at<double>(0, 0) << ", " << tvec.at<double>(1, 0)
		 << ", " << tvec.at<double>(2, 0) << "]" << endl;
		 */
	}

	//cerr << "imageFramePoints.size() = " << imageFramePoints.size() << endl;

	//disegno il punto dell'origine di riferimento
	if (!pOriginCalculated) {
		pOrigin = imageFramePoints[0];
		//	pOriginX = boardPoints2D[1];
		//	pOriginY = boardPoints2D[2];
		//	pOriginZ = boardPoints2D[3];
		pOriginCalculated = true;
	}
	cv::circle(webcamImage, pOrigin, 8, CV_RGB(263, 83, 185), -1);

	//disegno il punto della fotocamera
	cv::circle(webcamImage, pFixed, 8, CV_RGB(263, 83, 0), -1);

}

/**
 * ritorna il quadrante in cui si sta dirigendo il punto
 */
int calculateMoving(const cv::Mat &prec_marker3DPoint,
		const cv::Mat &cur_marker3DPoint, string& val) {

	string val_X;
	string val_Y;
	string val_Z;

	// controllo movimento su X
	if (cur_marker3DPoint.at<double>(0, 0)
			> prec_marker3DPoint.at<double>(0, 0))
		val_X = "RIGHT";
	else if (cur_marker3DPoint.at<double>(0, 0)
			< prec_marker3DPoint.at<double>(0, 0))
		val_X = "LEFT";
	else
		val_X = "STABLE";

	// controllo movimento su Y (Sistema coordinate solidale a OpenCV)
	if (cur_marker3DPoint.at<double>(1, 0)
			> prec_marker3DPoint.at<double>(1, 0))
		val_Y = "DOWN";
	else if (cur_marker3DPoint.at<double>(1, 0)
			< prec_marker3DPoint.at<double>(1, 0))
		val_Y = "UP";
	else
		val_Y = "STABLE";

	// controllo movimento su Z
	if (cur_marker3DPoint.at<double>(2, 0)
			> prec_marker3DPoint.at<double>(2, 0))
		val_Z = "FRONT";
	else if (cur_marker3DPoint.at<double>(2, 0)
			< prec_marker3DPoint.at<double>(2, 0))
		val_Z = "BACK";
	else
		val_Z = "STABLE";

	stringstream ss;
	ss << val_X << " - " << val_Y << " - " << val_Z;
	val = ss.str();

	//determino il quadrante
	int quad = -1;
	if (val_X == "RIGHT") {
		if ((val_Y == "DOWN") || (val_Y == "STABLE"))
			quad = 1; //quadrante positivo
		else if (val_Y == "UP")
			quad = 2; //quadrante positivo
	} else if (val_X == "LEFT") {
		if ((val_Y == "DOWN") || (val_Y == "STABLE"))
			quad = 4; //quadrante positivo
		else if (val_Y == "UP")
			quad = 3; //quadrante positivo
	}

	return quad;
}

/**
 *
 */
int calculateMoving2(const cv::Point &prec_marker, const cv::Point &cur_marker,
		string& val) {

	string val_X;
	string val_Y;
	string val_Z;

	// controllo movimento su X
	if (cur_marker.x > prec_marker.x)
		val_X = "RIGHT";
	else if (cur_marker.x < prec_marker.x)
		val_X = "LEFT";
	else
		val_X = "STABLE";

	// controllo movimento su Y (Sistema coordinate solidale a OpenCV)
	if (cur_marker.y > prec_marker.y)
		val_Y = "DOWN";
	else if (cur_marker.y < prec_marker.y)
		val_Y = "UP";
	else
		val_Y = "STABLE";

	val_Z = "???";

	stringstream ss;
	ss << val_X << " - " << val_Y << " - " << val_Z;
	val = ss.str();

	//determino il quadrante
	int quad = -1;
	if (val_X == "RIGHT") {
		if ((val_Y == "DOWN") || (val_Y == "STABLE"))
			quad = 1; //quadrante positivo
		else if (val_Y == "UP")
			quad = 2; //quadrante positivo
	} else if (val_X == "LEFT") {
		if ((val_Y == "DOWN") || (val_Y == "STABLE"))
			quad = 4; //quadrante positivo
		else if (val_Y == "UP")
			quad = 3; //quadrante positivo
	}

	return quad;
}

/**
 * q marker corrente
 */
cv::Point PatternPositionTracker::nextMarker(
		const std::vector<KeyPoint> &listMarker, const cv::Point q,
		const int quadrante) {

	//
	// calcolo la lista dei marker vicini ovvero quelli che sono ad una certa distanza epsilon
	//
	const int eps = 25;
	std::vector<cv::KeyPoint> listNeighMarker;
	getNeighboursMarker(q, listMarker, eps, listNeighMarker);

	cv::Point r;

	// se ci sono vicini seleziono il primo della lista ovvero quello piu vicino
	if (listNeighMarker.size() > 0) {
		//cerr << "----> seleziono il marker piu vicino" << endl;
		r = listNeighMarker[0].pt;
	} else {
		cerr << "----> seleziono il primo marker" << endl;
		r = listMarker[0].pt; //dovrebbe essere il primo marker
	}
	return r;

	// altrimenti seleziono quello che è sulla stessa traiettoria del precendete..
	cerr << "----> NO NEIGHBOURS! <-------" << endl;
	if (quadrante == -1) {
		cerr << "----> seleziono il primo marker" << endl;
		r = listMarker[0].pt; //dovrebbe essere il primo marker
	} else {
		cerr << "----> seleziono il marker dal quadrante =" << quadrante
				<< endl;
		std::vector<KeyPoint> listMarkerQuad;
		for (int i = 0; i < listMarker.size(); ++i) {
			Point p = listMarker[i].pt;
			//inserisco nella lista solo i punti nel quadrante relativo
			if (quadrante == 1) { // +x +y
				if ((p.x >= q.x) && (p.y >= q.y)) {
					listMarkerQuad.push_back(listMarker[i]);
				}
			} else if (quadrante == 2) { //+x -y
				if ((p.x >= q.x) && (p.y <= q.y)) {
					listMarkerQuad.push_back(listMarker[i]);
				}
			} else if (quadrante == 3) { //-x -y
				if ((p.x <= q.x) && (p.y <= q.y)) {
					listMarkerQuad.push_back(listMarker[i]);
				}
			} else if (quadrante == 4) { //-x +y
				if ((p.x <= q.x) && (p.y >= q.y)) {
					listMarkerQuad.push_back(listMarker[i]);
				}
			}
		}

		if (listMarkerQuad.size() > 0)
			r = listMarkerQuad[0].pt;
		else
			r = q;
	}
	return r;
}

cv::Mat PatternPositionTracker::poseEstimation(const cv::Point &marker) {

	cv::Mat marker3DPoint;

	// ricavo i vettori dello spsotamento dell'origine nel nuovo riferimento
	WLOG("estimate:: solvePnP");
	solvePnP(Mat(boardPoints3D), Mat(boardPoints2D), cameraMatrix,
			distortionCoeff, rvec, tvec, false);

	WLOG("estimate:: Rodrigues");
	rotationMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
	cv::Rodrigues(rvec, rotationMatrix);

	//Point tmp(pFixed.x + 20, pFixed.y + 20);
	// dovrebbe essere il vettore relativo al marker che sto seguendo sulla scena...
	uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	uvPoint.at<double>(0, 0) = marker.x;
	uvPoint.at<double>(1, 0) = marker.y;

	// calcolo le coordinate 3D del punto centrale della camera (u,v,1)
	WLOG("estimate:: calculating 3D Coordinates");
	cv::Mat tempMat = rotationMatrix.inv() * cameraMatrix.inv() * uvPoint;
	cv::Mat tempMat2 = rotationMatrix.inv() * tvec;
	double s = Zconst + tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);

	marker3DPoint = rotationMatrix.inv()
			* (s * cameraMatrix.inv() * uvPoint - tvec);

	// rifaccio l'operazione per calcolare solo lo spostamento su Z
	cv::Mat cam3DPointOnlyForZ = rotationMatrix.inv()
			* (cameraMatrix.inv() * uvPoint - tvec);
	marker3DPoint.at<double>(2, 0) = (cam3DPointOnlyForZ.at<double>(2, 0)) / 1;

	return marker3DPoint;
}

/**
 *
 */
void PatternPositionTracker::addMarkerToScene(const cv::Point &m) {
	boardPoints2D.clear();

	// Punto 1 (origine sistema)
	cv::Point marker = m;
	boardPoints2D.push_back(marker);

	// Punto 2
	marker.x = m.x;
	marker.y = m.y + 200;
	boardPoints2D.push_back(marker);

	// Punto 3
	marker.x = m.x + 200;
	marker.y = m.y + 200;
	boardPoints2D.push_back(marker);

	// Punto 4
	marker.x = m.x + 200;
	marker.y = m.y;
	boardPoints2D.push_back(marker);
}

/**
 * neighbourhood points of any point p
 **/
double getDistance(cv::Point p, cv::Point q) {
	int dx = p.x - q.x;
	int dy = p.y - q.y;
	return sqrt(dx * dx + dy * dy);
}

void PatternPositionTracker::getNeighboursMarker(const cv::Point p,
		const std::vector<cv::KeyPoint> &pointList, const int &eps,
		std::vector<cv::KeyPoint> &neigh) {
	neigh.clear();
	for (unsigned int i = 0; i < pointList.size(); ++i) {
		cv::KeyPoint q = pointList[i];
		if (getDistance(p, q.pt) <= eps) {
			neigh.push_back(q);
		}
	}
}

/**
 *
 */
void PatternPositionTracker::estimate3(cv::Mat &webcamImage) {

//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	vector<Point2d> imageFramePoints;
	std::vector<KeyPoint> markerList;
	cv::Mat marker3DPoint;
	bool found = false;
	try {
		// detect pattern corners
		WLOG("estimate:: detect pattern corners");
		Mat gray;
		cvtColor(webcamImage, gray, COLOR_BGR2GRAY);

		//-- Step 1: Detect the keypoints using SURF Detector
		WLOG("findKeyPoints:: Creating SURF Detector");
		int minHessian = 400;
		SurfFeatureDetector detector(minHessian);

		WLOG("findKeyPoints:: Detect the keypoints using SURF Detector");
		detector.detect(gray, markerList);

		if (markerList.size() > 0) {
			int dim_x_2D = webcamImage.cols;
			int dim_y_2D = webcamImage.rows;

			//Point marker(keypoints_cam[0].pt);
			Point marker = nextMarker(markerList, prec_marker,
					prec_marker_quadrante);

			addMarkerToScene(marker);
			//il contorno del pattern
			line(webcamImage, boardPoints2D[0], boardPoints2D[1],
					CV_RGB(0, 255, 0), 2);
			line(webcamImage, boardPoints2D[1], boardPoints2D[2],
					CV_RGB(0, 255, 0), 2);
			line(webcamImage, boardPoints2D[2], boardPoints2D[3],
					CV_RGB(0, 255, 0), 2);
			line(webcamImage, boardPoints2D[3], boardPoints2D[0],
					CV_RGB(0, 255, 0), 2);

			//		if (boardPoints2D.size() >= 4) {

			marker3DPoint = poseEstimation(pFixed);

			stringstream st_coordinates;
			st_coordinates << marker3DPoint;
			cerr << st_coordinates.str() << endl;

			if (!prec_markerCalculated) {
				prec_marker = marker;
				prec_marker3DCoord = marker3DPoint;
				prec_markerCalculated = true;
				return;
			}

			// disegno i punti significativi
			cv::drawKeypoints(webcamImage, markerList, webcamImage,
					Scalar::all(-1), DrawMatchesFlags::DEFAULT);

			//disegno i marker cur e prec
			cv::circle(webcamImage, marker, 8, Scalar(255, 128, 0), -1);
			cv::circle(webcamImage, prec_marker, 8, Scalar(51, 255, 255), -1);

			// scrivo le coordinate a video
//			cv::putText(webcamImage, st_coordinates.str(),
//					cv::Point(pFixed.x + 1, pFixed.y + 1),
//					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
//					cv::Scalar(0, 0, 255, 255));

			//disegno una linea tra il baricentro della fotocamera ed il marker
			cv::circle(webcamImage, pFixed, 8, CV_RGB(263, 83, 0), -1);
			line(webcamImage, marker, pFixed, CV_RGB(0, 0, 255), 2);

//			int radius = 15;
			//controllo se i keypoint sono quasi uguali vedenso se
			//appartengo allo stesso cerchio
//			double D = sqrt(
//					pow(prec_marker.x - marker.x, 2)
//							+ pow(prec_marker.y - marker.y, 2));
			//cerr << "D=" << D << endl;

			//se il marker è cambiato ne riseleziono uno ad hoc
//				if (D >= radius) {
//					cv::putText(webcamImage, "MARKER CAMBIATO",
//							cv::Point(pFixed.x + 10, pFixed.y + 10),
//							cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
//							cv::Scalar(0, 0, 255, 255));
//				}

			string move;
			//prec_marker_quadrante = calculateMoving(prec_marker3DCoord,	marker3DPoint, move);
			int quad = calculateMoving2(prec_marker, marker, move);
			if (quad != -1)
				prec_marker_quadrante = quad;
			prec_marker = marker;
			prec_marker3DCoord = marker3DPoint;

			cv::putText(webcamImage, move,
					cv::Point(pFixed.x + 10, pFixed.y + 30),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
					cv::Scalar(0, 255, 0, 255));

			stringstream ss;
			ss << "QUAD= " << prec_marker_quadrante;
			cv::putText(webcamImage, ss.str(),
					cv::Point(pFixed.x + 10, pFixed.y + 50),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
					cv::Scalar(0, 255, 0, 255));

			//	}
			waitKey(0);
		}
	} catch (cv::Exception &cvError) {
		found = false;
	} catch (...) {
		found = false;
	}

}

static string _type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:
		r = "8U";
		break;
	case CV_8S:
		r = "8S";
		break;
	case CV_16U:
		r = "16U";
		break;
	case CV_16S:
		r = "16S";
		break;
	case CV_32S:
		r = "32S";
		break;
	case CV_32F:
		r = "32F";
		break;
	case CV_64F:
		r = "64F";
		break;
	default:
		r = "User";
		break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

/**
 *
 */
void PatternPositionTracker::estimate4(cv::Mat &webcamImage) {

//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	vector<Point2d> imageFramePoints;
	std::vector<KeyPoint> keypoints_cam;
	cv::Mat marker3DPoint;
	bool found = false;
	try {
		// detect pattern corners
		WLOG("estimate:: detect pattern corners");
		Mat gray;
		cvtColor(webcamImage, gray, COLOR_BGR2GRAY);

		//-- Step 1: Detect the keypoints using SURF Detector
		WLOG("findKeyPoints:: Creating SURF Detector");
		int minHessian = 400;
		SurfFeatureDetector detector(minHessian);

		WLOG("findKeyPoints:: Detect the keypoints using SURF Detector");
		detector.detect(webcamImage, keypoints_cam);
		WLOG(
				"findKeyPoints:: Detect the keypoints using SURF Detector -> done!");

		if (keypoints_cam.size() >= 4) {
			boardPoints2D.clear();
			boardPoints3D.clear();

			int dim_x_2D = webcamImage.cols;
			int dim_y_2D = webcamImage.rows;

			//aggiungo i marker alla scena 2D e 3D
			for (int i = 0; i < 5/*keypoints_cam.size()*/; ++i) {
				Point marker = keypoints_cam[i].pt;

				boardPoints2D.push_back(marker);
				double x3D = (marker.x * m_pattern_WidthReal) / dim_x_2D;
				double y3D = (marker.y * m_pattern_HeightReal) / dim_y_2D;
				boardPoints3D.push_back(Point3d(x3D, y3D, Zconst));
			}

			// ricavo i vettori dello spsotamento dell'origine nel nuovo riferimento
			WLOG("estimate:: solvePnP");
			solvePnP(Mat(boardPoints3D), Mat(boardPoints2D), cameraMatrix,
					distortionCoeff, rvec, tvec, false);

			WLOG("estimate:: Rodrigues");
			rotationMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
			cv::Rodrigues(rvec, rotationMatrix);

			// disegno i punti significativi
			cv::drawKeypoints(webcamImage, keypoints_cam, webcamImage,
					Scalar::all(-1), DrawMatchesFlags::DEFAULT);

			// calcolo le coordinate 3D del punto centrale della camera (u,v,1)
			uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
			uvPoint.at<double>(0, 0) = pFixed.x;
			uvPoint.at<double>(1, 0) = pFixed.y;

			//disegno il punto della fotocamera
			cv::circle(webcamImage, pFixed, 8, CV_RGB(263, 83, 0), -1);

			cerr << "tvec = " << tvec << endl;
			cerr << "rvec = " << rvec << endl;
			//cerr << "rotationMatrix.type() = " << _type2str(rotationMatrix.type()) << endl;

			cv::Mat tmp = rotationMatrix * tvec;
			//cerr << "tmp.rows = " << tmp.rows << endl;
			//cerr << "tmp.cols = " << tmp.cols << endl;
			Mat warp_uvpoint(1, 3, CV_64F);
			warp_uvpoint.at<double>(0, 0) = uvPoint.at<double>(0, 0);
			warp_uvpoint.at<double>(0, 1) = uvPoint.at<double>(1, 0);
			warp_uvpoint.at<double>(0, 2) = uvPoint.at<double>(2, 0);

			cv::Mat cam3DPose = tmp * warp_uvpoint;
			cerr << "cam3DPose = " << cam3DPose << endl;

			//---------------------------------------------------------------
			Mat warp_tvec(2, 3, CV_64F);
			warp_tvec.at<double>(0, 0) = 1;
			warp_tvec.at<double>(0, 1) = 0;
			warp_tvec.at<double>(0, 2) = tvec.at<double>(0, 0);
			warp_tvec.at<double>(1, 0) = 0;
			warp_tvec.at<double>(1, 1) = 1;
			warp_tvec.at<double>(1, 2) = tvec.at<double>(1, 0);
			//warp_tvec.at<double>(2,0) = 0;
			//warp_tvec.at<double>(2,1) = 1;
			//warp_tvec.at<double>(2,2) = tvec.at<double>(2, 0);

			Mat rot_mat(2, 3, CV_64F);
			rot_mat.at<double>(0, 0) = rotationMatrix.at<double>(0, 0);
			rot_mat.at<double>(0, 1) = rotationMatrix.at<double>(0, 1);
			rot_mat.at<double>(0, 2) = rotationMatrix.at<double>(0, 2);
			rot_mat.at<double>(1, 0) = rotationMatrix.at<double>(1, 0);
			rot_mat.at<double>(1, 1) = rotationMatrix.at<double>(1, 1);
			rot_mat.at<double>(1, 2) = rotationMatrix.at<double>(1, 2);

			img_pattern = imread(m_PatternFileName, CV_LOAD_IMAGE_COLOR);

			/// Set the dst image the same type and size as src
			cv::Mat warp_dst = Mat::zeros(img_pattern.rows, img_pattern.cols,
					img_pattern.type());

			/// Apply the Affine Transform just found to the src image
			warpAffine(img_pattern, warp_dst, warp_tvec, warp_dst.size());
			//imshow("warp_dst", warp_dst);

			/// Rotate the warped image
			cv::Mat warp_rotate_dst = Mat::zeros(img_pattern.rows,
					img_pattern.cols, img_pattern.type());
			warpAffine(warp_dst, warp_rotate_dst, rot_mat, warp_dst.size());

			imshow("warp_rotate_dst", warp_rotate_dst);
			waitKey(1);

		}
	} catch (cv::Exception &cvError) {
		found = false;
	} catch (...) {
		found = false;
	}

}

/**
 *
 */
void traslateCameraPoseTo(const cv::Point &marker, cv::Mat &cam3DPose) {

	// ricavo il vaettore traslazione del marker verso l'origine
	Point2f markerSrc[1];
	markerSrc[0] = marker;

	Point2f markerTrasl[1]; //coordinate del marker traslato
	markerTrasl[0] = Point2f(0, 0);

	// Get the Affine Transform
	//Mat warp_mat( 2, 3, CV_32FC1 );
	//warp_mat = getAffineTransform( markerSrc, markerTrasl );
	cv::Mat warp_mat = cv::Mat::ones(2, 3, cv::DataType<double>::type);
	warp_mat.at<double>(0, 0) = 1;
	warp_mat.at<double>(0, 1) = 0;
	warp_mat.at<double>(0, 2) = marker.x;
	warp_mat.at<double>(1, 0) = 0;
	warp_mat.at<double>(1, 1) = 1;
	warp_mat.at<double>(1, 2) = marker.y;
	//cout << "warp_mat = " << warp_mat << endl;

	//e lo applico alle coordinate della fotocamera
	cv::Mat camSrc = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	camSrc.at<double>(0, 0) = cam3DPose.at<double>(0, 0);
	camSrc.at<double>(0, 1) = cam3DPose.at<double>(1, 0);
	camSrc.at<double>(0, 2) = 1;
	cout << "camSrc = " << camSrc << endl;

	// Apply the Affine Transform just found to the src image
	cv::Mat camDst = warp_mat * camSrc;

	//
	//cam3DPose.at<double>(0, 0) = camDst.at<double>(0, 0) ;
	//cam3DPose.at<double>(1, 0) = camDst.at<double>(1, 0) ;

	cout << "camDst = " << camDst << endl;
}

/**
 *
 */
void PatternPositionTracker::estimate5(cv::Mat &webcamImage) {
	//setup vectors to hold the chessboard corners in the chessboard coordinate system and in the image
	std::vector<KeyPoint> markerList;
	bool found = false;
	try {
		// detect pattern corners
		WLOG("estimate:: detect pattern corners");
		Mat gray;
		cvtColor(webcamImage, gray, COLOR_BGR2GRAY);

		//-- Step 1: Detect the keypoints using SURF Detector
		WLOG("estimate:: Creating SURF Detector");
		int minHessian = 400;
		SurfFeatureDetector detector(minHessian);

		WLOG("estimate:: Detect the keypoints using SURF Detector");
		detector.detect(gray, markerList);

		if (markerList.size() > 0) {
			int dim_x_2D = webcamImage.cols;
			int dim_y_2D = webcamImage.rows;

			WLOG("estimate::calculating next marker");
			Point marker = nextMarker(markerList, prec_marker,
					prec_marker_quadrante);
			cv::circle(webcamImage, marker, 8, Scalar(255, 128, 0), -1);

			addMarkerToScene(marker);

			// disegno il contorno del pattern
			line(webcamImage, boardPoints2D[0], boardPoints2D[1],
					CV_RGB(0, 255, 0), 2);
			line(webcamImage, boardPoints2D[1], boardPoints2D[2],
					CV_RGB(0, 255, 0), 2);
			line(webcamImage, boardPoints2D[2], boardPoints2D[3],
					CV_RGB(0, 255, 0), 2);
			line(webcamImage, boardPoints2D[3], boardPoints2D[0],
					CV_RGB(0, 255, 0), 2);

			// find the camera extrinsic parameters
			// do per certa la corrsipondenza fra in punti 3D (espressa in mm) e quelli in 2D
			WLOG("estimate:: solvePnP");
			solvePnP(Mat(boardPoints3D), Mat(boardPoints2D), cameraMatrix,
					distortionCoeff, rvec, tvec, false);

			WLOG("estimate:: Rodrigues");
			cv::Mat rotationMatrix(3, 3, cv::DataType<double>::type);
			cv::Rodrigues(rvec, rotationMatrix);

			// calcolo le coordinate 3D del punto centrale della camera (u,v,1)
			WLOG("estimate:: calculating 3D Coordinates");
			cv::Mat tempMat = rotationMatrix.inv() * cameraMatrix.inv()
					* uvPoint;

			cv::Mat tempMat2 = rotationMatrix.inv() * tvec;
			double s = Zconst + tempMat2.at<double>(2, 0);
			s /= tempMat.at<double>(2, 0);
			//cerr << "s = " << s << endl;

			cv::Mat cam3DPoint = rotationMatrix.inv()
					* (s * cameraMatrix.inv() * uvPoint - tvec);

			// rifaccio l'operazione per calcolare solo lo spostamento su Z
			cv::Mat cam3DPointOnlyForZ = rotationMatrix.inv()
					* (cameraMatrix.inv() * uvPoint - tvec);
			cam3DPoint.at<double>(2, 0) = (cam3DPointOnlyForZ.at<double>(2, 0))
					/ 1;

			// eseguo la traslazione del punto 3d calcolato rispetto all'origine
			// in alto a sinistra del frame
			traslateCameraPoseTo(marker, cam3DPoint);

			// scrivo l'output sul file di destinazione ...
//		m_CoordinatesFile << cam3DPoint << endl;//<-------------DEMO

			// ... e su video
			stringstream st_coordinates;
			st_coordinates << cam3DPoint;

			//project the reference frame onto the image
			//WLOG("estimate:: project the reference frame onto the image");
			//projectPoints(framePoints, rvec, tvec, cameraMatrix,
			//		distortionCoeff, imageFramePoints);

			WLOG("estimate:: drawing scene reference");
			cv::putText(webcamImage, st_coordinates.str(),
					cv::Point(pFixed.x + 1, pFixed.y + 1),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
					cv::Scalar(0, 0, 255, 255));

			//disegno il punto della fotocamera
			cv::line(webcamImage, marker, pFixed, CV_RGB(0, 0, 255), 2);
			cv::circle(webcamImage, pFixed, 8, CV_RGB(263, 83, 0), -1);

			found = true;
		}
	} catch (cv::Exception &cvError) {
		found = false;
	} catch (...) {
		found = false;
	}

}
