/*
 * poseEstimation.cpp
 *
 *  Created on: 29/set/2014
 *      Author: Sab
 */

#include <string>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "Utils.h"

using namespace cv;
using namespace std;

// parametri Chessboard pattern
const int boardHeight = 6;
const int boardWidth = 9;
const int DIM_MM_QUAD = 25; //dimensione in millimetri di ogni quadratino

class PatternPositionTracker {
public:
	PatternPositionTracker(const string &filename);
	PatternPositionTracker(const string &filename, const cv::Mat &webcamImage);
	~PatternPositionTracker();

	void estimate(cv::Mat &webcamImage);
	void estimate2(cv::Mat &webcamImage);
	void estimate3(cv::Mat &webcamImage);
	void estimate4(cv::Mat &webcamImage);
	void estimate5(cv::Mat &webcamImage);


	cv::Mat poseEstimation(const cv::Point &marker);
	int matchCam(cv::Mat img_1, cv::Mat img_2);
	void detectCorner(const cv::Mat &src) ;

private:
	void addMarkerToScene(const cv::Point &marker);

	cv::Point nextMarker(const std::vector<KeyPoint> &listMarker, const cv::Point q,
			const int quadrante);

	void getNeighboursMarker(const cv::Point p,
			const std::vector<cv::KeyPoint> &pointList, const int &eps,
			std::vector<cv::KeyPoint> &neigh);

	bool calculateHomography(const cv::Mat & img_object,
		const cv::Mat & img_cam, cv::Mat &H);

	void setUpParameters(const string &filename);

	void WLOG(const string& msg);
	void WLOG(const stringstream& msg);

	void setUpCameraModel(const string &filename, cv::Mat& cameraMatrix,
			cv::Mat& distortionCoeff);
	void initChessboard3DPoints(vector<Point3d> &boardPoints3D,
			const int boardWidth, const int boardHeight);
	void initPattern3DPoints(vector<Point3d> &boardPoints3D,
			const int sizeWidth, const int sizeHeight);
	void init3DPointsAxes(vector<Point3d> &framePoints, const int size);

	void drawSceneReference(cv::Mat &webcamImage, const Point2d& pFixed,
			const vector<Point2d>& boardPoints2D,
			const vector<Point2d>& imageFramePoints);
	bool isQuadrilateral(const vector<Point2d>& boardPoints2D);
	bool findPatternBorders(const cv::Mat & img_object, const cv::Mat & img_cam,
			vector<Point2d>& boardPoints2D);
	bool mapKeyPoints(const cv::Mat & img_cam, vector<Point2d>& boardPoints2D,
			vector<Point3d> &boardPoints3D,
			std::vector<KeyPoint> &keypoints_cam);
private:
	cv::Mat rotationMatrix;
	vector<Point2d> boardPoints2D;
	vector<Point2d> referenceFrameAxis;

	cv::Mat prec_marker3DCoord;
	int prec_marker_quadrante;
	Point prec_marker;
	bool prec_markerCalculated;

	bool pOriginCalculated;
	Point2d pOrigin; //primo keypoint individuato
	Point2d pOriginX; //primo keypoint individuato
	Point2d pOriginY; //primo keypoint individuato
	Point2d pOriginZ; //primo keypoint individuato

	bool pFixedCalculated;
	std::string m_CoordinatesFileName;
	std::ofstream m_CoordinatesFile;

	bool m_enableLog;
	std::string m_LogFileName;
	std::ofstream m_logFile;

	//double m_pattern_WidthPixel;
	//double m_pattern_HeightPixel;
	double m_pattern_WidthReal;
	double m_pattern_HeightReal;
	double Zconst;

	std::string m_PatternFileName;
	std::string m_CalibratorFileName;
	//Size cbSize;
	cv::Mat img_pattern;
	std::vector<Point3d> boardPoints3D;
	std::vector<Point3d> framePoints;
	cv::Mat uvPoint;
	Point2d pFixed;
	cv::Mat cameraMatrix;
	cv::Mat distortionCoeff;
	//set up matrices for storage
	cv::Mat rvec;
	cv::Mat tvec;
};

