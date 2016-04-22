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

class MarkerPositionTracker {
public:
	MarkerPositionTracker(const string &filename, const cv::Mat &webcamImage);
	~MarkerPositionTracker();

	void estimate(cv::Mat &webcamImage);

	cv::Mat poseEstimation(const cv::Point &marker);

private:
	bool addMarkerToScene(const cv::Point &marker, const cv::Mat &webcamImage);

	bool nextMarker(const std::vector<KeyPoint> &listMarker,
			const cv::Point q, cv::Point &new_marker, bool &isMarkerChanged);

	double getDistance(cv::Point p, cv::Point q);

	void getQuadrantMarker(const cv::Point p,
			const std::vector<cv::KeyPoint> &pointList,
			std::vector<cv::KeyPoint> &list);
	void getNeighboursMarker(const cv::Point p,
			const std::vector<cv::KeyPoint> &pointList, const int &eps,
			std::vector<cv::KeyPoint> &neigh);

	cv::Mat traslateCameraPoseToOrig(const cv::Point3f &marker, const cv::Mat &cam3DPose);

	void setUpParameters(const string &filename);

	void WLOG(const string& msg);
	void WLOG(const stringstream& msg);

	void setUpCameraModel(const string &filename, cv::Mat& cameraMatrix,
			cv::Mat& distortionCoeff);
	void set3DPointsReference(vector<Point3d> &boardPoints3D,
			const int offestX, const int offsetY,
			const int sizeWidth, const int sizeHeight);
	void drawSceneReference(cv::Mat &webcamImage, const Point2d& pFixed,
			const vector<Point2d>& boardPoints2D,
			const vector<Point2d>& imageFramePoints);
private:
	std::vector<string> listCoordinates;

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
	//cv::Mat rvec;
	//cv::Mat tvec;
};

