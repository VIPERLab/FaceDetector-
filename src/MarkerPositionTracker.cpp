/*
 * MarkerPositionTracker.cpp
 *
 *  Created on: 05/ott/2014
 *      Author: Sab
 */

#include "MarkerPositionTracker.h"

MarkerPositionTracker::MarkerPositionTracker(const string &filename,
		const cv::Mat &webcamImage) {

	if (!fileExists(filename.c_str())) {
		stringstream ss;
		ss << "ERROR: The settings file  " << filename << " does not exist!";
		throw std::runtime_error(ss.str());
	}

	setUpParameters(filename);

	WLOG("Started");
	m_CoordinatesFile.open(m_CoordinatesFileName.c_str());

	//set up matrices for storage
	//rvec = Mat(Size(3, 1), CV_64F);
	//tvec = Mat(Size(3, 1), CV_64F);
	setUpCameraModel(m_CalibratorFileName, cameraMatrix, distortionCoeff);

	//punto fisso 2D del quale voglio calcolare le coordinate 3D
	pFixed.x = webcamImage.cols / 2;
	pFixed.y = webcamImage.rows / 2;

	// punto 3D mobile del quale voglio calcolare le coordiante
	uvPoint = cv::Mat::ones(3, 1, cv::DataType<double>::type);
	uvPoint.at<double>(0, 0) = pFixed.x;
	uvPoint.at<double>(1, 0) = pFixed.y;
	uvPoint.at<double>(2, 0) = 1;

	// non mappo ancora l'origine 2D
	boardPoints2D.clear();

	// ma mappo l'origine 3D
	set3DPointsReference(boardPoints3D, 0, 0, m_pattern_WidthReal,
			m_pattern_HeightReal);

}

/**
 *
 */
MarkerPositionTracker::~MarkerPositionTracker() {
	if (m_enableLog) {
		m_logFile.close();
	}
	m_CoordinatesFile.close();
}

/**
 *
 */
void MarkerPositionTracker::WLOG(const string& msg) {
	if (m_enableLog) {
		m_logFile << "[" << getNowTime() << "]MarkerPositionTracker::" << msg
				<< endl;
	}
}

/**
 * No log!
 */
void MarkerPositionTracker::setUpParameters(const string &filename) {
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
void MarkerPositionTracker::WLOG(const stringstream& msg) {
	WLOG(msg.str());
}

/**
 *
 */
void MarkerPositionTracker::setUpCameraModel(const string &filename,
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

void MarkerPositionTracker::set3DPointsReference(vector<Point3d> &boardPoints3D,
		const int offsetX, const int offsetY, const int sizeWidth,
		const int sizeHeight) {
	WLOG("set3DPointsReference");

	boardPoints3D.clear();
	boardPoints3D.push_back(Point3d(offsetX, offsetY, Zconst));
	boardPoints3D.push_back(Point3d(offsetX, sizeHeight, Zconst));
	boardPoints3D.push_back(Point3d(sizeWidth, sizeHeight, Zconst));
	boardPoints3D.push_back(Point3d(sizeWidth, offsetY, Zconst));
}

/**
 *
 */
void MarkerPositionTracker::drawSceneReference(cv::Mat &webcamImage,
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
int calculateMoving(const cv::Point &prec_marker, const cv::Point &cur_marker,
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
bool MarkerPositionTracker::nextMarker(const std::vector<KeyPoint> &listMarker,
		const cv::Point q, cv::Point &new_marker, bool &isMarkerChanged) {

	bool found = false;
	std::vector<cv::KeyPoint> listMarkerQuad;
	//getQuadrantMarker(pFixed, listMarker, listMarkerQuad);

	listMarkerQuad = listMarker;
	if (listMarkerQuad.size() > 0) {
		//
		// calcolo la lista dei marker vicini ovvero quelli che sono ad una certa distanza epsilon
		//

		int listNeighMarkerSize = 0;
		int eps = 25;
		std::vector<cv::KeyPoint> listNeighMarker;
		while (listNeighMarkerSize <= 0) {
			listNeighMarker.clear();
			getNeighboursMarker(q, listMarkerQuad, eps, listNeighMarker);
			listNeighMarkerSize = listNeighMarker.size();

			eps++;
			if (eps == 100)
				break;
			//cout << "listNeighMarkerSize=" << listNeighMarkerSize << endl;
		}

		// se ci sono vicini seleziono il primo della lista ovvero quello piu vicino
		if (listNeighMarker.size() > 0) {
			//cerr << "----> seleziono il marker piu vicino" << endl;
			new_marker = listNeighMarker[0].pt;
			isMarkerChanged = false;
		} else {
			//cerr << "----> MARKER CAMBIATO <-----" << endl;
			new_marker = listMarkerQuad[0].pt; //dovrebbe essere il primo marker
			isMarkerChanged = true;
		}
		found = true;
	} else {
		found = false;
	}
	return found;

	/*
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
	 */
}

/**
 *
 */
bool MarkerPositionTracker::addMarkerToScene(const cv::Point &m,
		const cv::Mat &webcamImage) {

	int dim_size_x = m.x + 100;
	if ((dim_size_x) > webcamImage.cols)
		//dim_size_x = webcamImage.cols - 1;
		return false;

	int dim_size_y = m.y + 100;
	if ((dim_size_y) > webcamImage.rows)
		//dim_size_y = webcamImage.rows - 1;
		return false;

	boardPoints2D.clear();

	// Punto 1 (origine sistema)
	cv::Point marker = m;
	boardPoints2D.push_back(marker);

	// Punto 2
	marker.x = m.x;
	marker.y = dim_size_y;
	boardPoints2D.push_back(marker);

	// Punto 3
	marker.x = dim_size_x;
	marker.y = dim_size_y;
	boardPoints2D.push_back(marker);

	// Punto 4
	marker.x = dim_size_x;
	marker.y = m.y;
	boardPoints2D.push_back(marker);

	return true;
}

/**
 * neighbourhood points of any point p
 **/
double MarkerPositionTracker::getDistance(cv::Point p, cv::Point q) {
	int dx = p.x - q.x;
	int dy = p.y - q.y;
	return sqrt(dx * dx + dy * dy);
}

/**
 *
 */
void MarkerPositionTracker::getNeighboursMarker(const cv::Point p,
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

void MarkerPositionTracker::getQuadrantMarker(const cv::Point p,
		const std::vector<cv::KeyPoint> &pointList,
		std::vector<cv::KeyPoint> &list) {
	list.clear();
	for (unsigned int i = 0; i < pointList.size(); ++i) {
		cv::KeyPoint q = pointList[i];
		if ((q.pt.x <= p.x / 4) && (q.pt.y <= p.y / 4)) {
			list.push_back(q);
		}
	}
}

/**
 *
 */
cv::Mat MarkerPositionTracker::traslateCameraPoseToOrig(
		const cv::Point3f &center, const cv::Mat &_cam3dpose) {

	// Calcolo la trasformazione affine ovvero il vattore di traslazione del marker nel origine
	cv::Mat warp_mat = cv::Mat::ones(3, 4, cv::DataType<double>::type);
	warp_mat.at<double>(0, 0) = 1;
	warp_mat.at<double>(0, 1) = 0;
	warp_mat.at<double>(0, 2) = 0;
	warp_mat.at<double>(0, 3) = (1) * std::abs(center.x);
	warp_mat.at<double>(1, 0) = 0;
	warp_mat.at<double>(1, 1) = 1;
	warp_mat.at<double>(1, 2) = 0;
	warp_mat.at<double>(1, 3) = (1) * std::abs(center.y);
	warp_mat.at<double>(2, 0) = 0;
	warp_mat.at<double>(2, 1) = 0;
	warp_mat.at<double>(2, 2) = 1;
	warp_mat.at<double>(2, 3) = (1) * std::abs(center.z);
	//cout << "warp_mat = " << warp_mat << endl;

	// ... e lo applico alle coordinate della fotocamera calcolate prima
	cv::Mat camSrc = cv::Mat::ones(4, 1, cv::DataType<double>::type);
	camSrc.at<double>(0, 0) = _cam3dpose.at<double>(0, 0);
	camSrc.at<double>(0, 1) = _cam3dpose.at<double>(1, 0);
	camSrc.at<double>(0, 2) = _cam3dpose.at<double>(2, 0);
	camSrc.at<double>(0, 3) = 1;
//	cout << "camSrc = " << camSrc << endl;

	// Apply the Affine Transform just found to the src image
	cv::Mat tmp = warp_mat * camSrc;

	cv::Mat camDst = cv::Mat::ones(4, 1, cv::DataType<double>::type);
	camDst.at<double>(0, 0) = tmp.at<double>(0, 0);
	camDst.at<double>(1, 0) = tmp.at<double>(1, 0);
	camDst.at<double>(2, 0) = tmp.at<double>(2, 0);
	camDst.at<double>(3, 0) = 1;

	//
	//cam3DPose.at<double>(0, 0) = camDst.at<double>(0, 0) ;
	//cam3DPose.at<double>(1, 0) = camDst.at<double>(1, 0) ;

	//cout << "camDst = " << camDst << endl;
	return camDst;
}

/**
 *
 */
void MarkerPositionTracker::estimate(cv::Mat &webcamImage) {
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
			// disegno i punti significativi
			cv::drawKeypoints(webcamImage, markerList, webcamImage,
					Scalar::all(-1), DrawMatchesFlags::DEFAULT);

			int dim_x_2D = webcamImage.cols;
			int dim_y_2D = webcamImage.rows;

			WLOG("estimate::calculating next marker");
			bool isChanged = false;
			Point marker;
			bool foundm = nextMarker(markerList, prec_marker, marker,
					isChanged);

			if (foundm) {

				// e lo aggiungo alla scena
				if (addMarkerToScene(marker, webcamImage)) {
					prec_marker = marker;

					// disegno il marker sulla camera
					cv::circle(webcamImage, marker, 8, Scalar(255, 128, 0), -1);
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
					cv::Mat rvec = Mat(Size(3, 1), CV_64F);
					cv::Mat tvec = Mat(Size(3, 1), CV_64F);
					solvePnP(Mat(boardPoints3D), Mat(boardPoints2D),
							cameraMatrix, distortionCoeff, rvec, tvec, false);

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
					cam3DPoint.at<double>(2, 0) =
							(cam3DPointOnlyForZ.at<double>(2, 0)) / 1;

					// faccio la traslazione del punto 3D della camera solidale con l'origine
					// del marker verso l'origine che ho fissato.
					Point3f center3D(m_pattern_WidthReal / 2,
							m_pattern_HeightReal / 2, Zconst);
					cv::Mat cam3DPointTrans = traslateCameraPoseToOrig(center3D,
							cam3DPoint);

					// scrivo l'output sul file di destinazione ...
					//		m_CoordinatesFile << cam3DPoint << endl;//<-------------DEMO

					// ... e su video
					stringstream st_coordinates;
					st_coordinates << cam3DPoint;

					stringstream st_coordinates_t;
					st_coordinates_t << cam3DPointTrans;

					//stringstream st_coordinatesTrans;
					//st_coordinatesTrans << cam3DPointTrans;

					//project the reference frame onto the image
					//WLOG("estimate:: project the reference frame onto the image");
					//projectPoints(framePoints, rvec, tvec, cameraMatrix,
					//		distortionCoeff, imageFramePoints);

					WLOG("estimate:: drawing scene reference");
					cv::putText(webcamImage, st_coordinates.str(),
							cv::Point(pFixed.x + 15, pFixed.y + 5),
							cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
							cv::Scalar(0, 0, 255, 255));

					cv::putText(webcamImage, st_coordinates_t.str(),
							cv::Point(pFixed.x + 15, pFixed.y + 13),
							cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
							cv::Scalar(0, 100, 255, 255));


					//cv::putText(webcamImage, st_coordinatesTrans.str(),
					//		cv::Point(pFixed.x + 5, pFixed.y + 10),
					//		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
					//		cv::Scalar(0, 0, 155, 255));

					if (isChanged) {
						//cv::putText(webcamImage, "MARKER_IS_CHANGED",
						//		cv::Point(pFixed.x + 5, pFixed.y + 30),
						//		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
						//		cv::Scalar(0, 0, 255, 255));
						listCoordinates.push_back(st_coordinates.str());

					}

					//disegno il punto della fotocamera
					cv::line(webcamImage, marker, pFixed, CV_RGB(0, 0, 255), 2);
					cv::circle(webcamImage, pFixed, 8, CV_RGB(263, 83, 0), -1);

					for (int k = 0, j = 0; k < listCoordinates.size(); ++k, j +=
							10) {
						cv::putText(webcamImage, listCoordinates[k],
								cv::Point(pFixed.x + 5, pFixed.y + 20 + j),
								cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5,
								cv::Scalar(0, 255, 0, 255));
					}
					//Point uTrans (uvPointTrans.at<double>(0, 0),uvPointTrans.at<double>(1, 0));
					//cv::line(webcamImage, uTrans, pFixed, CV_RGB(0, 0, 255), 2);
					//cv::circle(webcamImage, uTrans, 8, CV_RGB(203, 183, 50), -1);

					//int offsetX = cam3DPointTrans.at<double>(0, 0);
					//int offsetY = cam3DPointTrans.at<double>(1, 0);
					//set3DPointsReference(boardPoints3D, offsetX, offsetY, m_pattern_WidthReal,
					//			m_pattern_HeightReal);

					found = true;
				}
			}
		}
	} catch (cv::Exception &cvError) {
		found = false;
	} catch (...) {
		found = false;
	}

}
