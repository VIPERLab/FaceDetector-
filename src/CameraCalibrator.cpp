#include "CameraCalibrator.h"

/**
 *
 */
CameraCalibrator::CameraCalibrator(const string &inputSettingsFile) {
	if (!fileExists(inputSettingsFile.c_str())) {
		stringstream ss;
		ss << "ERROR: The settings file  " << inputSettingsFile
				<< " not exist!";
		throw std::runtime_error(ss.str());
	}

	FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
	if (!fs.isOpened()) {
		stringstream ss;
		ss << "ERROR: Could not open the settings file: \""
				<< inputSettingsFile;
		throw std::runtime_error(ss.str());
	}

	fs["Settings"] >> m_Settings;
	fs.release();                                         // close Settings file

	if (!m_Settings.goodInput) {
		stringstream ss;
		ss << "Invalid input detected. Application stopping. ";
		throw std::runtime_error(ss.str());
	}

	if (m_Settings.m_enableLog) {
		m_enableLog = true;
		m_logFile.open(m_Settings.m_LogFileName.c_str());
	}
	WLOG("Started");

//	mode = m_Settings.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
	mode = CAPTURING;
	prevTimestamp = 0;
}

/**
 *
 */
CameraCalibrator::~CameraCalibrator() {
	WLOG("Stopped");
}

/**
 *
 */
void CameraCalibrator::WLOG(const string& msg) {
	if (m_enableLog) {
		m_logFile << "[" << getNowTime() << "]CameraCalibrator::" << msg
				<< endl;
	}
}

/**
 *
 */
void CameraCalibrator::WLOG(const stringstream& msg) {
	WLOG(msg.str());
}

/**
 *
 */
double CameraCalibrator::computeReprojectionErrors(
		const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs,
		const vector<Mat>& tvecs, const Mat& cameraMatrix,
		const Mat& distCoeffs, vector<float>& perViewErrors) {
	WLOG("computeReprojectionErrors");

	vector < Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int) objectPoints.size(); ++i) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);

		int n = (int) objectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

/**
 *
 */
void CameraCalibrator::calcBoardCornerPositions(Size boardSize,
		float squareSize, vector<Point3f>& corners,
		Settings::Pattern patternType /*= Settings::CHESSBOARD*/) {
	WLOG("calcBoardCornerPositions");

	corners.clear();

	switch (patternType) {
	case Settings::CHESSBOARD:
	case Settings::CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				corners.push_back(
						Point3f(float(j * squareSize), float(i * squareSize),
								0));
		break;

	case Settings::ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(
						Point3f(float((2 * j + i % 2) * squareSize),
								float(i * squareSize), 0));
		break;
	default:
		break;
	}
}

/**
 *
 */
bool CameraCalibrator::runCalibration(Settings& s, Size& imageSize,
		Mat& cameraMatrix, Mat& distCoeffs,
		vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs,
		vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr) {
	WLOG("runCalibration");

	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector < vector<Point3f> > objectPoints(1);
	calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0],
			s.calibrationPattern);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
			cameraMatrix, distCoeffs, rvecs, tvecs,
			s.flag | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

	stringstream ss;
	ss << "Re-projection error reported by calibrateCamera: " << rms << endl;
	WLOG(ss);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs,
			tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

/**
 * Print camera parameters to the output file
 */
void CameraCalibrator::saveCameraParams(Settings& s, Size& imageSize,
		Mat& cameraMatrix, Mat& distCoeffs, const vector<Mat>& rvecs,
		const vector<Mat>& tvecs, const vector<float>& reprojErrs,
		const vector<vector<Point2f> >& imagePoints, double totalAvgErr) {
	WLOG("saveCameraParams");

	FileStorage fs(s.outputFileName, FileStorage::WRITE);

	// controllo se il path del file di output è corretto ...
	if (!fileExists(s.outputFileName.c_str())) {
		stringstream ss;
		ss << "ERROR: The output file " << s.outputFileName
				<< " not exist or the path is not valid!";
		WLOG(ss);
		WLOG("ss");
		throw std::runtime_error(ss.str());
	}

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_Time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nrOfFrames" << (int) std::max(rvecs.size(), reprojErrs.size());
	fs << "image_Width" << imageSize.width;
	fs << "image_Height" << imageSize.height;
	fs << "board_Width" << s.boardSize.width;
	fs << "board_Height" << s.boardSize.height;
	fs << "square_Size" << s.squareSize;

	if (s.flag & CV_CALIB_FIX_ASPECT_RATIO)
		fs << "FixAspectRatio" << s.aspectRatio;

	if (s.flag) {
		sprintf(buf, "flags: %s%s%s%s",
				s.flag & CV_CALIB_USE_INTRINSIC_GUESS ?
						" +use_intrinsic_guess" : "",
				s.flag & CV_CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
				s.flag & CV_CALIB_FIX_PRINCIPAL_POINT ?
						" +fix_principal_point" : "",
				s.flag & CV_CALIB_ZERO_TANGENT_DIST ?
						" +zero_tangent_dist" : "");
		cvWriteComment(*fs, buf, 0);

	}

	fs << "flagValue" << s.flag;

	fs << "Camera_Matrix" << cameraMatrix;
	fs << "Distortion_Coefficients" << distCoeffs;

	fs << "Avg_Reprojection_Error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty()) {
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int) rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int) rvecs.size(); i++) {
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		cvWriteComment(*fs,
				"a set of 6-tuples (rotation vector + translation vector) for each view",
				0);
		fs << "Extrinsic_Parameters" << bigmat;
	}

	if (!imagePoints.empty()) {
		Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(),
				CV_32FC2);
		for (int i = 0; i < (int) imagePoints.size(); i++) {
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "Image_points" << imagePtMat;
	}
}

/**
 *
 */
bool CameraCalibrator::runCalibrationAndSave(Settings& s, Size imageSize,
		Mat& cameraMatrix, Mat& distCoeffs,
		vector<vector<Point2f> > imagePoints) {
	WLOG("runCalibrationAndSave");

	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(s, imageSize, cameraMatrix, distCoeffs,
			imagePoints, rvecs, tvecs, reprojErrs, totalAvgErr);

	stringstream ss;
	ss << (ok ? "Calibration succeeded" : "Calibration failed")
			<< ". avg re projection error = " << totalAvgErr;
	WLOG(ss);

	if (ok)
		saveCameraParams(s, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
				reprojErrs, imagePoints, totalAvgErr);
	return ok;
}

/**
 *
 */
int CameraCalibrator::Calibrate(const cv::Mat& frame, cv::Mat& calib_frame) {
	WLOG("Calibrate");

	bool blinkOutput = false;

	Mat view;
//	view = m_Settings.nextImage();//ATTENZIONE!!!!
	view = m_Settings.nextImage(frame);

	stringstream ss;
	ss << "mode=" << mode << endl;
	ss << "imagePoints.size()=" << imagePoints.size() << endl;
	ss << "(unsigned)s.nrFrames=" << (unsigned) m_Settings.nrFrames << endl;
	WLOG(ss);

	//-----  If no more image, or got enough, then stop calibration and show result -------------
	int nSize = imagePoints.size();
	if (mode == CAPTURING && nSize >= (unsigned) m_Settings.nrFrames) {
		if (runCalibrationAndSave(m_Settings, imageSize, cameraMatrix,
				distCoeffs, imagePoints))
			mode = CALIBRATED;
		else
			mode = DETECTION;
	}

	if (view.empty()) // If no more images then run calibration, save and stop loop.
	{
		if (nSize > 0) {
			runCalibrationAndSave(m_Settings, imageSize, cameraMatrix,
					distCoeffs, imagePoints);
		}
		return mode;
	}

	imageSize = view.size();  // Format input image.
	if (m_Settings.flipVertical)
		flip(view, view, 0);

	vector < Point2f > pointBuf;

	bool found;
	switch (m_Settings.calibrationPattern) // Find feature points on the input format
	{
	case Settings::CHESSBOARD:
		found = findChessboardCorners(view, m_Settings.boardSize, pointBuf,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
						| CV_CALIB_CB_NORMALIZE_IMAGE);
		break;
	case Settings::CIRCLES_GRID:
		found = findCirclesGrid(view, m_Settings.boardSize, pointBuf);
		break;
	case Settings::ASYMMETRIC_CIRCLES_GRID:
		found = findCirclesGrid(view, m_Settings.boardSize, pointBuf,
				CALIB_CB_ASYMMETRIC_GRID);
		break;
	default:
		found = false;
		break;
	}

	if (found)                // If done with success,
	{
		WLOG("improve the found corners' coordinate accuracy for chessboard");

		// improve the found corners' coordinate accuracy for chessboard
		if (m_Settings.calibrationPattern == Settings::CHESSBOARD) {
			Mat viewGray;
			cvtColor(view, viewGray, CV_BGR2GRAY);
			cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1),
					TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		}

		if (mode == CAPTURING && // For camera only take new samples after delay time
				(!m_Settings.inputCapture.isOpened()
						|| clock() - prevTimestamp
								> m_Settings.delay * 1e-3 * CLOCKS_PER_SEC)) {
			imagePoints.push_back(pointBuf);
			prevTimestamp = clock();
			blinkOutput = m_Settings.inputCapture.isOpened();
		}

		// Draw the corners.
		drawChessboardCorners(view, m_Settings.boardSize, Mat(pointBuf), found);
	}

	WLOG("Output text");
	//----------------------------- Output Text ------------------------------------------------
	string msg = (mode == CAPTURING) ? "100/100" :
					mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
	int baseLine = 0;
	Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
	Point textOrigin(view.cols - 2 * textSize.width - 10,
			view.rows - 2 * baseLine - 10);

	if (mode == CAPTURING) {
		if (m_Settings.showUndistorsed)
			msg = format("%d/%d Undist", (int) imagePoints.size(),
					m_Settings.nrFrames);
		else
			msg = format("%d/%d", (int) imagePoints.size(),
					m_Settings.nrFrames);
	}

	putText(view, msg, textOrigin, 1, 1, mode == CALIBRATED ? GREEN : RED);

	if (blinkOutput)
		bitwise_not(view, view);

	//------------------------- Video capture  output  undistorted ------------------------------
	if (mode == CALIBRATED && m_Settings.showUndistorsed) {
		Mat temp = view.clone();
		undistort(temp, view, cameraMatrix, distCoeffs);
	}

	//------------------------------ Show image and check for input commands -------------------
	//if (m_Settings.inputCapture.isOpened() && key == 'g') {
	//	mode = CAPTURING;
	//	imagePoints.clear();
	//}

	calib_frame = view;

	return mode;
}

