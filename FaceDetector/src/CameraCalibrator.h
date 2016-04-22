#include "CameraSettings.h"
#include <stdexcept>

enum {
	DETECTION = 0, CAPTURING = 1, CALIBRATED = 2
};

/**
 *
 */
static void read(const FileNode& node, Settings& x,
		const Settings& default_value) {
	if (node.empty())
		x = default_value;
	else
		x.read(node);
}

const Scalar RED(0, 0, 255), GREEN(0, 255, 0);

class CameraCalibrator {
public:
	CameraCalibrator(const string &inputSettingsFile);
	~CameraCalibrator();
	int Calibrate(const cv::Mat& frame, cv::Mat& calib_frame);
private:
	void WLOG(const string& msg);
	void WLOG(const stringstream& msg);

	static void read(const FileNode& node, Settings& x,
			const Settings& default_value = Settings());

	bool runCalibrationAndSave(Settings& s, Size imageSize, Mat& cameraMatrix,
			Mat& distCoeffs, vector<vector<Point2f> > imagePoints);

	double computeReprojectionErrors(
			const vector<vector<Point3f> >& objectPoints,
			const vector<vector<Point2f> >& imagePoints,
			const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const Mat& cameraMatrix, const Mat& distCoeffs,
			vector<float>& perViewErrors);

	void calcBoardCornerPositions(Size boardSize, float squareSize,
			vector<Point3f>& corners,
			Settings::Pattern patternType /*= Settings::CHESSBOARD*/);

	bool runCalibration(Settings& s, Size& imageSize, Mat& cameraMatrix,
			Mat& distCoeffs, vector<vector<Point2f> > imagePoints,
			vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs,
			double& totalAvgErr);

// Print camera parameters to the output file
	void saveCameraParams(Settings& s, Size& imageSize, Mat& cameraMatrix,
			Mat& distCoeffs, const vector<Mat>& rvecs, const vector<Mat>& tvecs,
			const vector<float>& reprojErrs,
			const vector<vector<Point2f> >& imagePoints, double totalAvgErr);

private:
	Settings m_Settings;
	bool m_enableLog;
	std::ofstream m_logFile;

	vector<vector<Point2f> > imagePoints;
	Mat cameraMatrix, distCoeffs;
	Size imageSize;
	int mode;
	clock_t prevTimestamp;
};
