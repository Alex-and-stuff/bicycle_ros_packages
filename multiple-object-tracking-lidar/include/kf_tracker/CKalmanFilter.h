#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

/*
// Build a Class similar (on top of) to the openCV KalmanFilter class
// Class methods defined at CKalmanFilter.cpp
// Achive Prediction from previous state
// Achive Update with current measurement
*/

class CKalmanFilter
{
public:
	CKalmanFilter(vector<Vec2f>);
	~CKalmanFilter();
	vector<Vec2f> predict();
	vector<Vec2f> update(vector<Vec2f>);

	KalmanFilter* kalman;
	vector<Vec2f> prevResult;

};