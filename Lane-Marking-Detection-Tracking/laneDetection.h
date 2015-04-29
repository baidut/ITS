#include <iostream>
#include <vector>
#include <fstream>
#include <string>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;

class laneDetection{

public:
	laneDetection();
	~laneDetection();
	void LMFiltering(Mat);
	vector<Vec2f> houghTransform();
	Mat drawLines(Mat,vector<Vec2f>, string);
	int _width, _height;

protected:
	bool findIntersection(vector<Point>, Point&);
	vector<Point2f> ransac(vector<Point2f>);
	void visualize(Mat);
	vector<Vec2f> _lines;
	ofstream myfile;
	Mat _detectedEdges;
	int _LMWidth; // Lane Mark Width
	int _thres;
	float _rho, _theta,_ransacThres, _houghThres;
	
};