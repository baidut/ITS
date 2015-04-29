#include <iostream>
#include <string.h>
#include <fstream>

#include "laneDetection.h"
#include "CKalmanFilter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
	laneDetection detect; // object of laneDetection class

	string ippath = "./images/"; // Relative path of the images
	string oppath = "./output/"; // Relative path of the output folder
	string imname;
	ifstream imageNames ("imNames.txt"); // Name of the text file which containes name of all the images.
	getline(imageNames,imname); //getting the name of the image
    
	ippath += imname;
	Mat img1 = imread(ippath,0); // Read the image
	resize(img1,img1,Size(detect._width,detect._height)); // Resizing the image (only for display purposes)
	
	detect.LMFiltering(img1); // Filtering to detect Lane Markings
	vector<Vec2f> lines = detect.houghTransform(); // Hough Transform
	Mat imgFinal = detect.drawLines(img1, lines, imname); // draw final Lane Markings on the original image
	oppath += imname;
	imwrite(oppath,imgFinal); // writing the final result to a folder

	// Till we consider all the images
	while ( getline (imageNames,imname) ){
		ippath = "./images/"; // Relative path of the images
		oppath = "./output/"; // Relative path of the output folder
		ippath += imname;

		Mat img2 = imread(ippath,0); // Read the image
		resize(img2,img2,Size(detect._width,detect._height)); // Resizing the image (only for display purposes)
		
		detect.LMFiltering(img2); // Filtering to detect Lane Markings
		vector<Vec2f> lines2 = detect.houghTransform(); // Hough Transform
		
		
		// if lanes are not detected, then use the Kalman Filter prediction
		if (lines2.size() < 2) {
			imgFinal = detect.drawLines(img2,lines, imname); // draw final Lane Markings on the original image for display
			oppath += imname;
			imwrite(oppath,imgFinal); 
			continue;
		}
		
		///// Kalman Filter to predict the next state
		CKalmanFilter KF2(lines); // Initialization 
		vector<Vec2f> pp = KF2.predict(); // Prediction

		vector<Vec2f> lines2Final = KF2.update(lines2); // Correction
		lines = lines2Final; // updating the model
		imgFinal = detect.drawLines(img2,lines2, imname); // draw final Lane Markings on the original image
		/////
		
		oppath += imname;
		imwrite(oppath,imgFinal); // writing the final result to a folder

	}

}