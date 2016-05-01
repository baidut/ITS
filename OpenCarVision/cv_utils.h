#ifndef CV_UTILS
#define CV_UTILS

#include <QString>
#include <QColor>

#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>

namespace CV_Utils {

cv::Mat imread(QString filename);
cv::Mat maskIfEq(cv::Mat image, cv::Scalar color);
cv::Scalar qcolor2scalar(QColor color);
QColor scalar2qcolor(cv::Scalar color);
}

#endif // CV_UTILS
