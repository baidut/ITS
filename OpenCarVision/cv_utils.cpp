
#include "cv_utils.h"
#include <QDebug>

using namespace CV_Utils;

cv::Mat
CV_Utils::imread(QString filename) {
    return cv::imread(filename.toLatin1().data());
}

//cv::Mat
//CV_Utils::imread(const char* filename) {
//    cv::Mat image;
//    image = cv::imread(filename,CV_LOAD_IMAGE_COLOR);
//    Q_ASSERT(image.data!=0);
////    if(!image.data){
////      throw invalid_argument(string(filename)+" can not  be open or read in!\t");
////    }
//    return image;
//}

cv::Mat
CV_Utils::maskIfEq(cv::Mat image, cv::Scalar color)
{ // mask if eq Scalar(b,g,r)
   cv::Mat mask;
   cv::inRange(image, color, color, mask);
   return mask;
}

cv::Scalar
CV_Utils::qcolor2scalar(QColor color)
{
    int r,g,b;
    color.getRgb(&r, &g, &b);
    // OPENCV-BGR
    return cv::Scalar(b,g,r);
}


QColor
CV_Utils::scalar2qcolor(cv::Scalar color)
{
    return QColor(color[2],color[1],color[0]);
}
