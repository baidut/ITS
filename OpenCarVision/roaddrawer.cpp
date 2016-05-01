#include "roaddrawer.h"

using namespace cv;
using namespace CV_Utils;

bool operator<(const QColor & a, const QColor & b) {
   return a.redF() < b.redF()
       || a.greenF() < b.greenF()
       || a.blueF() < b.blueF()
       || a.alphaF() < b.alphaF();
}

void
RoadDrawer::loadResultFile(QStringList files)
{
    this->roadResFiles = files;
}

void
RoadDrawer::drawResult(int imgID, Mat& rawImg)
{
    // imgID = arg1;
    // rawImg = cv::imread(this->rawFiles.at(arg1).toLatin1().data());

//    ui->label_rawdata->imshow(QString("E:\Sync\my\project\datasets\nicta-RoadImageDatabase\After-Rain\after_rain%05d.tif").arg(arg1));
//    cv::Mat rawImg,gtImg,addingImg;
    cv::Mat gtImg = cv::imread(this->roadResFiles.at(imgID).toLatin1().data(), CV_LOAD_IMAGE_COLOR);
    // already a color image cv::cvtColor ( gtImg, colorGtImg, CV_GRAY2BGR );// bw to rgb

//    for(auto c : colorMap.keys()) {
//        cv::Scalar color1 = CV_Utils::qcolor2scalar(c);
//        cv::Scalar color2 = CV_Utils::qcolor2scalar(colorMap.value(c));

//        gtImg.setTo(color2, maskIfEq(gtImg, color1));
//    }

    QMapIterator<QColor, QColor> c(colorMap);
    while (c.hasNext()) {
        c.next();
        //cout << i.key() << ": " << i.value() << endl;
        cv::Scalar color1 = CV_Utils::qcolor2scalar(c.key());
        cv::Scalar color2 = CV_Utils::qcolor2scalar(c.value());
        gtImg.setTo(color2, maskIfEq(gtImg, color1));
    }

    addWeighted( rawImg, alpha, gtImg, 1.0 - alpha, 0.0, rawImg); // addingImg --> rawImg
}
