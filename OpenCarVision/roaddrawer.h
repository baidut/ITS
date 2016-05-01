#ifndef ROADDRAWER_H
#define ROADDRAWER_H

#include "common.h"
#include <QColor>
#include <QMap>
#include <QPair>
#include "cv_utils.h"

#include <QDebug>

// http://stackoverflow.com/questions/32512125/is-it-possible-to-store-qcolor-in-a-qmap-as-key
bool operator<(const QColor & a, const QColor & b);

class RoadDrawer
{
public:
    RoadDrawer() : alpha(0.5){ //, colorPair(20)
//        qDebug() << colorPair.size();
//        cv::Scalar color = cv::Scalar(0,0,0);
//        for(int i = 0 ; i < 20; i++) {
//            colorPair.push_back(qMakePair(color,color));
//        }
//        qDebug() << colorPair.size();
        // QVector<QPair<cv::Scalar(0,0,0), cv::Scalar(0,0,0)> >(0)
        // qDebug()<<CV_Utils::scalar2qcolor(cv::Scalar(255,255,255));
    }

    void    loadResultFile(QStringList files);
    void    drawResult(int imgID, cv::Mat &rawImg);
    void    setAlpha(double a) {alpha = a; }
    void    setColorMap(QColor color1, QColor color2) {
        colorMap[color1] = color2;
    }

private:
    QStringList roadGtFiles; // GT
    QStringList roadResFiles; // detection results

    double alpha;
    QMap<QColor, QColor> colorMap;

    //std::string basePath;
    //int curMode;
    //int imgNum;
};

#endif // ROADDRAWER_H
