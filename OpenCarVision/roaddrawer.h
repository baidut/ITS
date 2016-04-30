#ifndef ROADDRAWER_H
#define ROADDRAWER_H

#include "common.h"
#include <QColor>
#include <QVector>
#include <QPair>
#include "cv_utils.h"

#include <QDebug>

class RoadDrawer
{
public:
    RoadDrawer() : alpha(0.5), colorPair(0){ //, colorPair(20)
//        qDebug() << colorPair.size();
//        cv::Scalar color = cv::Scalar(0,0,0);
//        for(int i = 0 ; i < 20; i++) {
//            colorPair.push_back(qMakePair(color,color));
//        }
//        qDebug() << colorPair.size();
        // QVector<QPair<cv::Scalar(0,0,0), cv::Scalar(0,0,0)> >(0)
    }

    void    loadResultFile(QStringList files);
    void    drawResult(int imgID, cv::Mat &rawImg);
    void    setAlpha(double a) {alpha = a; }
    void    addColorPair(QColor color1, QColor color2) {
        colorPair.push_back(qMakePair(color1,color2));
    }

private:
    QStringList roadGtFiles; // GT
    QStringList roadResFiles; // detection results

    double alpha;
    QVector<QPair<QColor, QColor> > colorPair;

    //std::string basePath;
    //int curMode;
    //int imgNum;
};

#endif // ROADDRAWER_H
