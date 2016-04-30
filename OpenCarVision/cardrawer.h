#ifndef CARDRAWER_H
#define CARDRAWER_H

#include "common.h"
#include "cv_utils.h"

// QTL vs STL
#define STD_CPP

#ifdef STD_CPP
    #include <string>
    #include <vector>
    #include <iostream>
    #include <fstream>
#else
    #include <QString>
    #define string QString
    #include <QVector>
    #define vector QVector
#endif

class CarInfo
{
public:
    int x1 = 0;
    int y1 = 0;
    int x2 = 0;
    int y2 = 0;
    int type = 0;

    CarInfo(int x1, int y1, int x2, int y2, int type)
    {
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        this->type = type;
    }

    CarInfo(int x1, int y1, int x2, int y2)
    {
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
    }

};

class CarDrawer
{
public:
    CarDrawer(std::string gt_file, std::string basePath, int file_mode);

    void    loadResultFile(const std::string gt_file);
    int     getImgNum() { return imgNum; }
    cv::Mat drawResult(int imgID);
    std::string  getCarType(int typeID);

private:
    std::vector<std::string> imgList;
    std::vector<std::vector<CarInfo> > resultList;
    std::string basePath;
    int curMode;
    int imgNum;
};

#endif // CARDRAWER_H
