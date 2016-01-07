/*
* Copyright (c) 2015-2016 Zhenqiang.YING yingzhenqiang-at-gmail-dot-com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "qlabelplus.h"
#include <opencv2/imgproc/imgproc.hpp> // cvtColor
#include <opencv2/highgui/highgui.hpp> // imread

QLabelPlus::QLabelPlus(QWidget* parent)
    : QLabel(parent)
{
}

QLabelPlus::QLabelPlus(const QString& text, QWidget* parent)
    : QLabel(parent)
{
    setText(text);
}

QLabelPlus::~QLabelPlus()
{
}

void QLabelPlus::mousePressEvent(QMouseEvent* event)
{
    emit clicked();
}

/**
 * @see Clickable QLabel https://wiki.qt.io/Clickable_QLabel
 * @see Qt Mouse Event on QLabel https://www.youtube.com/watch?v=d0CDMtfefB4
 */

void QLabelPlus::imshow(cv::Mat image){
    QImage img;
    cv::Mat imageRGB;
    // qDebug("Channels:%d",image.channels());

    if(1==image.channels()){
        img = QImage((const unsigned char*)(image.data),
                            image.cols,image.rows,image.cols,QImage::Format_Indexed8);
        //qDebug("黑白图片");
    }
    else{
        cv::cvtColor(image,imageRGB,CV_BGR2RGB);
        img = QImage((const unsigned char*)(imageRGB.data),
                            image.cols,image.rows,image.cols*image.channels(),QImage::Format_RGB888);
        //qDebug("彩色图片");
    }
    // img = img.scaled(this->size()); // 缩放有bug 不进行缩放
    // img = img.scaled(this->size(),Qt::KeepAspectRatio);
    this->clear(); // 先清理
    this->setPixmap(QPixmap::fromImage(img));
    this->resize(this->pixmap()->size());
}

void QLabelPlus::imshow(char* imageUrl){
    if(imageUrl == NULL||strcmp(imageUrl,"")==0)this->clear();
    this->imshow(cv::imread(imageUrl));
}

void QLabelPlus::imshow(QString imageUrl){
    this->imshow(cv::imread(imageUrl.toLatin1().data()));
}
