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

#ifndef QLABELPLUS_H
#define QLABELPLUS_H

#include <QWidget>
#include <QLabel>
#include <opencv2/core/core.hpp>

/**
 * @brief The QLabelPlus class
 */
class QLabelPlus : public QLabel
{
    Q_OBJECT
public:
    explicit QLabelPlus(QWidget *parent = 0);
    explicit QLabelPlus( const QString& text="", QWidget* parent=0 );
    ~QLabelPlus();

    void imshow(cv::Mat image);
    void imshow(QString imageUrl);
    void imshow(char*   imageUrl);

signals:
    void clicked();
public slots:
    void mousePressEvent(QMouseEvent* event);
};

#endif // QLABELPLUS_H
