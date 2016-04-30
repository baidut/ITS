#-------------------------------------------------
#
# Project created by QtCreator 2015-03-22T21:05:34
#
#-------------------------------------------------

QT       += core gui
QT += xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenCarVision
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qlabelplus.cpp \
    datasetloader.cpp \
    roaddrawer.cpp \
    cardrawer.cpp \
    cv_utils.cpp

HEADERS  += mainwindow.h \
    linefinder.h \
    histogram.h \
    edgedetector.h \
    LaneDetect.h \
    qlabelplus.h \
    datasetloader.h \
    dataset.h \
    ui_utils.h \
    roaddrawer.h \
    cardrawer.h \
    common.h \
    cv_utils.h

FORMS    += mainwindow.ui

INCLUDEPATH +=  D:/OpenCV/include/opencv \
                D:/OpenCV/include/opencv2 \
                D:/OpenCV/include
#LIBS += D:/OpenCV/mingw/lib/libopencv_calib3d2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_contrib2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_core2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_features2d2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_flann2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_gpu2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_highgui2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_imgproc2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_legacy2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_ml2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_objdetect2410.dll.a \
#        D:/OpenCV/mingw/lib/libopencv_video2410.dll.a

LIBS += D:/OpenCV/lib/libopencv_calib3d310.dll.a \
        D:/OpenCV/lib/libopencv_core310.dll.a \
        D:/OpenCV/lib/libopencv_features2d310.dll.a \
        D:/OpenCV/lib/libopencv_flann310.dll.a \
        D:/OpenCV/lib/libopencv_highgui310.dll.a \
        D:/OpenCV/lib/libopencv_imgproc310.dll.a \
        D:/OpenCV/lib/libopencv_ml310.dll.a \
        D:/OpenCV/lib/libopencv_objdetect310.dll.a \
        D:/OpenCV/lib/libopencv_video310.dll.a \
        D:/OpenCV/lib/libopencv_imgcodecs310.dll.a # fix bug of undefined imread an imwrite, see http://www.opencv.org.cn/forum.php?mod=viewthread&tid=33294

RESOURCES += \
    images.qrc

CONFIG += c++11
