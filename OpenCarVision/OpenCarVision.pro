#-------------------------------------------------
#
# Project created by QtCreator 2015-03-22T21:05:34
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = OpenCarVision
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH +=  D:/AcademicLearning/OpenCV/mingw/include/opencv \
                D:/AcademicLearning/OpenCV/mingw/include/opencv2 \
                D:/AcademicLearning/OpenCV/mingw/include
LIBS += D:/AcademicLearning/OpenCV/mingw/lib/libopencv_calib3d2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_contrib2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_core2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_features2d2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_flann2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_gpu2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_highgui2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_imgproc2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_legacy2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_ml2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_objdetect2410.dll.a \
        D:/AcademicLearning/OpenCV/mingw/lib/libopencv_video2410.dll.a

