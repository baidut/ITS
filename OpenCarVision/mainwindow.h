#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_open_clicked();
    void on_pushButton_flip_clicked();
    void on_pushButton_salt_clicked();
    void on_image_changed();
    void on_image_processed();

    void on_pushButton_reduceColor_clicked();

    void on_pushButton_sharpen_clicked();

    void on_pushButton_gray_clicked();

    void on_pushButton_sharpen2D_clicked();

    void on_pushButton_blur_clicked();

    void on_pushButton_gaussianBlur_clicked();

    void on_pushButton_medianBlur_clicked();

    void on_pushButton_canny_clicked();

    void on_pushButton_hough_clicked();

    void on_pushButton_upsideDown_clicked();

    void on_pushButton_slt_clicked();

signals:
    void imageChanged();
    void imageProcessed();

private:
    Ui::MainWindow *ui; 
    cv::Mat image;
    cv::Mat imgProc;
};

#endif // MAINWINDOW_H
