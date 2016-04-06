#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "datasetloader.h"

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

    void on_pushButton_upsideDown_clicked();

    void on_pushButton_slt_clicked();

    void on_action_openFile_triggered();

    void on_pushButton_apply_clicked();

    void on_pushButton_histogram_clicked();

    void on_pushButton_sobel_clicked();

    void on_pushButton_2inch_clicked();

    void on_pushButton_findLines_clicked();

    void on_pushButton_display_clicked();

    void on_pushButton_resume_clicked();

    void on_pushButton_detectLanes_clicked();

    void on_pushButton_threshold_clicked();

    void on_pushButton_loadCamvid_clicked();

    void on_spinBox_camvidNo_valueChanged(int arg1);

    void on_pushButton_play_clicked(bool checked);

    void on_pushButton_loadCamvid4_clicked();

    void on_pushButton_loadKittiTrain_clicked();

    void on_pushButton_loadKittiTest_clicked();

    void on_pushButton_loadAfterRain_clicked();

    void on_pushButton_s2_afterrain_clicked();

    void on_spinBox_s2_no_valueChanged(int arg1);

    void on_pushButton_s2_kitti_clicked();

    void on_spinBox_nFrameOfDataset_valueChanged(int arg1);

    void on_actionLoad_triggered();

    void on_comboBox_datasetName_currentIndexChanged(int index);

    void on_horizontalSlider_plendAlpha_valueChanged(int value);

signals:
    void imageChanged();
    void imageProcessed();

private:
    QStringList datasetPathList;
    Ui::MainWindow *ui; 
    QString fileName;
    cv::Mat image;
    cv::Mat imageROI;
    cv::Mat imgProc;


    DatasetLoader loader;
    QStringList files;
    QStringList rawFiles;
    QStringList roadGtFiles;
    // to be removed
    QString raw;
    QString gt;
    QString res;
    void load_dataset(int arg1, QString raw, QString gt, QString res);
};

#endif // MAINWINDOW_H
