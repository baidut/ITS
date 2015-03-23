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

    void on_pushButton_reduceColor_clicked();

signals:
    void imageChanged();

private:
    Ui::MainWindow *ui; 
    cv::Mat image;
};

#endif // MAINWINDOW_H
