#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_open_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,tr("Open Image"),
                                    ".",tr("Image Files (*.png *.jpg *.bmp)"));
    qDebug()<<"filenames:"<<fileName;
    // QString转char* qstr.toLatin1().data()
    image = cv::imread(fileName.toLatin1().data()); //fileName.toAscii().data()
    cv::namedWindow(fileName.toLatin1().data(),CV_WINDOW_AUTOSIZE);
    cv::imshow(fileName.toLatin1().data(), image);
}

void MainWindow::on_pushButton_flip_clicked()
{
    cv::flip(image,image,1);

    cv::cvtColor(image,image,CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(image.data),
                        image.cols,image.rows,QImage::Format_RGB888);

    ui->label_img->setPixmap(QPixmap::fromImage(img));
    ui->label_img->resize(ui->label_img->pixmap()->size());
}
