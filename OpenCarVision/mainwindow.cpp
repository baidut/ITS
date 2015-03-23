#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QInputDialog>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this,SIGNAL(imageChanged()),this,SLOT(on_image_changed()));
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

    emit imageChanged();
    //cv::namedWindow(fileName.toLatin1().data(),CV_WINDOW_AUTOSIZE);
    //cv::imshow(fileName.toLatin1().data(), image);
}

void MainWindow::on_pushButton_flip_clicked()
{
    //要求image已经载入
    cv::flip(image,image,1);
    emit imageChanged();
}

void MainWindow::on_pushButton_salt_clicked()
{
    //要求image已经载入
    bool ok;
    int N = QInputDialog::getInt(this,tr("SALT n"),tr("Please input:"),
                                     1000,0,image.cols*image.rows,1,&ok); // 旧版本可能是getInteger
    if (ok){
        for(int k = 0; k < N; k++){
            // 需要进度条
            int i = rand()%image.cols;
            int j = rand()%image.rows;

            switch(image.channels()){
            case 1: image.at<uchar>(j,i)=255; break;
            case 3: image.at<cv::Vec3b>(j,i)[0] =
                    image.at<cv::Vec3b>(j,i)[1] =
                    image.at<cv::Vec3b>(j,i)[2] = 255;break;
            default: break;
            }
        }
        emit imageChanged();
    }
}

void MainWindow::on_image_changed(){
    cv::Mat imageRGB;
    cv::cvtColor(image,imageRGB,CV_BGR2RGB);
    QImage img = QImage((const unsigned char*)(imageRGB.data),
                        image.cols,image.rows,QImage::Format_RGB888);

    ui->label_img->setPixmap(QPixmap::fromImage(img));
    ui->label_img->resize(ui->label_img->pixmap()->size());
}

void MainWindow::on_pushButton_reduceColor_clicked()
{
    bool ok;
    int div = QInputDialog::getInt(this,tr("DIV"),tr("Please input:"),
                                     64,1,128,1,&ok); // 旧版本可能是getInteger
    if (ok){
        int nl = image.rows;
        int nc = image.cols*image.channels();
        // 以下写法从速度考虑
        if( 0 == ( div & (div - 1) )){ // 快速判断2的整数幂 http://blog.csdn.net/hackbuteer1/article/details/6681157
            int x = 0,tmp = div;
            while(tmp>1){tmp >>= 1;x++;} // x = log2(div)
            uchar mask = 0xFF << x;
            for(int j=0;j<nl;j++) {
                uchar* data = image.ptr<uchar>(j);
                for(int i=0;i<nc;i++)
                    data[i] = (data[i]&mask) + div/2;
            }
        }
        else{
            for(int j=0;j<nl;j++) {
                uchar* data = image.ptr<uchar>(j);
                for(int i=0;i<nc;i++){
                    data[i] = data[i]/div*div + div/2;
                    // 等价： = data[i] - data[i]%div + div/2 较慢
                }
            }
        }
        emit imageChanged();
    }
}
