#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QInputDialog>

#include <linefinder.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this,SIGNAL(imageChanged()),this,SLOT(on_image_changed()));
    connect(this,SIGNAL(imageProcessed()),this,SLOT(on_image_processed()));
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
    QImage img;
    cv::Mat imageRGB;
    if(1==image.channels()){
        img = QImage((const unsigned char*)(image.data),
                            image.cols,image.rows,QImage::Format_Indexed8);
    }
    else{
        cv::cvtColor(image,imageRGB,CV_BGR2RGB);
        img = QImage((const unsigned char*)(imageRGB.data),
                            image.cols,image.rows,QImage::Format_RGB888);
    }
    ui->label_img->setPixmap(QPixmap::fromImage(img));
    ui->label_img->resize(ui->label_img->pixmap()->size());
}

void MainWindow::on_image_processed(){
    QImage img;
    cv::Mat imageRGB;
    if(1==imgProc.channels()){
        img = QImage((const unsigned char*)(imgProc.data),
                            imgProc.cols,imgProc.rows,QImage::Format_Indexed8);
    }
    else{
        cv::cvtColor(imgProc,imageRGB,CV_BGR2RGB);
        img = QImage((const unsigned char*)(imageRGB.data),
                            imgProc.cols,imgProc.rows,QImage::Format_RGB888);
    }
    ui->label_imgProc->setPixmap(QPixmap::fromImage(img));
    ui->label_imgProc->resize(ui->label_img->pixmap()->size());
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
        // 如果图像没有在行尾填补像素，则是连续的一维数组，遍历更快。 由于可能用到二维空间信息，这里不采用reshape降低维度
        if(image.isContinuous()){nc = nc*nl;nl=1;}
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

void MainWindow::on_pushButton_sharpen_clicked()
{
    // 这里必须再构建一个图像先
    // 锐化是基于灰度图片的，如果不是，先转为灰度图
    cv::Mat result;
    result.create(image.rows,image.cols,image.type()); // 书中错误 result.create(image.size(),image.type());
    for(int j=1;j<image.rows-1;j++){ // 外循环是列，这样效率高
        const uchar* previous = image.ptr<const uchar>(j-1);
        const uchar* current = image.ptr<const uchar>(j);
        const uchar* next = image.ptr<const uchar>(j+1);
        uchar* output = result.ptr<uchar>(j);
        for(int i=1;i<image.cols-1;i++){
            *output++= cv::saturate_cast<uchar>(
                        5*current[i]-current[i-1]
                        -current[i+1]-previous[i]-next[i]);
        }
    }
    result.row(0).setTo(cv::Scalar(0));
    result.row(result.rows-1).setTo(cv::Scalar(0));
    result.col(0).setTo(cv::Scalar(0));
    result.col(result.cols-1).setTo(cv::Scalar(0));
    image = result.clone(); // 深复制
    emit imageChanged();
}

void MainWindow::on_pushButton_gray_clicked()
{
    cv::cvtColor(image,image,CV_BGR2GRAY);
    emit imageChanged();
}

void MainWindow::on_pushButton_sharpen2D_clicked()
{
    cv::Mat kernel(3,3,CV_32F,cv::Scalar(0));
    kernel.at<float>(1,1) = 5.0;
    kernel.at<float>(0,1) = -1.0;
    kernel.at<float>(2,1) = -1.0;
    kernel.at<float>(1,0) = -1.0;
    kernel.at<float>(1,2) = -1.0;
    cv::filter2D(image,image,image.depth(),kernel);
    emit imageChanged();
}

void MainWindow::on_pushButton_blur_clicked()
{
    cv::blur(image,image,cv::Size(5,5));
    emit imageChanged();
}

void MainWindow::on_pushButton_gaussianBlur_clicked()
{
    cv::GaussianBlur(image,image,cv::Size(5,5),1.5);
    emit imageChanged();
}

void MainWindow::on_pushButton_medianBlur_clicked()
{ // 多点几次中值滤波，生成油画特效！
    cv::medianBlur(image,image,5);
    emit imageChanged();
}

void MainWindow::on_pushButton_canny_clicked()
{
    cv::Canny(image,imgProc,ui->doubleSpinBox_threshold1->value(),ui->doubleSpinBox_threshold2->value());
    emit imageProcessed();
}

void MainWindow::on_pushButton_hough_clicked()
{
    LineFinder finder;
    finder.setLineLengthAndGap(100,20);
    finder.setMinVote(80);

    cv::Mat contours;
    cv::Canny(image,contours,125,350);

    std::vector<cv::Vec4i>lines = finder.findLines(contours);
    finder.drawDetectedLines(image);
    cv::namedWindow("Dectected Lines with HoughP");
    cv::imshow("Detected Lines with HoughP",image);
    //emit imageChanged();
}
