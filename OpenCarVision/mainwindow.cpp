#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QInputDialog>

#include "linefinder.h"
#include "histogram.h"

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

void MainWindow::on_pushButton_flip_clicked()
{
    //要求image已经载入
    cv::flip(image,imgProc,1);
    emit imageProcessed();
}

void MainWindow::on_pushButton_upsideDown_clicked()
{
    //要求image已经载入
    cv::flip(image,imgProc,0);
    emit imageProcessed();
}

void MainWindow::on_pushButton_salt_clicked()
{
    //要求image已经载入
    bool ok;
    int N = QInputDialog::getInt(this,tr("SALT n"),tr("Please input:"),
                                     1000,0,image.cols*image.rows,1,&ok); // 旧版本可能是getInteger
    imgProc = image.clone();
    if (ok){
        for(int k = 0; k < N; k++){
            // 需要进度条
            int i = rand()%imgProc.cols;
            int j = rand()%imgProc.rows;

            switch(imgProc.channels()){
            case 1: imgProc.at<uchar>(j,i)=255; break;
            case 3: imgProc.at<cv::Vec3b>(j,i)[0] =
                    imgProc.at<cv::Vec3b>(j,i)[1] =
                    imgProc.at<cv::Vec3b>(j,i)[2] = 255;break;
            default: break;
            }
        }
        emit imageProcessed();
    }
}

void display_image(cv::Mat image,QLabel* label){
    if(label){
        QImage img;
        cv::Mat imageRGB;
        if(1==image.channels()){
            img = QImage((const unsigned char*)(image.data),
                                image.cols,image.rows,image.cols*image.channels(),QImage::Format_Indexed8);
            //qDebug("黑白图片");
        }
        else{
            cv::cvtColor(image,imageRGB,CV_BGR2RGB);
            img = QImage((const unsigned char*)(imageRGB.data),
                                image.cols,image.rows,image.cols*image.channels(),QImage::Format_RGB888);
            //qDebug("彩色图片");
        }
        //img = img.scaled(label->size()); // 缩放有bug 不进行缩放
        label->clear(); // 先清理
        label->setPixmap(QPixmap::fromImage(img));
        label->resize(label->pixmap()->size());
    }
    else{
        cv::imshow("Figure",image);
    }
}

void MainWindow::on_image_changed(){
    cv::Mat imgResized;
    cv::resize(image,imgResized,cv::Size(100,100)); //利用OpenCV的缩放代替QT的缩放解决缩放bug
    display_image(imgResized,ui->label_img);
}

void MainWindow::on_image_processed(){
    display_image(imgProc,ui->label_imgProc);
}

void MainWindow::on_pushButton_reduceColor_clicked()
{
    bool ok;
    int div = QInputDialog::getInt(this,tr("DIV"),tr("Please input:"),
                                     64,1,128,1,&ok); // 旧版本可能是getInteger
    imgProc = image.clone();
    if (ok){
        int nl = imgProc.rows;
        int nc = imgProc.cols*imgProc.channels();
        // 以下写法从速度考虑
        // 如果图像没有在行尾填补像素，则是连续的一维数组，遍历更快。 由于可能用到二维空间信息，这里不采用reshape降低维度
        if(imgProc.isContinuous()){nc = nc*nl;nl=1;}
        if( 0 == ( div & (div - 1) )){ // 快速判断2的整数幂 http://blog.csdn.net/hackbuteer1/article/details/6681157
            int x = 0,tmp = div;
            while(tmp>1){tmp >>= 1;x++;} // x = log2(div)
            uchar mask = 0xFF << x;
            for(int j=0;j<nl;j++) {
                uchar* data = imgProc.ptr<uchar>(j);
                for(int i=0;i<nc;i++)
                    data[i] = (data[i]&mask) + div/2;
            }
        }
        else{
            for(int j=0;j<nl;j++) {
                uchar* data = imgProc.ptr<uchar>(j);
                for(int i=0;i<nc;i++){
                    data[i] = data[i]/div*div + div/2;
                    // 等价： = data[i] - data[i]%div + div/2 较慢
                }
            }
        }
        emit imageProcessed();
    }
}

void MainWindow::on_pushButton_sharpen_clicked()
{
    // 这里必须再构建一个图像先
    // 锐化是基于灰度图片的，如果不是，先转为灰度图
    imgProc.create(image.rows,image.cols,image.type()); // 书中错误 imgProc.create(image.size(),image.type());
    for(int j=1;j<image.rows-1;j++){ // 外循环是列，这样效率高
        const uchar* previous = image.ptr<const uchar>(j-1);
        const uchar* current = image.ptr<const uchar>(j);
        const uchar* next = image.ptr<const uchar>(j+1);
        uchar* output = imgProc.ptr<uchar>(j);
        for(int i=1;i<image.cols-1;i++){
            *output++= cv::saturate_cast<uchar>(
                        5*current[i]-current[i-1]
                        -current[i+1]-previous[i]-next[i]);
        }
    }
    imgProc.row(0).setTo(cv::Scalar(0));
    imgProc.row(imgProc.rows-1).setTo(cv::Scalar(0));
    imgProc.col(0).setTo(cv::Scalar(0));
    imgProc.col(imgProc.cols-1).setTo(cv::Scalar(0));
    emit imageProcessed();
}

void MainWindow::on_pushButton_gray_clicked()
{
    cv::cvtColor(image,imgProc,CV_BGR2GRAY);
    emit imageProcessed();
}

void MainWindow::on_pushButton_sharpen2D_clicked()
{
    cv::Mat kernel(3,3,CV_32F,cv::Scalar(0));
    kernel.at<float>(1,1) = 5.0;
    kernel.at<float>(0,1) = -1.0;
    kernel.at<float>(2,1) = -1.0;
    kernel.at<float>(1,0) = -1.0;
    kernel.at<float>(1,2) = -1.0;
    cv::filter2D(image,imgProc,image.depth(),kernel);
    emit imageProcessed();
}

void MainWindow::on_pushButton_blur_clicked()
{
    cv::blur(image,imgProc,cv::Size(5,5));
    emit imageProcessed();
}

void MainWindow::on_pushButton_gaussianBlur_clicked()
{
    cv::GaussianBlur(image,imgProc,cv::Size(5,5),1.5);
    emit imageProcessed();
}

void MainWindow::on_pushButton_medianBlur_clicked()
{ // 多点几次中值滤波，生成油画特效！
    cv::medianBlur(image,imgProc,5);
    emit imageProcessed();
}

void MainWindow::on_pushButton_canny_clicked()
{
    // canny阈值越小，收集到的特征点越多，从大阈值调小，感觉就像画画一样！
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

void MainWindow::on_pushButton_slt_clicked()
{
    // 静态阈值 处理灰度图！注意先要转为灰度图
    // DLD特性是针对一条车道线的，因此该算法写的有问题，应当改成一定范围内搜索，而不是整行的AverageL和AverageR
    // 改为采用核滤波的方式-属于图像增强，而不是提取特征，找最匹配DLD模板的区域
    int Th = ui->spinBox_sltTh->value();
    int Range = ui->spinBox_sltRange->value();
    // Range要比白线宽度宽才合理，因此检索车道线边界的时候也可以采用此Range，提高阈值，使得只有中心被匹配
    // Range应当由远到近不断增大才合适
    imgProc.create(image.rows,image.cols,image.type());
    for(int j=1;j<image.rows-1;j++){
        const uchar* current = image.ptr<const uchar>(j);
        uchar* output = imgProc.ptr<uchar>(j);

        int sumL,sumR;
        for(int i=Range;i<image.cols-1-Range;i++){ // i 表征当前像素左边像素有几个
            sumL=sumR=0;
            for(int k=1;k<=Range;k++){
                sumL += current[i-k];
                sumR += current[i+k];
            }
            if( (current[i] > sumL/Range + Th)
                && (current[i] > sumR/Range + Th ) // 共image.cols个，当前1个，左边i个
              ){ // Ip > AverageR + Th && Ip > AverageL + Th
                // 找到了一个特征点！
                if(ui->checkBox_extendedSlt->isChecked()){


                }
                output[i] = 255; // 左边车道线
            }
            else output[i] = 0;
        }
    }
    emit imageProcessed();
}

void MainWindow::on_action_openFile_triggered()
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

void MainWindow::on_pushButton_apply_clicked()
{
    image = imgProc.clone(); // 深复制
    emit imageChanged();
}

void MainWindow::on_pushButton_histogram_clicked()
{
    // The histogram object
        Histogram1D h;

        // Compute the histogram
        // cv::MatND histo= h.getHistogram(image);
        // Loop over each bin
        // for (int i=0; i<256; i++)cout << "Value " << i << " = " << histo.at<float>(i) << endl;

        // Display a histogram as an image
        cv::namedWindow("Histogram");
        cv::imshow("Histogram",h.getHistogramImage(image));

        // creating a binary image by thresholding at the valley
        cv::Mat thresholded;
        cv::threshold(image,thresholded,60,255,cv::THRESH_BINARY);

        // Display the thresholded image
        cv::namedWindow("Binary Image");
        cv::imshow("Binary Image",thresholded);
        cv::imwrite("binary.bmp",thresholded);

        /*
        // Equalize the image
        cv::Mat eq= h.equalize(image);

        // Show the result
        cv::namedWindow("Equalized Image");
        cv::imshow("Equalized Image",eq);

        // Show the new histogram
        cv::namedWindow("Equalized Histogram");
        cv::imshow("Equalized Histogram",h.getHistogramImage(eq));

        // Stretch the image ignoring bins with less than 5 pixels
        cv::Mat str= h.stretch(image,5);

        // Show the result
        cv::namedWindow("Stretched Image");
        cv::imshow("Stretched Image",str);

        // Show the new histogram
        cv::namedWindow("Stretched Histogram");
        cv::imshow("Stretched Histogram",h.getHistogramImage(str));

        // Create an image inversion table
        uchar lookup[256];

        for (int i=0; i<256; i++) {

            lookup[i]= 255-i;
        }

        // Apply lookup and display negative image
        cv::namedWindow("Negative image");
        cv::imshow("Negative image",h.applyLookUp(image,lookup));
        */
}
