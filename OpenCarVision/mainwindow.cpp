
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include <QDebug>
#include <QInputDialog>
#include <QMessageBox>
#include <ui_utils.h>

#include "linefinder.h"
#include "histogram.h"
#include "edgedetector.h"

#include "datasetloader.h"

#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(this,SIGNAL(imageChanged()),this,SLOT(on_image_changed()));
    connect(this,SIGNAL(imageProcessed()),this,SLOT(on_image_processed()));


    timer = new QTimer(this);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(on_timeout()));

    roadDrawer = new RoadDrawer();
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
        qDebug("Channels:%d",image.channels());
        if(1==image.channels()){
            img = QImage((const unsigned char*)(image.data),
                                image.cols,image.rows,image.cols,QImage::Format_Indexed8);
            qDebug("黑白图片");
        }
        else{
            cv::cvtColor(image,imageRGB,CV_BGR2RGB);
            img = QImage((const unsigned char*)(imageRGB.data),
                                image.cols,image.rows,image.cols*image.channels(),QImage::Format_RGB888);
            qDebug("彩色图片");
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
    cv::resize(image,imgResized,(image.cols>image.rows)?
                   cv::Size(100,100*image.rows/image.cols):cv::Size(100*image.cols/image.rows,100)); //利用OpenCV的缩放代替QT的缩放解决缩放bug
    display_image(imgResized,ui->label_img);

    // 添加重绘直方图
    // The histogram object
    Histogram1D h;
    // Compute the histogram
    /*cv::MatND histo=*/ h.getHistogram(image);
    // Display a histogram as an image
    cv::resize(h.getHistogramImage(image),imgResized,cv::Size(100,100));
    display_image(imgResized,ui->label_histogram);
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
    //cv::Canny(image,imgProc,ui->doubleSpinBox_threshold1->value(),ui->doubleSpinBox_threshold2->value());

    double threshold1 = ui->doubleSpinBox_threshold1->value();
    double threshold2 = ui->doubleSpinBox_threshold2->value();

    cv::Mat contours/*黑底白线*/;// 默认不反转，便于后续处理，可以通过invert按钮反转,contoursInv/*白底黑线*/;

    cv::Canny(image,contours,threshold1,threshold2);
    // cv::threshold(contours,contoursInv,128,255,cv::THRESH_BINARY_INV);

    imgProc = contours.clone();
    emit imageProcessed();
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

        // 可以左边 中间 右边 分开处理 混合处理时中间没必要判断越界条件

        for(int i = 1; i < image.cols-1; i++){ // i 表征当前像素左边像素有几个
            sumL = sumR = 0;
            int k;
            for(k = 1; k <= Range && i - k >= 0; k++){
                sumL += current[i-k];
            }
            int avrL = sumL / (k-1);

            for(k = 1; k <= Range && i + k < image.cols; k++){
                sumR += current[i+k];
            }
            int avrR = sumR / (k-1);

            if( (current[i] > avrL + Th)
                && (current[i] > avrR + Th ) // 共image.cols个，当前1个，左边i个
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
    fileName = QFileDialog::getOpenFileName(this,tr("Open Image"),
                                    ".",tr("Image Files (*.png *.jpg *.bmp)"));
    qDebug()<<"filenames:"<<fileName;
    // QString转char* qstr.toLatin1().data()
    image = cv::imread(fileName.toLatin1().data()); //fileName.toAscii().data()

    if (!image.data) // 图片打开失败的情形
        return;
    emit imageChanged();
    imgProc = image.clone();
    emit imageProcessed();
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

#define WIN_NAME_SOBEL  "Sobel"

static void on_trackbar_sobel(int value, void* usrdata)
{
    cv::Mat img=*(cv::Mat*)(usrdata);   //强制类型转换
    //Mat dst;

    // Compute Sobel
    EdgeDetector ed;
    ed.computeSobel(img);

    cv::imshow (WIN_NAME_SOBEL,ed.getBinaryMap(value));
}

void MainWindow::on_pushButton_sobel_clicked()
{ // 此处待修复
    int poiTrackBar=120;//trackbar的值

    cv::namedWindow(WIN_NAME_SOBEL);
    //cv::imshow(WIN_NAME_SOBEL,image);
        //创建trackbar，我们把img作为数据传进回调函数中
    cv::createTrackbar("NULL",WIN_NAME_SOBEL,&poiTrackBar,1000,on_trackbar_sobel,&image);// 注意 ,&image
    //on_trackbar_sobel(120,&image);
/*
    // Compute Sobel
    EdgeDetector ed;
    ed.computeSobel(image);

    // Display the Sobel orientation
    // 浮雕特效图
    // cv::imshow("Sobel (orientation)",ed.getSobelOrientationImage());
    // cv::imwrite("ori.bmp",ed.getSobelOrientationImage());
    // 二值化图
    // 可以确定是label显示的问题，这里由于不是研究重点，先暂放，待在linux下安装最新版Qt再测试
    // 探究一下imshow添加拖动条
    imgProc = ed.getBinaryMap(ui->doubleSpinBox_sobelThreshold->value());

    createTrackbar( "Threshold", "Connected Components", &threshval, 255, on_trackbar );
    on_trackbar_sobel(threshval, 0);//轨迹条回调函数
    //emit imageProcessed();
*/
}

void MainWindow::on_pushButton_2inch_clicked()
{ // 图片dpi不符合要求，待修复
    cv::Mat imgResized;
    cv::resize(image,imgResized,cv::Size(413,626)); //利用OpenCV的缩放代替QT的缩放解决缩放bug
    cv::imwrite("output.jpg",imgResized); //两寸413*626像素35*53毫米
    QMessageBox::about(NULL, "Note", "Finished! Please check current folder.");
    // 再用默认程序弹出图片！
}

void MainWindow::on_pushButton_findLines_clicked()
{ // 要求原图是提取了边缘的二值图，否则程序会当机
    // 可选择底片，标识在原图上，还是边缘提取后的图上
    imgProc = cv::imread(fileName.toLatin1().data()); // 这里克隆原始图片
    // imgProc = image.clone();二值图上加白线看不清楚，需要加彩色线

    if(ui->checkBox_probabilistic->isChecked()){
        cv::Mat contours = image.clone();
        // Create LineFinder instance
        LineFinder ld;

        // Set probabilistic Hough parameters
        int minVote = ui->spinBox_minVote->value();
        ld.setLineLengthAndGap(100,20);
        ld.setMinVote(minVote);

        // Detect lines
        std::vector<cv::Vec4i> li= ld.findLines(contours);

        // 消去不连续直线 可选项目， 不知道是否影响虚线车道识别
        if(ui->checkBox_dirFilter->isChecked()){
            EdgeDetector ed;
            ed.computeSobel(imgProc/*原始图像*/);
            // eliminate inconsistent lines
            ld.removeLinesOfInconsistentOrientations(ed.getOrientation(),0.4,0.1);
        }

        ld.drawDetectedLines(imgProc);

        ui->statusBar->showMessage(QString("%1 lines detected.").arg(li.size()));
        if( 10 >= li.size() ){
            for(unsigned int n = 0; n < /*1*/ li.size(); n++ ){
                // cv::line(image, cv::Point(li[n][0],li[n][1]),cv::Point(li[n][2],li[n][3]),cv::Scalar(255),5);
                // cv::imshow("One line of the Image",image);

                // Extract the contour pixels of the first detected line
                cv::Mat oneline(image.size(),CV_8U,cv::Scalar(0)); // 0 为 黑色
                cv::line(oneline, cv::Point(li[n][0],li[n][1]),cv::Point(li[n][2],li[n][3]),cv::Scalar(255),5);
                cv::bitwise_and(image/*即contours*/,oneline,oneline);

                /// cv::imshow("One line", oneline);
                // cv::Mat onelineInv;
                // cv::threshold(oneline,onelineInv,128,255,cv::THRESH_BINARY_INV);
                // cv::namedWindow("One line");
                // cv::imshow("One line",onelineInv); // 白底黑线

                std::vector<cv::Point> points;

                // Iterate over the pixels to obtain all point positions
                for( int y = 0; y < oneline.rows; y++ ) {

                    uchar* rowPtr = oneline.ptr<uchar>(y);

                    for( int x = 0; x < oneline.cols; x++ ) {

                        // if on a contour
                        if (rowPtr[x]) {

                            points.push_back(cv::Point(x,y));
                        }
                    }
                }

                // find the best fitting line
                cv::Vec4f line;
                cv::fitLine(cv::Mat(points),line,CV_DIST_L2,0,0.01,0.01);

                // std::cout << "line: (" << line[0] << "," << line[1] << ")(" << line[2] << "," << line[3] << ")\n";

                int x0= line[2];
                int y0= line[3];
                int x1= x0-200*line[0];
                int y1= y0-200*line[1];

                cv::line(imgProc,cv::Point(x0,y0),cv::Point(x1,y1),cv::Scalar(0),3);
                // cv::imshow("Estimated line",image);
            }
        }
    }
    else{
        // Hough tranform for line detection
        std::vector<cv::Vec2f> lines;
        int minVote = ui->spinBox_minVote->value();

        cv::HoughLines(image/*contours*/,lines,1,PI/180,minVote);

        // Draw the lines
        cv::Mat result(image.rows,image.cols,CV_8U,cv::Scalar(255));

        imgProc.copyTo(result);

        std::vector<cv::Vec2f>::const_iterator it= lines.begin();

        ui->statusBar->showMessage(QString("%1 lines detected.").arg(lines.size()));

        while (it!=lines.end()) {

            float rho= (*it)[0];   // first element is distance rho
            float theta= (*it)[1]; // second element is angle theta

            if (theta < PI/4. || theta > 3.*PI/4.) { // ~vertical line

                // point of intersection of the line with first row
                cv::Point pt1(rho/cos(theta),0);
                // point of intersection of the line with last row
                cv::Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);
                // draw a white line
                cv::line( result, pt1, pt2, cv::Scalar(255), 3);

            } else { // ~horizontal line

                // point of intersection of the line with first column
                cv::Point pt1(0,rho/sin(theta));
                // point of intersection of the line with last column
                cv::Point pt2(result.cols,(rho-result.cols*cos(theta))/sin(theta));
                // draw a white line
                cv::line( result, pt1, pt2, cv::Scalar(255), 3);
            }

            ++it;
        }
        imgProc = result.clone();
    }
    emit imageProcessed();
}

void MainWindow::on_pushButton_display_clicked()
{
    imgProc = image.clone();
    emit imageProcessed();
}

void MainWindow::on_pushButton_resume_clicked()
{
    image = cv::imread(fileName.toLatin1().data());
    emit imageChanged();
    imgProc = image.clone();
    emit imageProcessed();
}


// 中值虽然可以滤去噪声，但是计算复杂度高
inline uchar median(uchar a,uchar b,uchar c){
    return a > b ? (b > c ? b : (a > c ? c : a)) : (a > c ? a : (b > c ? c : b));
}

void MainWindow::on_pushButton_detectLanes_clicked()
{
    // 建议通过面向对象方式完成
    // 先灰度化
    cv::cvtColor(image,imgProc,CV_BGR2GRAY);// 需要根据图像源采取不同处理
    // TODO...

#if 0 // 智能小车的算法实现起来比较麻烦
    // 先将图像倒置？没必要
    // 是否利用色彩信息？基于色彩肯定更准确，但这里暂不做相关研究
    // 先完成找突变标定工作

    int L1, L2 = 0, R1 = image.cols, R2, M = image.cols / 2;
    int max, min;


    for (int j = image.rows-1; j; j--){
        const uchar* r = image.ptr<const uchar>(j);
        uchar* output = imgProc.ptr<uchar>(j);

        max = min = median(r[M-1],r[M],r[M+1]);
        max += 10;
        min -= 10;

        for (int i = M - 3; i > 1 ; i -= 3){
            uchar cur = median(r[i-1],r[i],r[i+1]);
            if (cur > max){
                if (cur - max >  (max - min) ){
                    output[i] = 255;
                    output[i-1] = output[i+1] = 0; // 更清晰
                    L2 = i;
                    break;
                }
                else max = cur;
            }
            else if(cur < min){
                min = cur;
            }
        }
        for (int i = M + 3; i < image.cols - 1 ; i += 3){
            uchar cur = median(r[i-1],r[i],r[i+1]);
            if (cur > max){
                if (cur - max >  (max - min) ){
                    output[i] = 255;
                    output[i-1] = output[i+1] = 0; // 更清晰
                    R1 = i;
                    break;
                }
                else max = cur;
            }
            else if(cur < min){
                min = cur;
            }
        }
        //M = (R1 + L2) / 2;
        //output[M-1] = output[M+1] = output[M] = 0; // 标中线
    }
#endif
    emit imageProcessed();
}

// to be removed
void MainWindow::load_dataset(int arg1, QString raw, QString gt, QString res)
{
    if(raw.isEmpty()) return;
    ui->label_camvid_raw->imshow(QString(raw).arg(arg1));
    ui->label_camvid_gt->imshow(QString(gt).arg(arg1));
    ui->label_camvid_res->imshow(QString(res).arg(arg1));
}

void MainWindow::on_pushButton_threshold_clicked()
{
    cv::threshold(image,imgProc,
                  ui->doubleSpinBox_thresh->value(),
                  ui->doubleSpinBox_maxval->value(),
                  ui->comboBox_threshType->currentIndex());
    emit imageProcessed(); // TODO: 改成整个Process区域被点击后触发图像改变，触发顺序问题，需要等待处理完成
}

void MainWindow::on_pushButton_loadCamvid_clicked()
{
    raw = "../SegNet/raw/%1.jpg";
    gt = "../SegNet/gt/%1.jpg";
    res = "../SegNet/res/%1.jpg";
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()+1);
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()-1);
    ui->spinBox_camvidNo->setMaximum(232);
    ui->label_camvid_legend->imshow("../SegNet/legend.jpg");
}

void MainWindow::on_spinBox_camvidNo_valueChanged(int arg1)
{
    load_dataset(arg1, raw, gt, res);
    //ui->label_camvid_raw->imshow(QString("../SegNet/raw/%1.jpg").arg(arg1));
    //ui->label_camvid_gt->imshow(QString("../SegNet/gt/%1.jpg").arg(arg1));
    //ui->label_camvid_res->imshow(QString("../SegNet/res/%1.jpg").arg(arg1));
}

void MainWindow::on_pushButton_play_clicked(bool checked)
{

}


void MainWindow::on_pushButton_loadCamvid4_clicked()
{
    raw = "../results/segnet_basic_cam_test/%1_raw.png";
    gt = "../results/segnet_basic_cam_test/%1_gt.png";
    res = "../results/segnet_basic_cam_test/%1_seg.png";
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()+1);
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()-1);
    ui->spinBox_camvidNo->setMaximum(286);
    ui->label_camvid_legend->clear();
}

void MainWindow::on_pushButton_loadKittiTrain_clicked()
{
    raw = "../results/segnet_basic_kit_train/%1_raw.png";
    gt = "../results/segnet_basic_kit_train/%1_gt.png";
    res = "../results/segnet_basic_kit_train/%1_seg.png";
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()+1);
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()-1);
    ui->spinBox_camvidNo->setMaximum(286);
    ui->label_camvid_legend->clear();
}

void MainWindow::on_pushButton_loadKittiTest_clicked()
{
    raw = "../results/segnet_basic_kit_test/%1_raw.png";
    gt = "";
    res = "../results/segnet_basic_kit_test/%1_seg.png";
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()+1);
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()-1);
    ui->spinBox_camvidNo->setMaximum(290);
    ui->label_camvid_legend->clear();
}

void MainWindow::on_pushButton_loadAfterRain_clicked()
{
    raw = "../results/segnet_basic_afterrain/%1_raw.png";
    gt = "../results/segnet_basic_afterrain/%1_gt.png";
    res = "../results/segnet_basic_afterrain/%1_seg.png";
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()+1);
    ui->spinBox_camvidNo->setValue(ui->spinBox_camvidNo->value()-1);
    ui->spinBox_camvidNo->setMaximum(285);
    ui->label_camvid_legend->clear();
}

void MainWindow::on_pushButton_s2_afterrain_clicked()
{
    raw = "../results/segnet_basic_afterrain/%1_raw.png";
    ui->spinBox_s2_no->setValue(ui->spinBox_s2_no->value()+1);
    ui->spinBox_s2_no->setValue(ui->spinBox_s2_no->value()-1);
    ui->spinBox_s2_no->setMaximum(285);
}

void MainWindow::on_spinBox_s2_no_valueChanged(int arg1)
{
    using namespace cv;
    Mat raw_image;
    Mat channel[3];
    Mat V,V_MIN;
    raw_image = imread(raw.arg(arg1).toLatin1().data());
    split(raw_image,channel);

//    imshow("original",img);
//    imshow("B",channel[0]);
//    imshow("G",channel[1]);
//    imshow("R",channel[2]); 矫正亮度后再合并 HSV 矫正V为S2 去除阴影

#define B (channel[0])
#define G (channel[1])
#define R (channel[2])

    V = max(R,max(B,G));
    V_MIN = min(R,min(B,G));

    Mat V2; //(raw_image.rows, raw_image.cols, CV_64FC1);
    V2 = max(V, Mat::ones(raw_image.rows, raw_image.cols, CV_8UC1));

//    Mat S2(raw_image.rows, raw_image.cols, CV_64F);
//    Mat S(raw_image.rows, raw_image.cols, CV_64F);

//    S2 = (V - B) / V2;
//    S = (V - V_MIN) / V2;
//    Mat VminusB = V - B;
//    VminusB.convertTo(VminusB, CV_64F);

    // 可检测车辆
    // equalizeHist(S2,S2);
    // equalizeHist(S,S);

    Mat S22(raw_image.rows, raw_image.cols, CV_8UC1);
    Mat S(raw_image.rows, raw_image.cols, CV_8UC1);
    for( int y = 0; y < S22.rows; y++ )
    {
       for( int x = 0; x < S22.cols; x++ )
       {
           S22.at<uchar>(y,x) = ( V.at<uchar>(y,x) - B.at<uchar>(y,x) ) * 255 / V2.at<uchar>(y,x)  ;
           S.at<uchar>(y,x) = ( V.at<uchar>(y,x) - V_MIN.at<uchar>(y,x) ) * 255 / V2.at<uchar>(y,x)  ;
       }
    }


    //
    Mat HSV, image2, image3;
    Mat channel_hsv[3];
    cvtColor(raw_image, HSV, COLOR_BGR2HSV);

    split(HSV,channel_hsv);
    channel_hsv[2] = S22; // 2

    merge(channel_hsv,3,image2);
    cvtColor(image2, image3, COLOR_HSV2BGR);

    ui->label_raw_s2->imshow(raw_image);
    ui->label_s2_s2->imshow(image3);  // V - B
    ui->label_s_s2->imshow(S22);
    ui->label_ii_s2->imshow(S);

#undef B
#undef G
#undef R
}

void MainWindow::on_pushButton_s2_kitti_clicked()
{
    raw = "../results/segnet_basic_kit_train/%1_raw.png";
    ui->spinBox_s2_no->setValue(ui->spinBox_s2_no->value()+1);
    ui->spinBox_s2_no->setValue(ui->spinBox_s2_no->value()-1);
    ui->spinBox_s2_no->setMaximum(286);
}

void MainWindow::on_spinBox_nFrameOfDataset_valueChanged(int arg1)
{
    Q_ASSERT(arg1<=rawFiles.count());
//    ui->label_rawdata->imshow(QString("E:\Sync\my\project\datasets\nicta-RoadImageDatabase\After-Rain\after_rain%05d.tif").arg(arg1));

    cv::Mat rawImg;
    QColor colorSetting;

    rawImg = cv::imread(this->rawFiles.at(arg1).toLatin1().data());
    //rawImg = cv::resize();
    double alpha = ui->horizontalSlider_plendAlpha->value()/100.0;

    this->roadDrawer->setAlpha(alpha);

    if ( ui->checkBox_roadLabel->isChecked() ) {
        // may make it very slow, resize first may be help

        //color = widget.palette().color(QPalette.Background);
        colorSetting = ui->toolButton_roadColor->palette().background().color();
        // colorSetting = ui->toolButton_carBoxColor->palette().button().style().BackgroundColorRole;
                //->palette().color(QToolButton::backgroundRole());
        // cv::Scalar color2 = cv::Scalar(colorSetting.red(),colorSetting.green(), colorSetting.blue());
        this->roadDrawer->setColorMap(QColor(255, 255, 255), colorSetting ); // QColor(255, 170, 255)
    }

    this->roadDrawer->drawResult(arg1,rawImg);
    ui->label_rawdata->imshow(rawImg);

    // How to get the color QToolButton
    // color of QPushButton or QToolButton

}

void MainWindow::on_actionLoad_triggered()
{
//    loader.writeTemplate("nicta-RoadImageDatabase.xml");
    loader.import("nicta-RoadImageDatabase.xml");
    qDebug()<<loader.getNames();
    qDebug()<<"load ok";
    ui->comboBox_datasetName->addItems(loader.getNames());

    this->roadDrawer->loadResultFile(this->roadGtFiles); // TODO: remove roadGtFiles
}

void MainWindow::on_comboBox_datasetName_currentIndexChanged(int index)
{
    this->rawFiles = this->loader.getFiles(index,"raw");
    this->roadGtFiles = this->loader.getFiles(index,"gt-road");

    this->roadDrawer->loadResultFile(this->roadGtFiles); // TODO: remove roadGtFiles

//    qDebug()<<files;
    ui->spinBox_nFrameOfDataset->setMinimum(1);
    ui->spinBox_nFrameOfDataset->setMaximum(rawFiles.count()-1); // +1

    ui->horizontalSlider_PlayProgress->setMaximum(rawFiles.count()-1);

    if (ui->spinBox_nFrameOfDataset->value() > rawFiles.count()) {
        ui->spinBox_nFrameOfDataset->setValue(rawFiles.count());
    }

    on_spinBox_nFrameOfDataset_valueChanged(ui->spinBox_nFrameOfDataset->value());

}

void MainWindow::on_horizontalSlider_plendAlpha_valueChanged(int value)
{
    emit on_spinBox_nFrameOfDataset_valueChanged(ui->spinBox_nFrameOfDataset->value());
}

void MainWindow::on_pushButton_playVideo_clicked(bool checked)
{
    if(checked){
        int fps = ui->spinBox_playFps->value();
        timer->start(1000/fps); //time specified in ms
        ui->pushButton_playVideo->setText("Pause");
    } else {
        timer->stop();
        ui->pushButton_playVideo->setText("Play");
    }
}

void MainWindow::on_timeout()
{
    int value = ui->spinBox_nFrameOfDataset->value();
    int max = ui->spinBox_nFrameOfDataset->maximum();
    if (value < max) {
        ui->spinBox_nFrameOfDataset->setValue(value+1);
    }
}



void MainWindow::on_toolButton_roadColor_clicked()
{
    QColor color = QColorDialog::getColor();
    QString s("background: #"
                          + QString(color.red() < 16? "0" : "") + QString::number(color.red(),16)
                          + QString(color.green() < 16? "0" : "") + QString::number(color.green(),16)
                          + QString(color.blue() < 16? "0" : "") + QString::number(color.blue(),16) + ";");
    ui->toolButton_roadColor->setStyleSheet(s);
    ui->toolButton_roadColor->update();
}

void MainWindow::on_toolButton_carBoxColor_clicked()
{
    QColor color = UI_Utils::colorPicker(ui->toolButton_carBoxColor);
    qDebug() << color;
}
