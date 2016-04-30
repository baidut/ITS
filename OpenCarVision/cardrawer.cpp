#include "cardrawer.h"

using namespace std;
using namespace cv;

CarDrawer::CarDrawer(string gt_file, string basePath, int file_mode)
{
    this->basePath = basePath;
    this->curMode = file_mode;
    loadResultFile(gt_file);
}

void
CarDrawer::loadResultFile(const string gt_file)
{
    ifstream fin;
    fin.open(gt_file.c_str());

    if (!fin)
    {
        cerr << "open file failed !" << endl;
        exit(-1);
    }
    string line;
    int img_id = 0;
    while (std::getline(fin, line))
    {
        string imgName;
        int carNum;
        int x1 = 0, x2 = 0, y1 = 0, y2 = 0, type = 0;
        istringstream parser(line);
        parser >> imgName;
        parser >> carNum;

        //cout << "imageName" << " " << imgName << endl;
        //cout << "carNum" << " " << carNum << endl;

        imgList.push_back(imgName);
        vector<CarInfo> v;

        if (curMode == 0)
        {
            for (int i = 0; i < carNum; i++)
            {
                parser >> x1 >> y1 >> x2 >> y2;
                //cout << x1 << " " << y1 << " " << x2 << " " << y2 << " " << endl;
                CarInfo carInfo = CarInfo(x1, y1, x2, y2);
                cout << carInfo.x1 << " " << carInfo.y1 << " " << carInfo.x2 << " " << carInfo.y2 << " " << endl;
                v.push_back(carInfo);
            }
        }

        if (curMode == 1)
        {
            for (int i = 0; i < carNum; i++)
            {
                parser >> x1 >> y1 >> x2 >> y2 >> type;
                //cout << x1 << " " << y1 << " " << x2 << " " << y2 << " " << type << endl;
                CarInfo carInfo = CarInfo(x1, y1, x2, y2, type);
                v.push_back(carInfo);
            }
        }
        resultList.push_back(v);
        img_id++;

    }
    fin.close();
    imgNum = imgList.size();
}

Mat
CarDrawer::drawResult(int imgID)
{

    Mat curImg = imread(basePath + "\\"+ imgList[imgID]);
    int carNum = resultList[imgID].size();
    cout << "CarNum"<<carNum;

    for (int i = 0; i < carNum; i++)
    {

        cout << "Point 1 (" << resultList[imgID][i].x1 << ","<<resultList[imgID][i].y1 << ")" << " ";
        cout << "Point 2 (" << resultList[imgID][i].x2 << ","<<resultList[imgID][i].y2 << ")" << " " << endl;
        rectangle(curImg, Point(resultList[imgID][i].x1, resultList[imgID][i].y1), Point(resultList[imgID][i].x2, resultList[imgID][i].y2), Scalar(0, 255, 0),2);
        if (curMode == 1)
        {
            string carType = getCarType(resultList[imgID][i].type);
            putText(curImg, carType, Point(resultList[imgID][i].x1, resultList[imgID][i].y1), FONT_HERSHEY_SIMPLEX,1, Scalar(0, 255, 0),2);
        }
    }

    return curImg;
}

string
CarDrawer::getCarType(int typeID)
{
    string carType;
    //add more types if needed later
    switch(typeID)
    {
        case 1: carType = "Sedan"; break;
        case 2: carType = "SUV"; break;
        case 3: carType = "MPV"; break;
        case 4: carType = "Pickup"; break;
        case 5: carType = "Truck"; break;
        case 6: carType = "Bus"; break;
    }
    return carType;
}
