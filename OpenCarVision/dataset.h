#ifndef DATASET_H
#define DATASET_H

#include <QMap>
//#include <opencv2/core/core.hpp>

class Dataset
{
public:
    Dataset(){}
    Dataset(QString name, QString path, QMap<QString, QString> selectors)
        :name(name),path(path),selectors(selectors){
    }
//    cv::Mat getImg(QString selectorname, int index){}
    QStringList getFiles(QString selectorname){
        QString selector = this->selectors[selectorname];
        //QString filter = QString("%1\\%2").arg(this->path).arg(selector);
        return dir(selector);
    }
    QString getName(){return this->name;}

private:
    // http://www.qtcentre.org/threads/61195-glob-like-fuction-to-list-all-files-in-subfolders-that-match-wildcard
    QStringList dir(const QString &filter)
    {
        QStringList files;
        QDir dir(this->path);
        QStringList name_filters;
        name_filters << filter;

//        qDebug() << filter;

        QFileInfoList fil = dir.entryInfoList(name_filters, QDir::NoDotAndDotDot|QDir::AllDirs|QDir::Files);
        for (int i = 0; i < fil.size(); i++)
        {
            QFileInfo fi = fil.at(i);
            if (fi.isDir()){
                // recursive
                // dir(fi.absolutePath());
            }
            else if (fi.isFile()){
                // do_something_with_file(fi);
                files << QString("%1\\%2").arg(this->path).arg(fi.fileName());

            }
            else{}
        }
        return files;
    }
    QString name;
    QString path;
    QMap<QString, QString> selectors;

    unsigned int number;
};

#endif // DATASET_H
