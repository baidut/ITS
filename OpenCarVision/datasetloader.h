#ifndef DATASETLOADER_H
#define DATASETLOADER_H

//#include <QWidget>
//#include <opencv2/core/core.hpp>
#include <QString>
#include <QtXML>
#include "dataset.h"

class DatasetLoader
{
public:
    DatasetLoader();

    bool import(QString xmlFile);
    QStringList getNames(){
        QStringList names;
        for (int i = 0; i < this->subdatasets.count(); ++i){
            names << QString("%1-%2").arg(this->name).arg(this->subdatasets[i].getName());
        }
        return names;
    }
    QStringList getFiles(unsigned int index, QString selectorname){
        return this->subdatasets[index].getFiles(selectorname);
    }

    void writeTemplate(QString outFile);

//    static const QString DATASET;
//    static const QString FILE;
private:
    bool parseXml(const QDomElement &element);
    bool parseXmlFolder(const QDomElement &element);

//    unsigned int number;
    QString rootPath;
    QString name;
    QVector<Dataset> subdatasets;
    int number;
//    QStringList subdatasets;
//    QVector<QVector<QPair<QString, QString>>> selectors;
//    QPair<QString, QString> pair;
};

#endif // DATASETLOADER_H
