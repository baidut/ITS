#ifndef DATASETLOADER_H
#define DATASETLOADER_H

//#include <QWidget>
//#include <opencv2/core/core.hpp>
#include <QString>

class DatasetLoader
{
public:
    DatasetLoader();

    QStringList dir(const QString path, const QString filters);
    void writeTemplate(QString outFile);

//    static const QString DATASET;
//    static const QString FILE;
private:
    unsigned int number;
    QString root;
//    QPair<QString, QString> pair;
};

#endif // DATASETLOADER_H
