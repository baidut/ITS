/*
* Copyright (c) 2015-2016 Zhenqiang.YING yingzhenqiang-at-gmail-dot-com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "datasetloader.h"

#include <QDir>
#include <QStringList>
#include <QMessageBox>

//#include <opencv2/imgproc/imgproc.hpp> // cvtColor
//#include <opencv2/highgui/highgui.hpp> // imread

DatasetLoader::DatasetLoader()
{

}

bool DatasetLoader::import(QString xmlFile)
{
    // open xmlFile and load data
    QFile file(xmlFile);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug() << "Error: Open the file for importing failed";
        return false;
    }

    QString errorStr;
    int errorLine;
    int errorColumn;

    QDomDocument doc;
    if (!doc.setContent(&file, false, &errorStr, &errorLine,
                        &errorColumn)) {
        qDebug() << QString("XML Parse error at line %1, column %2: %3")
                    .arg(errorLine).arg(errorColumn).arg(errorStr);
        return false;
    }

    QDomElement root = doc.documentElement();
    if (root.tagName() != "dataset") {
        qDebug() << "Not a valid dataset file";
        return false;
    }

    this->rootPath = root.attribute("Path");
    this->name = root.attribute("Name");
    return parseXml(root);
}

bool DatasetLoader::parseXml(const QDomElement &element)
{
    qDebug() << "parseXml...";
    this->number = 0;

    QDomNode child = element.firstChild();

    while(!child.isNull()){
        if(child.toElement().tagName()=="folder") {
            if(false == parseXmlFolder(child.toElement())){
                return false;
            }
        }
        child = child.nextSibling();
    }
    return true;
}

bool DatasetLoader::parseXmlFolder(const QDomElement &element)
{ // construct subdatasets
    QMap<QString, QString> selectors;
    QDomNode child = element.firstChild();

    qDebug() << "parseXmlFolder...";
    this->number++;
    while(!child.isNull()){
        if(child.toElement().tagName()=="file") {
            QString name = child.toElement().attribute("Name");
            QString selector = child.toElement().attribute("Selector");
            selectors[name] = selector;
        }
        child = child.nextSibling();
    }

    this->subdatasets.append(
               Dataset(
                element.attribute("Name"),
                QString("%1\\%2").arg(this->rootPath).arg(element.attribute("Path")),
                selectors) );
    return true;
}

#if 1
//const QString DatasetLoader::DATASET( "dataset" );
//const QString DatasetLoader::FILE( "file" );



void DatasetLoader::writeTemplate(QString outFile)
{
    // create a template file which can be loaded via DatasetLoader
    // NICTA -----------------------------------------------------
    // Create a document to write XML
    QDomDocument document;

    // Making the root element
    QDomElement root = document.createElement("dataset");
    root.setAttribute("Name", "nicta-RoadImageDatabase");
    root.setAttribute("Path", "E:\\Sync\\my\\project\\datasets\\nicta-RoadImageDatabase");

    // Adding the root element to the docuemnt
    document.appendChild(root);

    // Adding more elements
    QStringList subfolder;
    subfolder << "After-Rain" << "Sunny-Shadows" ;

    for(int i = 0; i < subfolder.size(); i++)
    {
        QDomElement dorm = document.createElement("folder");
        dorm.setAttribute("Name", subfolder.at(i));
        dorm.setAttribute("Path", subfolder.at(i));
        dorm.setAttribute("ID", QString::number(i+1)); // id start from 1
        root.appendChild(dorm);

        // Adding rooms to each dorm building
        {
            QDomElement file = document.createElement("file");
            file.setAttribute("Name", "raw");
            file.setAttribute("Selector", "*.tif");
            dorm.appendChild(file);
        }
        {
            QDomElement file = document.createElement("file");
            file.setAttribute("Name", "gt-road");
            file.setAttribute("Selector", "*.png");
            dorm.appendChild(file);
        }
    }
    // Writing to a file
    QFile file(outFile);
    if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "Open the file for writing failed";
    }
    else
    {
        QTextStream stream(&file);
        stream << document.toString();
        file.close();
        qDebug() << "Writing is done";
    }
}
#endif
