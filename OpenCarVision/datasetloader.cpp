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
#include <QtXML>
#include <QStringList>

//#include <opencv2/imgproc/imgproc.hpp> // cvtColor
//#include <opencv2/highgui/highgui.hpp> // imread

DatasetLoader::DatasetLoader()
{

}


#if 1

#define DATASET "dataset"
#define FILE    "file"
//const QString DatasetLoader::DATASET( "dataset" );
//const QString DatasetLoader::FILE( "file" );

// http://www.qtcentre.org/threads/61195-glob-like-fuction-to-list-all-files-in-subfolders-that-match-wildcard
QStringList DatasetLoader::dir(const QString path, const QString filters)
{
    QStringList files;
    QDir dir(path);
    QStringList name_filters;
    name_filters << filters;
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
            files << fi.fileName();
        }
        else{}
    }
    return files;
}

void DatasetLoader::writeTemplate(QString outFile)
{
    // create a template file which can be loaded via DatasetLoader
    // NICTA -----------------------------------------------------
    // Create a document to write XML
    QDomDocument document;

    // Making the root element
    QDomElement root = document.createElement(DATASET);
    root.setAttribute("Name", "nicta-RoadImageDatabase");
    root.setAttribute("Path", "After-Rain");

    // Adding the root element to the docuemnt
    document.appendChild(root);

    // Adding more elements
    QStringList subfolder;
    subfolder << "After-Rain" << "Sunny-Shadows" ;

    for(int i = 0; i < subfolder.size(); i++)
    {
        QDomElement dorm = document.createElement(DATASET);
        dorm.setAttribute("Name", subfolder.at(i));
        dorm.setAttribute("Path", subfolder.at(i));
        dorm.setAttribute("ID", QString::number(i+1)); // id start from 1
        root.appendChild(dorm);

        // Adding rooms to each dorm building
        {
            QDomElement file = document.createElement(FILE);
            file.setAttribute("Name", "raw");
            file.setAttribute("Selector", "*.tif");
            dorm.appendChild(file);
        }
        {
            QDomElement file = document.createElement(FILE);
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
