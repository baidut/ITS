#ifndef UI_UTILS
#define UI_UTILS

#include <QColorDialog>

namespace UI_Utils {

// toolbutton -onclick--> QColorDialog -selectColor--> update toolbutton color
// http://stackoverflow.com/questions/15080404/colouring-a-button-in-qt
QColor colorPicker(QToolButton *button) {
    QColor color = QColorDialog::getColor();
    QString s("background: #"
              + QString(color.red() < 16? "0" : "") + QString::number(color.red(),16)
              + QString(color.green() < 16? "0" : "") + QString::number(color.green(),16)
              + QString(color.blue() < 16? "0" : "") + QString::number(color.blue(),16) + ";");
    button->setStyleSheet(s);
    button->update();

    //qDebug() << "Set color: " << s;
    return color;

}

}

#endif // UI_UTILS

