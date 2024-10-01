#include "widget.h"

#include <QApplication>
#include <QtGlobal>

#define QT_MESSAGE_PATTERN "[%{time process} %{type}] %{function} - %{message}"

int main(int argc, char *argv[])
{
    qSetMessagePattern(QT_MESSAGE_PATTERN);

    QApplication app(argc, argv);
    Widget widget;
    widget.setWindowFlag(Qt::WindowStaysOnTopHint);
    widget.show();
    return app.exec();
}
