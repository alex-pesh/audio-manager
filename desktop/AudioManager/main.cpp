#include "widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    Widget widget;
    widget.setWindowFlag(Qt::WindowStaysOnTopHint);
    widget.show();
    return app.exec();
}
