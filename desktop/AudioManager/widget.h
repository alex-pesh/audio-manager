#ifndef WIDGET_H
#define WIDGET_H

#include "handler.h"
#include <QWidget>
#include <QtSerialPort/QSerialPort>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();


private slots:
    void on_deviceComboBox_currentTextChanged(const QString &arg1);

    void on_connectCheckBox_toggled(bool checked);

    void on_volumeSlider_valueChanged(int value);

    void on_balanceSlider_valueChanged(int value);

    void on_trebleSlider_valueChanged(int value);

    void on_bassSlider_valueChanged(int value);

    void on_muteCheckBox_toggled(bool checked);

    void on_logClearBtn_pressed();

    void connectionError(const QString & _t1);

private:
    Ui::Widget *ui;
    QSerialPort *serial;
    SerialHandler *handler;
};
#endif // WIDGET_H
