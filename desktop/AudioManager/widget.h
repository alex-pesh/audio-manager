#ifndef WIDGET_H
#define WIDGET_H

#include "handler.h"
#include <QWidget>
#include <QErrorMessage>
#include <QMessageBox>
#include <QSlider>
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

public slots:
    void on_connect();
    void on_disconnect();
    void on_connectionError(QString &error);
    void on_syncEvent(Values &values);
    void on_valueEvent(CMD cmd, int8_t &value);

private slots:
    void on_sendBtn_pressed();

    void on_deviceComboBox_currentTextChanged(const QString &deviceName);

    void on_connectCheckBox_toggled(bool checked);

    void on_volumeSlider_valueChanged(int value);

    void on_balanceSlider_valueChanged(int value);

    void on_trebleSlider_valueChanged(int value);

    void on_bassSlider_valueChanged(int value);

    void on_muteCheckBox_toggled(bool checked);

    void on_logClearBtn_pressed();

private:
    Ui::Widget *ui;
    SerialHandler *handler;

    void showEvent(QShowEvent *event) override;
    static void setSliderValue(QObject *slider, const int &value);
};


class MessageBox : public QMessageBox {
    void resizeEvent(QResizeEvent *Event) override {
        QMessageBox::resizeEvent(Event);
        this->setFixedWidth(400);
    }
};

#endif // WIDGET_H
