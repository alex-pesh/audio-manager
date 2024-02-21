#include "widget.h"
#include "./ui_widget.h"
#include <QtSerialPort/QSerialPort>
#include <thread>

#include <QDebug>
#include <QtCore/qglobal.h>
#include <QScreen>

#ifndef __DEBUG__H
#define __DEBUG__H
#endif



Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , serial(new QSerialPort(this))
    , handler(new SerialHandler())

{

//    handler->connectTo("/dev/ttyUSB0");
//    handler->connectTo("/tmp/simavr-uart0-tap");
//    handler->connectTo("/tmp/simavr-uart0");

    move(qApp->primaryScreen()->availableGeometry().center()-rect().center());
    ui->setupUi(this);

    connect(handler, SIGNAL(connected()), this, SLOT(on_connect()));
    connect(handler, SIGNAL(connectionError(QString&)), this, SLOT(on_connectionError(QString&)));

    handler->connectTo(ui->deviceComboBox->currentText());
}


Widget::~Widget()
{
    serial->close();
    delete serial;
    delete handler;
    delete ui;
}


void Widget::on_connect() {
    ui->controlGroupBox->setEnabled(true);
}


void Widget::on_connectionError(QString &error)
{
    ui->controlGroupBox->setEnabled(false);
    ui->connectCheckBox->setCheckState(Qt::Unchecked);

    MessageBox msgBox;
    msgBox.setText("Error");
    msgBox.setInformativeText(error);
    msgBox.setStandardButtons(MessageBox::Ok);
    msgBox.setIcon(MessageBox::Icon::Critical);

    int result = msgBox.exec();
    if (result== MessageBox::Ok) {
//        exit(1);
    }
}


void Widget::on_deviceComboBox_currentTextChanged(const QString &deviceName)
{
    qDebug() << "Device changed: " << deviceName;
    on_connectCheckBox_toggled(false);
}


void Widget::on_connectCheckBox_toggled(bool checked)
{
    if (checked) {
        handler->connectTo(ui->deviceComboBox->currentText());
    } else {
        ui->controlGroupBox->setEnabled(false);
        ui->connectCheckBox->setCheckState(Qt::Unchecked);
        handler->disconnect();
    }
}


void Widget::on_volumeSlider_valueChanged(int value)
{
//    ui->logBrowser->append("========= Volume =========");

    value = value*63/100;
    qDebug() << "Volume: " << value;

    handler->sendCommand(CMD::SET_VOLUME, value);
}


void Widget::on_balanceSlider_valueChanged(int value)
{
    value = value-31;
    qDebug() << "Balance: " << value;

    handler->sendCommand(CMD::SET_BALANCE, value);
}


void Widget::on_muteCheckBox_toggled(bool value)
{
    qDebug() << "Mute: " << value;

    handler->sendCommand(CMD::SET_MUTE, (value ? 1 : 0));
}


void Widget::on_trebleSlider_valueChanged(int value)
{
    value = (value-50)*7/50;
    qDebug() << "Treble: " << value;

    handler->sendCommand(CMD::SET_TREBLE, value);
}


void Widget::on_bassSlider_valueChanged(int value)
{
    value = (value-50)*7/50;
    qDebug() << "Bass: " << value;

    handler->sendCommand(CMD::SET_BASS, value);
}


void Widget::on_logClearBtn_pressed()
{
    ui->logBrowser->clear();
}

