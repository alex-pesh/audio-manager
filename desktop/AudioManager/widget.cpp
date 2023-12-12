#include "widget.h"
#include "./ui_widget.h"
#include <QtSerialPort/QSerialPort>
#include <QErrorMessage>
#include <QMessageBox>

#include <QDebug>
#include <QtCore/qglobal.h>

#ifndef __DEBUG__H
#define __DEBUG__H
#endif


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , serial(new QSerialPort(this))

{

    QString portName = "/dev/ttyUSB0";
    serial->setPortName(portName);
    serial->setBaudRate(QSerialPort::BaudRate::Baud9600);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open serial " << portName << ": " << serial->errorString();

        QMessageBox msgBox;
        msgBox.setText("Error");
        msgBox.setInformativeText(QString::fromLatin1(
                                   "Failed to open serial port ").append(portName).append(": ").append(serial->errorString()));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setIcon(QMessageBox::Icon::Critical);

        int result = msgBox.exec();
        if (result== QMessageBox::Ok) {
            exit(1);
        }
    }

    ui->setupUi(this);

}

Widget::~Widget()
{
    serial->close();
    delete serial;
    delete ui;
}


void Widget::on_volumeSlider_valueChanged(unsigned value)
{
    qDebug() << "\n========= Volume =========";
    ui->logBrowser->append("========= Volume =========");

    value = value*63/100;
    qDebug() << "Volume: " << value;

    if (serial->isOpen()) {

        char cmd = 1;
//        value = (cmd << 16) | (value & ~0xFFFF0000);
        char buf[4] = {0, cmd, 0, (char) value};

        qDebug()  << "Sending data: " << QString(buf);

        qint64 length = serial->write(buf, 4);
        serial->flush();

        qDebug() << "Bytes sent: " << length;
        qDebug() << "";

//        QByteArray dataResp = serial->readAll();

        qint64 readSize = 0;
        qint64 available = serial->bytesAvailable();
        char respBuff[available];

        qDebug() << "Responce: ";
        ui->logBrowser->append("Responce: ");

        while(available > 0 && (readSize = serial->readLine(respBuff, available)) > 0) {
            QString respString = QString(respBuff);
            qDebug() << respString;
            ui->logBrowser->append(respString);
        }

        qDebug() << "";
        ui->logBrowser->append("\n");
    }
}

void Widget::on_balanceSlider_valueChanged(int value)
{
    value = value-31;
    qDebug() << "Balance: " << value;

    if (serial->isOpen()) {
        QByteArray data;
        data.append('4').append(std::to_string(value).c_str());
        serial->write(data);
        data.clear();

        data = serial->readAll();
        qDebug() << "Responce: " << QString(data);
    }
}


void Widget::on_muteCheckBox_toggled(bool value)
{
    qDebug() << "Mute: " << value;

    if (serial->isOpen()) {
        QByteArray data;
        data.append('6').append(value ? "1" : "0");
        serial->write(data);
        data.clear();
        data = serial->readAll();
        qDebug() << "Responce: " << QString(data);
    }
}


void Widget::on_trebleSlider_valueChanged(int value)
{
    value = (value-50)*31/50;
    qDebug() << "Treble: " << value;

    if (serial->isOpen()) {
        QByteArray data;
        data.append('2').append(std::to_string(value).c_str());
        serial->write(data);
        data.clear();

        data = serial->readAll();
        qDebug() << "Responce: " << QString(data);
    }
}


void Widget::on_bassSlider_valueChanged(int value)
{
    value = (value-50)*31/50;
    qDebug() << "Bass: " << value;

    if (serial->isOpen()) {
        QByteArray data;
        data.append('3').append(std::to_string(value).c_str());
        serial->write(data);
        data.clear();

        data = serial->readAll();
        qDebug() << "Responce: " << QString(data);
    }
}



void Widget::on_logClearBtn_pressed()
{
    ui->logBrowser->clear();
}


