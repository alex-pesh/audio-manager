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
    , handler(new SerialHandler())

{

//    handler->connectTo("/dev/ttyUSB0");
//    handler->connectTo("/tmp/simavr-uart0-tap");
//    handler->connectTo("/tmp/simavr-uart0");

    move(qApp->primaryScreen()->availableGeometry().center()-rect().center());

    ui->setupUi(this);

    connect(handler, &SerialHandler::device_plugged, this, &Widget::on_device_plugged);
    connect(handler, &SerialHandler::connected, this, &Widget::on_connect);
    connect(handler, &SerialHandler::disconnected, this, &Widget::on_disconnect);
    connect(handler, &SerialHandler::connectionError, this, &Widget::on_connectionError);
    connect(handler, &SerialHandler::synced, this, &Widget::on_syncEvent);
    connect(handler, &SerialHandler::valueEvent, this, &Widget::on_valueEvent);

    ui->deviceComboBox->addItems(handler->availablePorts());
//    handler->connectTo(ui->deviceComboBox->currentText());
}


Widget::~Widget() {
    delete handler;
    delete ui;
}

void Widget::showEvent(QShowEvent *event) {

}

void Widget::refreshDevicesList() {
    ui->deviceComboBox->clear();
    ui->deviceComboBox->addItems(handler->availablePorts());
}

void Widget::on_device_plugged() {
    refreshDevicesList();
}


void Widget::on_connect() {
    ui->controlGroupBox->setEnabled(true);
    ui->connectCheckBox->setChecked(true);
}

void Widget::on_disconnect() {
    ui->controlGroupBox->setEnabled(false);
    ui->connectCheckBox->setChecked(false);
}


void Widget::on_connectionError(QString &error)
{
    ui->controlGroupBox->setEnabled(false);
    ui->connectCheckBox->setCheckState(Qt::Unchecked);

    MessageBox msgBox;
//    msgBox.setParent(this);
    msgBox.setText("Error");
    msgBox.setInformativeText(error);
    msgBox.setStandardButtons(MessageBox::Ok);
    msgBox.setIcon(MessageBox::Icon::Critical);

    int result = msgBox.exec();
    if (result== MessageBox::Ok) {
//        exit(1);
    }
}


void Widget::on_syncEvent(Values &values) {
    setSliderValue(ui->volumeSlider, values.volume*100/63);
    setSliderValue(ui->trebleSlider, (values.treble+7)*100/14);
    setSliderValue(ui->bassSlider, (values.bass+7)*100/14);
    setSliderValue(ui->balanceSlider, values.balance+31);

    switch (values.source) {
        case 0:
            ui->sourceCdBtn->setChecked(true);
            break;
        case 1:
            ui->sourcePcBtn->setChecked(true);
            break;
        case 2:
            ui->sourceAuxBtn->setChecked(true);
            break;
    }
}

void Widget::on_valueEvent(CMD cmd, int8_t &value) {
    switch (cmd) {
        case SET_VOLUME:
            setSliderValue(ui->volumeSlider, value*100/63);
            break;

        case SET_TREBLE:
            setSliderValue(ui->trebleSlider, (value+7)*100/14);
            break;

        case SET_BASS:
            setSliderValue(ui->bassSlider, (value+7)*100/14);
            break;
    }
}


void Widget::setSliderValue(QObject *slider, const int &value) {
    const bool wasBlocked = slider->blockSignals(true);
    ((QSlider*) slider)->setValue(value);
    slider->blockSignals(wasBlocked);
}



void Widget::on_deviceComboBox_currentTextChanged(const QString &deviceName)
{
    qDebug() << "Device changed: " << deviceName;
    on_connectCheckBox_toggled(false);
}


void Widget::on_connectCheckBox_toggled(bool checked)
{
    if (checked && !handler->isConnected()) {
        handler->connectTo(ui->deviceComboBox->currentText());
    } else if (handler->isConnected()) {
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


void Widget::on_sendBtn_pressed()
{
    QString command = ui->cmdValueText->toPlainText();

    if (command.length() > 0) {
        handler->sendCommand(command);
        qDebug() << "Sending command: " << command << endl;
    }
}


void Widget::on_sourceAuxBtn_clicked(bool checked)
{
    if (checked) {
        int8_t value = 2;
        qDebug() << "Source: " << value;
        handler->sendCommand(CMD::SET_SOURCE, value);
    }
}

void Widget::on_sourcePcBtn_clicked(bool checked)
{
    if (checked) {
        int8_t value = 1;
        qDebug() << "Source: " << value;
        handler->sendCommand(CMD::SET_SOURCE, value);
    }
}

void Widget::on_sourceCdBtn_clicked(bool checked)
{
    if (checked) {
        int8_t value = 0;
        qDebug() << "Source: " << value;
        handler->sendCommand(CMD::SET_SOURCE, value);
    }
}
