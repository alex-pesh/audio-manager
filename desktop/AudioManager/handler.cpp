#include "handler.h"
#include <QApplication>
#include <QtSerialPort/QSerialPort>
#include <QThread>

#include <QDebug>

SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {

    m_serial = new QSerialPort();
}

SerialHandler::~SerialHandler() {

    if (m_serial->isOpen()) {
        m_serial->close();
    }

    delete m_serial;
    delete m_receiver;
}


void SerialHandler::connectTo(const QString& portName) {

    if (m_serial && m_serial->isOpen()) {
        disconnect();
    }

    m_serial->setPortName(portName);
    m_serial->setBaudRate(QSerialPort::BaudRate::Baud9600);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serial->open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open serial " << portName << ": " << m_serial->errorString();

        emit connectionError(QString::fromLatin1("Failed to open serial port ")
                                .append(portName).append(": ").append(m_serial->errorString()));
    } else {
        emit connected();
    }


    QThread *thread = new QThread();
    thread->setObjectName("Receiver_Thread");
    m_receiver = new Receiver(*m_serial); //new Receiver() ;
    m_receiver->moveToThread(thread);

//    connect(receiver, SIGNAL(error(QStrin)), this, SLOT(onReceiverError(QString)));
    connect(thread, SIGNAL(started()), m_receiver, SLOT(process()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
    connect(m_receiver, SIGNAL(finished()), thread, SLOT(quit()));
    connect(m_receiver, SIGNAL(finished()), m_receiver, SLOT(deleteLater()));
    connect(m_serial, SIGNAL(readyRead()), m_receiver, SLOT(process()));

    thread->start();
}


void SerialHandler::disconnect() {
    if (!m_serial->isOpen()) {
        return;
    }
    m_receiver->stop();
    m_serial->close();

    emit disconnected();
}

bool SerialHandler::isConnected() {
    return m_serial->isOpen();
}


void SerialHandler::sendCommand(const CMD &cmd, const int16_t value) {
    if (m_serial->isOpen()) {
        qDebug() << "------ Sending command ------";
        qDebug() << "CMD: " << cmd << "; Value: " << value;

        char buf[4] = {(char) cmd, 0, (char) value};
//        uint32_t data = (cmd) | (value & ~0xFFFF0000) << 16;
//        memccpy(buf, &data, '\n', sizeof (int));

        qint64 length = m_serial->write(buf, 4);
        m_serial->flush();

        qDebug() << "Bytes sent: " << length;
        qDebug() << "----------------------------" << endl;
    }
}

void SerialHandler::sendCommand(const QString &input) {
    QStringList values = input.split(QRegExp("\\s+"));
    int32_t inputCmd = values.at(0).toInt();
    int16_t value = (int16_t) values.at(1).toInt();

    CMD cmd = CMD(inputCmd);
    if (!cmd) {
        cmd = CMD::CUSTOM;
    }

    sendCommand(cmd, value);
}


Receiver::Receiver(QSerialPort &serial) {
    m_serial = &serial;
}

Receiver::~Receiver() {
    qDebug() << "Receiver is destructed";
}

void Receiver::process() {

    if (!m_quit && m_serial->bytesAvailable() > 0) {

        qDebug() << "---------- Response ----------";

        QByteArray respBuff;
        do {
            respBuff += m_serial->readAll();
            QThread::msleep(200);
        } while (!m_quit && m_serial->bytesAvailable() > 0);

        QString respString = QString(respBuff).trimmed();

        qDebug() << "Size: " << respBuff.size();
        qDebug().noquote() << respString;

        qDebug() << "-------------------------------" << endl;
    }

}


void Receiver::stop() {
    m_quit = true;
    emit finished();
}


