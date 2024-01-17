#include "handler.h"
#include <QtSerialPort/QSerialPort>
#include <QThread>

#include <QDebug>

SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {

    m_serial = new QSerialPort();
    m_receiver = new Receiver(*m_serial);
}

SerialHandler::~SerialHandler() {

    if (m_serial->isOpen()) {
        m_serial->close();
    }

    delete m_serial;
//    delete m_receiver;
}


void SerialHandler::connectTo(const QString& portName) {

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
    }


    QThread *thread = new QThread();
    Receiver *receiver = m_receiver; //new Receiver() ;
    receiver->moveToThread(thread);

//    connect(receiver, SIGNAL(error(QStrin)), this, SLOT(onReceiverError(QString)));
    connect(thread, SIGNAL(started()), receiver, SLOT(process()));
    connect(receiver, SIGNAL(finished()), thread, SLOT(quit()));
    connect(receiver, SIGNAL(finished()), receiver, SLOT(deleteLater()));
    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));

    thread->start();
}


void SerialHandler::disconnect() {
    m_receiver->stop();
    m_serial->close();
}


void SerialHandler::sendCommand(const CMD &cmd, const int value) {
    if (m_serial->isOpen()) {

        char cmd = 1;
//        value = (cmd << 16) | (value & ~0xFFFF0000);
        char buf[4] = {0, cmd, 0, (char) value};

        qDebug()  << "Sending data: " << QString(buf);

        qint64 length = m_serial->write(buf, 4);
        m_serial->flush();

        qDebug() << "Bytes sent: " << length;
        qDebug() << "";
    }
}


Receiver::Receiver(QSerialPort &serial) {
    m_serial = &serial;
}

Receiver::~Receiver() {

}

void Receiver::process() {
    qint64 readSize = 0;

    while (!m_quit) {

        qint64 available = m_serial->bytesAvailable();
        if (available > 0) {
            qDebug() << "Response: ";

            char respBuff[available];
            while (available > 0 && (readSize = m_serial->readLine(respBuff, available)) > 0) {
                QThread::msleep(100);
                QString respString = QString(respBuff);
                qDebug() << respString;
            }

            qDebug() << "-----------------------";

        }


        continue;

        while (m_serial->waitForReadyRead(100)) {
//            qDebug() << "Response: ";

            QByteArray resp = m_serial->readAll();
            qDebug().noquote() << QString::fromUtf8(resp);

            continue;

            qint64 available = m_serial->bytesAvailable();
            char respBuff[available];

            while ((readSize = m_serial->readLine(respBuff, available)) > 0) {
                QString respString = QString(respBuff);
                qDebug() << respString;
            }

//            qDebug() << "-----------------------";

        }


    }

    emit finished();
}


void Receiver::stop() {
    m_quit = true;
}


