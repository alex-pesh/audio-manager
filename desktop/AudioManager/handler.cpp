#include "handler.h"
#include <QApplication>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QDebug>

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>

SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {

    m_serial = new QSerialPort();
}

SerialHandler::~SerialHandler() {

    if (m_serial->isOpen()) {
        m_serial->close();
    }

    delete m_serial;
}


void SerialHandler::connectTo(const QString& portName) {

    if (m_serial && m_serial->isOpen()) {
        disconnect();
    }

    m_serial->setPortName(portName);
    m_serial->setBaudRate(QSerialPort::BaudRate::Baud9600);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setStopBits(QSerialPort::TwoStop);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);
    m_serial->clear(QSerialPort::AllDirections);

    if (!m_serial->open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open serial " << portName << ": " << m_serial->errorString();

        emit connectionError(QString::fromLatin1("Failed to open serial port ")
                                .append(portName).append(": ").append(m_serial->errorString()));
    } else {
        emit connected();
    }

/*

    const char *_portName = qPrintable(portName);
    int fd = open(_portName, O_RDWR | O_NOCTTY | O_NDELAY);
//    fcntl(fd, F_SETFL, O_NONBLOCK);
    if (fd < 0) {
        qDebug() << errno;
    }

    int bytes = 0;
    int res = ioctl(fd, FIONREAD, &bytes);
    qDebug() << "Buffer size: " << bytes;*/


/*
    serial_struct serinfo;
    memset(&serinfo, 0, sizeof(serinfo));
    int res = ioctl(fd, TIOCGSERIAL, &serinfo);
    qDebug() << "Buffer size: " << serinfo.xmit_fifo_size;
*/


//    QThread *thread = new QThread();
//    thread->setObjectName("Receiver_Thread");
    m_receiver = new Receiver(*m_serial);
//    m_receiver->moveToThread(thread);

//    connect(thread, SIGNAL(started()), m_receiver, SLOT(init()));
//    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
//    connect(m_receiver, SIGNAL(finished()), thread, SLOT(quit()));
    connect(m_receiver, SIGNAL(finished()), m_receiver, SLOT(deleteLater()));
    connect(m_serial, SIGNAL(readyRead()), m_receiver, SLOT(process()));
    connect(m_serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(handleError(QSerialPort::SerialPortError)));
//    thread->start();
}


QList<QString> SerialHandler::availablePorts() {
    QList<QString> portNames;
    QList<QSerialPortInfo> portInfos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : portInfos) {
        qDebug() << info.systemLocation();
        portNames.append(info.systemLocation());
    }

    return portNames;
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

bool SerialHandler::checkSerial()
{
    QSerialPortInfo *portInfo = new QSerialPortInfo(m_serial->portName());
    // ui->serialDevice being a combobox of available serial ports

    if (portInfo->isValid())
    {
        return true;
    }
    else
    {
        return false;
    }
}

void SerialHandler::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        qDebug() << "Connection error " << m_serial->portName() << ": " << m_serial->errorString();

        emit connectionError(QString::fromLatin1("Connection error ")
                                     .append(m_serial->portName()).append(": ").append(m_serial->errorString()));

        disconnect();
    }
}

void SerialHandler::sendCommand(const CMD &cmd, const int16_t value) {
    if (m_serial->isOpen()) {
        qDebug() << "------ Sending command ------";
        qDebug(">> CMD: %d ; Value: %d", cmd, value);

        char buf[4] = {(char) cmd, 0, (char) value};
//        uint32_t data = (cmd) | (value & ~0xFFFF0000) << 16;
//        memccpy(buf, &data, '\n', sizeof (int));

        qint64 length = m_serial->write(buf, 4);
        m_serial->flush();

        qDebug("Bytes sent: %lld", length);
        qDebug() << "-----------------------------" << endl;
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
    this->m_serial = &serial;
    qDebug() << "Receiver is created";
}

Receiver::~Receiver() {
    qDebug() << "Receiver is destructed";
}


void Receiver::init() {
    qDebug() << "Initialized receiver thread";
}


void Receiver::process() {

    qint64 available = 0;
    if (!m_quit && (available = m_serial->bytesAvailable()) > 0) {

        qDebug("---------- Response ----------");
/*
        QByteArray respBuff = m_serial->read(8); // read command CMD
        CMD cmd = (CMD) *respBuff.data();
        respBuff.clear();
*/

        QThread::msleep(100);
        QByteArray respBuff;
        do {
            respBuff += m_serial->readAll();
        } while (!m_quit && m_serial->bytesAvailable() > 0);

        QString respString = QString(respBuff).trimmed();
        qDebug().noquote() << "<<" << respString;

        qDebug("Bytes received: %d", respBuff.size());
        qDebug("------------------------------");
    }

/*

    if (!m_quit && m_serial->bytesAvailable() < sizeof (DataHead)) {
        return;
    }

    DataHead head{};
    m_serial->read((char *) &head, sizeof(head));

    if (head.type == MESSAGE && head.length > 0) {
        QByteArray respBuff = m_serial->read(head.length);

        DataPacket packet{.head = head, .payload = new char[head.length]};
        memcpy(packet.payload, respBuff.data(), head.length);

        QString respString = QString(packet.payload).trimmed();
        free(packet.payload);

        qDebug() << "Size: " << respBuff.size();
        qDebug().noquote() << respString;
    }
    qDebug() << "-------------------------------" << endl;
*/

}


void Receiver::stop() {
    m_quit = true;
    emit finished();
}
