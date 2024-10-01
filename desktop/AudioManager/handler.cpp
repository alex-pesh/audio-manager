#include "handler.h"
#include <QApplication>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QDebug>

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cerrno>

SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {

    m_serial = new QSerialPort();
}

SerialHandler::~SerialHandler() {

    if (m_serial->isOpen()) {
        m_serial->close();
    }

    delete m_serial;
}


void serialInfo(const char *_portName) {

    int fd = open(_portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        qDebug() << errno;
    }

    int bytes = 0;
    int res = ioctl(fd, FIONREAD, &bytes);
    qDebug() << "Buffer size: " << bytes;

/*
    serial_struct serinfo;
    memset(&serinfo, 0, sizeof(serinfo));
    int res = ioctl(fd, TIOCGSERIAL, &serinfo);
    qDebug() << "Buffer size: " << serinfo.xmit_fifo_size;
*/

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

    if (!m_serial->open(QIODevice::ReadWrite)) {
        qDebug() << "Failed to open serial " << portName << ": " << m_serial->errorString();

        emit connectionError(QString::fromLatin1("Failed to open serial port ")
                                .append(portName).append(": ").append(m_serial->errorString()));
        return;
    } else {
        qDebug() << "Connected to " << portName;

        m_serial->clear(QSerialPort::AllDirections);
        sendCommand(CMD::SYNC);
        emit connected();
    }

//    QThread *thread = new QThread();
//    thread->setObjectName("Receiver_Thread");
    m_receiver = new Receiver(*m_serial);
//    m_receiver->moveToThread(thread);

//    connect(thread, SIGNAL(started()), m_receiver, SLOT(init()));
//    connect(thread, SIGNAL(finished()), thread, SLOT(deleteLater()));
//    connect(m_receiver, SIGNAL(finished()), thread, SLOT(quit()));

    connect(m_receiver, SIGNAL(finished()), m_receiver, SLOT(deleteLater()));
    connect(m_serial, SIGNAL(readyRead()), this, SLOT(processEvent()));
    connect(m_serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(handleError(QSerialPort::SerialPortError)));

//    thread->start();

}


QList<QString> SerialHandler::availablePorts() {
    QList<QString> portNames;
    QList<QSerialPortInfo> portInfos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : portInfos) {
//        qDebug() << info.systemLocation();
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


void SerialHandler::processEvent() {

    if (m_serial->bytesAvailable() > 0) {

        qDebug("---------- Response [%lld bytes] ----------", m_serial->bytesAvailable());

        QByteArray respBuff;
        CMD cmd;
        m_serial->read(reinterpret_cast<char *>(&cmd), sizeof(CMD)); // read command CMD

        std::string label;

        switch (cmd) {
            case SET_VOLUME:
            case SET_BASS:
            case SET_TREBLE: {
                int8_t value;
                m_serial->read(reinterpret_cast<char *>(&value), sizeof(int8_t)); // read command value
                emit SerialHandler::valueEvent(cmd, value);
                break;
            }

            case CUSTOM: {
                int8_t value;
                m_serial->peek(reinterpret_cast<char *>(&value), sizeof(int8_t)); // read command value
                qDebug("DATA: %d", value);
                break;
            }

            case DISCONNECT: {
                int8_t value;
                m_serial->read(reinterpret_cast<char *>(&value), sizeof(int8_t)); // read command value
                qDebug("Disconnect signal received: %d", value);
                disconnect();
                return;
            }

            case SYNC: {
                Values values;
                m_serial->read(reinterpret_cast<char *>(&values), sizeof(Values));
                emit SerialHandler::synced(values);
                qDebug("Sync values [Volume: %d; Treble: %d; Bass: %d; Balance: %d]",
                       values.volume, values.treble, values.bass, values.balance);
                break;
            }

            default:
                respBuff += cmd;
        }


        do {
            respBuff += m_serial->readAll();
        } while (m_serial->bytesAvailable() > 0);

        if (respBuff.size() > 0) {
            QString respString = QString(respBuff).trimmed();
            qDebug().noquote() << "<<" << respString;
        }
//        qDebug("Bytes received: %d", respBuff.size());
        qDebug("------------------------------");
    }
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
