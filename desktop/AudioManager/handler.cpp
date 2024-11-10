#include "handler.h"
#include <QApplication>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QThread>
#include <QDebug>

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>

#include <libudev.h>
#include <poll.h>


SerialHandler::SerialHandler(QObject *parent) : QObject(parent) {

    m_serial = new QSerialPort();

    QThread *thread = new QThread();
    thread->setObjectName("Receiver_Thread");
    m_devMonitor = new DeviceMonitor(*this);
    m_devMonitor->moveToThread(thread);

    connect(thread, &QThread::started, m_devMonitor, &DeviceMonitor::process);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);
    connect(m_devMonitor, &DeviceMonitor::finished, thread, &QThread::quit);
    connect(m_devMonitor, &DeviceMonitor::finished, m_devMonitor, &DeviceMonitor::deleteLater);
    thread->start();

}

SerialHandler::~SerialHandler() {
    if (m_serial && m_serial->isOpen()) {
        m_serial->close();
    }

    delete m_devMonitor;
    delete m_serial;
}


void serialInfo(const char *_portName) {

//    struct udev_monitor *udev_monitor = udev_monitor_new_from_netlink(udev, _portName);
//    if (!udev_monitor) {
//        fprintf(stderr, "udev_monitor_new_from_netlink returned NULL\n");
//        exit(EXIT_FAILURE);
//    }


    int fd = open(_portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        qDebug() << errno;
        return;
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

    close(fd);

}

void SerialHandler::connectTo(const QString& portName) {

//    serialInfo(portName.toStdString().c_str());

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
        emit connected();
    }

    connect(m_serial, SIGNAL(readyRead()), this, SLOT(processEvent()));
    connect(m_serial, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(handleError(QSerialPort::SerialPortError)));

    m_serial->clear(QSerialPort::AllDirections);
    sendCommand(CMD::SYNC);

}


QList<QString> SerialHandler::availablePorts() {
    QList<QString> portNames;
    QList<QSerialPortInfo> portInfos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : portInfos) {
        portNames.append(info.systemLocation());
/*
        qDebug() << info.systemLocation();
        qDebug() << "manufacturer: " << info.manufacturer();
        qDebug() << "description: " << info.description();
        qDebug() << "manufacturer: " << info.manufacturer();
        qDebug() << "productIdentifier: " << info.productIdentifier();
        qDebug() << "serialNumber: " << info.serialNumber();
        qDebug() << "vendorIdentifier: " << info.vendorIdentifier();
        qDebug() << "isValid: " << info.isValid() << endl;
*/
    }

    return portNames;
}

void SerialHandler::disconnect() {
    if (!m_serial->isOpen()) {
        return;
    }
    m_serial->close();

    emit disconnected();
    qDebug() << "Disconnected from " << m_serial->portName();
}

bool SerialHandler::isConnected() {
    return m_serial->isOpen();
}

bool SerialHandler::checkSerial() {
    QSerialPortInfo *portInfo = new QSerialPortInfo(m_serial->portName());
    if (portInfo->isValid()) {
        return true;
    } else {
        return false;
    }
}


void SerialHandler::sendCommand(const CMD &cmd, const int8_t value) {
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


void SerialHandler::handleError(QSerialPort::SerialPortError error) {
    if (error == QSerialPort::ResourceError) {
        qDebug() << "Connection error " << m_serial->portName() << ": " << m_serial->errorString();

        emit connectionError(QString::fromLatin1("Connection error ")
                                     .append(m_serial->portName()).append(": ").append(m_serial->errorString()));

        disconnect();
    }
}


void SerialHandler::processEvent() {

    if (m_serial->bytesAvailable() > 0) {

        qDebug("---------- Response [%lld bytes] ----------", m_serial->bytesAvailable());

        QByteArray respBuff;
        CMD cmd;
        m_serial->read(reinterpret_cast<char *>(&cmd), sizeof(CMD)); // read command CMD

        std::string label;

        switch (cmd) {
            case SET_SOURCE:
            case SET_VOLUME:
            case SET_BALANCE:
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
                qDebug("Sync values [Volume: %d; Treble: %d; Bass: %d; Balance: %d; Loudness: %d; Gain: %d; Source: %d]",
                       values.volume, values.treble, values.bass, values.balance, values.mute_loud, values.gain, values.source);
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
        qDebug() << "------------------------------" << endl;
    }
}



DeviceMonitor::DeviceMonitor(SerialHandler &handler) : handler(handler) {
    qDebug() << "DeviceMonitor is created";
}

DeviceMonitor::~DeviceMonitor() {
    qDebug() << "DeviceMonitor is destructed";
}


void DeviceMonitor::init() {
    qDebug() << "Initialized monitor thread";
}


void DeviceMonitor::process() {

//    https://www.freedesktop.org/software/systemd/man/latest/libudev.html
//    https://ftp.ntu.edu.tw/linux/utils/kernel/hotplug/libudev/libudev-udev-monitor.html


//    const char *UDEV_MONITOR_NAME = "/dev/ttyUSB0";

    udev *udev = udev_new();

//    struct udev_enumerate *devices = udev_enumerate_new(udev);

    udev_monitor* hotplug_monitor = udev_monitor_new_from_netlink(udev, "udev");
    udev_monitor_enable_receiving(hotplug_monitor);

    pollfd fd{
            .fd = udev_monitor_get_fd(hotplug_monitor),
            .events = POLLIN,
            .revents = 0
    };

    while( poll(&fd, 1, -1) > 0 ) {

        // receive the relevant device
        udev_device* dev = udev_monitor_receive_device(hotplug_monitor);
        if(!dev) {
            continue;
        }

        const char *action = udev_device_get_action(dev);
        const char *subsystem = udev_device_get_subsystem(dev);

        if  (!strcmp(subsystem, "usb-serial") &&
            (!strcmp(action, "bind") || !strcmp(action, "unbind"))) {

            handler.emit device_plugged();
        }

/*
        qDebug() << "hotplug[" << fd.revents << "]";
        qDebug() << "action: " << udev_device_get_action(dev);
        qDebug() << "devnode: " << udev_device_get_devnode(dev);
        qDebug() << "subsystem: " << udev_device_get_subsystem(dev);
        qDebug() << "devtype: " << udev_device_get_devtype(dev);
        qDebug() << "driver: " << udev_device_get_driver(dev);
        qDebug() << "devpath: " << udev_device_get_devpath(dev);
        qDebug() << "syspath: " << udev_device_get_syspath(dev);
        qDebug() << "sysname: " << udev_device_get_sysname(dev);
        qDebug() << "sysnum: " << udev_device_get_sysnum(dev);
        qDebug() << "initialized: " << udev_device_get_is_initialized(dev);

        udev_list_entry *devlinks = udev_device_get_devlinks_list_entry(dev);
        udev_list_entry *properties_entry = udev_device_get_properties_list_entry(dev);
        udev_list_entry *tags = udev_device_get_tags_list_entry(dev);
        udev_list_entry *sysattrs = udev_device_get_sysattr_list_entry(dev);


        while (devlinks) {
            qDebug() << "   Link " << udev_list_entry_get_name(devlinks)  << ": " << udev_list_entry_get_value(devlinks);
            devlinks = udev_list_entry_get_next(devlinks);
        }

        while (properties_entry) {
            qDebug() << "   Prop " << udev_list_entry_get_name(properties_entry)  << ": " << udev_list_entry_get_value(properties_entry);
            properties_entry = udev_list_entry_get_next(properties_entry);
        }

        while (tags) {
            qDebug() << "   Tag " << udev_list_entry_get_name(tags)  << ": " << udev_list_entry_get_value(tags);
            tags = udev_list_entry_get_next(tags);
        }

        while (sysattrs) {
            qDebug() << "   Attr " << udev_list_entry_get_name(sysattrs)  << ": " << udev_list_entry_get_value(sysattrs);
            sysattrs = udev_list_entry_get_next(sysattrs);
        }
*/

        udev_device_unref(dev);
        fd.revents = 0;
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


void DeviceMonitor::stop() {
    m_quit = true;
    emit finished();
}
