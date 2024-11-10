#ifndef HANDLER_H
#define HANDLER_H

#include "exchange.h"

#include <QMutex>
#include <QThread>
#include <QtSerialPort/QSerialPort>


class DeviceMonitor;

class SerialHandler : public QObject
{
    Q_OBJECT

private:
    QSerialPort *m_serial;
    DeviceMonitor *m_devMonitor;

    bool checkSerial();

public:
    explicit SerialHandler(QObject *parent = nullptr);
    ~SerialHandler() override;

    QList<QString> availablePorts();
    void connectTo(const QString &portName);
    void disconnect();
    bool isConnected();
    void sendCommand(const CMD &cmd, int8_t value = 0);
    void sendCommand(const QString &value);

signals:
    void device_plugged();
    void connected();
    void disconnected();
    void connectionError(QString &error);
    void synced(Values &values);
    void valueEvent(CMD cmd, int8_t &value);

public slots:
    void handleError(QSerialPort::SerialPortError error);
    void processEvent();

};


class DeviceMonitor: public QObject
{
Q_OBJECT

private:
    SerialHandler &handler;
    volatile bool m_quit = false;

public:
    explicit DeviceMonitor(SerialHandler &handler);
    ~DeviceMonitor() override;


public slots:
    void init();
    void process();
    void stop();

//Q_SIGNALS
signals:
    void finished();
    void error(const QString& error);
};

#endif // HANDLER_H


