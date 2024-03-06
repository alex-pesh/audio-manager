#ifndef HANDLER_H
#define HANDLER_H

#include <QMutex>
#include <QThread>
#include <QtSerialPort/QSerialPort>
#include <type_traits>


enum CMD {
    SET_VOLUME = 1,
    SET_TREBLE,
    SET_BASS,
    SET_BALANCE,
    SET_MUTE,
    SET_LOUDNESS,
    SYNC = 100,
    CUSTOM = 200
};


class Receiver: public QObject
{
    Q_OBJECT

private:
    QSerialPort *m_serial;
    volatile bool m_quit = false;

public:
    explicit Receiver(QSerialPort &m_serial);
    ~Receiver() override;


public slots:
    void process();
    void stop();

//Q_SIGNALS
signals:
    void finished();
    void error(QString error);
};



class SerialHandler : public QObject
{
    Q_OBJECT

private:
    QSerialPort *m_serial;
    Receiver *m_receiver;

    bool checkSerial();

public:
    explicit SerialHandler(QObject *parent = nullptr);
    ~SerialHandler() override;

    QList<QString> availablePorts();
    void connectTo(const QString &portName);
    void disconnect();
    bool isConnected();
    void sendCommand(const CMD &cmd, int16_t value);
    void sendCommand(const QString &value);

signals:
    void connected();
    void disconnected();
    void connectionError(QString &error);

public slots:
    void handleError(QSerialPort::SerialPortError error);

};


#endif // HANDLER_H
