#ifndef HANDLER_H
#define HANDLER_H

#include <QMutex>
#include <QThread>
#include <QtSerialPort/QSerialPort>


enum CMD {
    SET_VOLUME,
    SET_BALANCE,
    SET_TREBLE,
    SET_BASS
};


class ReceiverThread : public QThread
{

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

public:
    explicit SerialHandler(QObject *parent = nullptr);
    ~SerialHandler() override;

    void connectTo(const QString &portName);
    void disconnect();
    void sendCommand(const CMD &cmd, int value);

signals:
    void connectionError(const QString &error);


};


#endif // HANDLER_H
