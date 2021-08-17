#include <QCoreApplication>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QElapsedTimer>
#include <QTimer>


#define YIANKANG_MAJOR_VER 0
#define YIANKANG_MINOR_VER_ 1


bool serialPortConfig(QSerialPort *serial, qint32 baudRate, QString dataPortNum);
void sendCfg();
void serialstart();
void readAndParseUart();

static QSerialPort *userport,*dataport;
static bool  FlagSerialPort_Connected, userPort_Connected,dataPort_Connected;
QString dataPortNum, userPortNum;   // Serial Port configuration
int frameTime = 50;
QTimer * timerRead;

bool serialPortConfig(QSerialPort *serial, qint32 baudRate, QString dataPortNum)
{
    qDebug()<<"serialPortConfig"<<endl;
    serial->setPortName(dataPortNum);
    if(serial->open(QIODevice::ReadWrite) )
    {
        FlagSerialPort_Connected = 1;
    }
    else
    {
        FlagSerialPort_Connected = 0;
        return FlagSerialPort_Connected;
    }
    serial->setBaudRate(baudRate);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    return FlagSerialPort_Connected;
}
void sendCfg()
{
    qDebug()<<"sendCfg"<<endl;
    QStringList cfg_list;

    QObject::connect(userport,&QSerialPort::readyRead,[=]{
        QByteArray receiveArray = userport->readAll();
        qDebug()<<"userport_reply:"<<receiveArray;});

    cfg_list<<"sensorStop"
    <<"flushCfg"
    <<"dfeDataOutputMode 1"
    <<"channelCfg 15 7 0"
    <<"adcCfg 2 1"
    <<"adcbufCfg -1 0 1 1 1"
    <<"lowPower 0 0"
    <<"profileCfg 0 60.75 30.00 25.00 59.10 394758 0 54.71 1   96 2950.00 2 1 36"
    <<"chirpCfg 0 0 0 0 0 0 0 1"
    <<"chirpCfg 1 1 0 0 0 0 0 2"
    <<"chirpCfg 2 2 0 0 0 0 0 4"
    <<"frameCfg 0 2  96  0 55.00 1 0"
    <<"dynamicRACfarCfg -1 4 4 2 2 8 12 4 8 5.00  8.00 0.40 1 1"
    <<"staticRACfarCfg -1 6 2 2 2 8 8 6 4 8.00 15.00 0.30 0 0"
    <<"dynamicRangeAngleCfg -1 0.75 0.0010 1 0"
    <<"dynamic2DAngleCfg -1 1.5 0.0300 1 0 1 0.30 0.85 8.00"
    <<"staticRangeAngleCfg -1 0 8 8"
    <<"antGeometry0 -1 -1 0 0 -3 -3 -2 -2 -1 -1 0 0"
    <<"antGeometry1 -1 0 -1 0 -3 -2 -3 -2 -3 -2 -3 -2"
    <<"antPhaseRot 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1"
    <<"fovCfg -1 70.0 20.0"
    <<"compRangeBiasAndRxChanPhase 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 "
    <<"staticBoundaryBox -3 3 2 5.5 0 3"
    <<"boundaryBox -4 4 0.5 6 0 3"
    <<"sensorPosition 2 0 15"
    <<"gatingParam 3 2 2 2 4"
    <<"stateParam 3 3 6 500 5 6000"
    <<"allocationParam 40 100 0.1 20 0.5 20"
    <<"maxAcceleration 0.1 0.1 0.1"
    <<"trackingCfg 1 2 800 30 46 96 55"
    <<"presenceBoundaryBox -4 4 0.5 6 0 3"
    <<"sensorStart";
    for(int i = 0;i < cfg_list.size();++i)
    {
        qDebug()<<cfg_list.at(i)<<endl;
        userport->write(cfg_list.at(i).toLatin1());
        userport->write("\n");
        userport->waitForBytesWritten(200);

        QElapsedTimer t;
        t.start();
        while(t.elapsed()<500);
    }

    if(userPort_Connected)
    {
        timerRead = new QTimer(nullptr);
        timerRead->setInterval(frameTime);
        QObject::connect(timerRead,&QTimer::timeout,[=]{
            readAndParseUart();});
        timerRead->start();
    }
}
void serialstart()
{
    qDebug()<<"serialstart"<<endl;
    static qint32 cou_i = 0;
    foreach (const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts())
    {
        if (cou_i == 0)
            userPortNum = serialPortInfo.portName();
        else
            dataPortNum = serialPortInfo.portName();
        cou_i++;
    }
    userport = new QSerialPort;
    userPort_Connected = serialPortConfig(userport, 115200, userPortNum);
    dataport = new QSerialPort;
    dataPort_Connected = serialPortConfig(dataport, 921600, dataPortNum);

    if(!dataPort_Connected || !userPort_Connected)
    {
        qDebug()<<"userPort_Connected:"<<userPort_Connected<<"dataPort_Connected:"<<dataPort_Connected;
        cou_i = 0;
        serialstart();
    }
    else
    {
        qDebug()<<"userPort_Connected:"<<userPortNum<<"dataPort_Connected:"<<dataPortNum<<endl;
        sendCfg();
    }
}
void readAndParseUart()
{
    qint64 numBytes = 4666;
    QByteArray byteData;
    QByteArray data = dataport->read(numBytes);
    qDebug()<<"readAndParseuart:"<<data<<endl;

    if(byteData.isNull() || byteData.isEmpty())
        byteData = data;
    else
        byteData += data;

//    byteData = Capon3DHeader(byteData);
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    serialstart();

    return a.exec();
}
