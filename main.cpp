#include <QCoreApplication>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QElapsedTimer>
#include <QTimer>
#include <QDateTime>
#include "include/cppystruct.h"
#include "NumCpp.hpp"
#include<iostream>
#include <tuple>
//#include <QtMath>


using namespace std;

#define YIANKANG_MAJOR_VER 0
#define YIANKANG_MINOR_VER_ 1


bool serialPortConfig(QSerialPort *serial, qint32 baudRate, QString dataPortNum);
void sendCfg();
void serialstart();
nc::NdArray<float> readAndParseUart();
tuple<unsigned int,unsigned int> tlvHeaderDecode(QByteArray data);
void parseCapon3DPolar(QByteArray data,int tlvLength);
void polar2Cart3D();
void parseDetectedTracksSDK3x(QByteArray data,int tlvLength);
//QVector<QVector<double>> zeros(int sizeX, int sizeY);
//QVector<QVector<double>> ones(int sizeX, int sizeY,int multiple);

static QSerialPort *userport,*dataport;
static bool  FlagSerialPort_Connected, userPort_Connected,dataPort_Connected;
QString dataPortNum, userPortNum;   // Serial Port configuration
int frameTime = 50;
QTimer * timerRead;
QByteArray byteData("");
qint8 fail = 0;
int maxPoints = 1150,numDetectedTarget = 0,numDetectedObj = 0,frameNum = 0,missedFrames = 0;
QVector<int> indexes;
//QVector<QVector<double>> pcBufPing = zeros(5, maxPoints);
auto pcBufPing = nc::zeros<float>(5,maxPoints);
auto pcPolar = nc::zeros<float>(5, maxPoints);
auto targetBufPing = nc::ones<int>(10,20)*-1;
//QByteArray magicWord = "0x708050603040102";
unsigned long magicWord = 506660481457717506;


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
    //        qDebug()<<cfg_list.at(i)<<endl;
            userport->write(cfg_list.at(i).toLatin1());
            userport->write("\n");
            userport->waitForBytesWritten(200);

            QElapsedTimer t;
            t.start();
            while(t.elapsed()<100);
        }

        if(userPort_Connected)
        {
            timerRead = new QTimer(nullptr);
            timerRead->setInterval(frameTime);
            QObject::connect(timerRead,&QTimer::timeout,[=]{
                readAndParseUart();});
            timerRead->start();
    //        QObject::connect(dataport,&QSerialPort::readyRead,[=]{
    //            readAndParseUart();
    //        });
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
QByteArray Capon3DHeader(QByteArray dataIn)
    {
        pcBufPing = nc::zeros<float>(5,maxPoints);
        pcPolar = nc::zeros<float>(5,maxPoints);
        targetBufPing =  nc::zeros<int>(13,20);
        numDetectedTarget = 0;
        numDetectedObj = 0;
        indexes.clear();
        int tlvHeaderLength = 8,headerLength = 48;

         unsigned long long magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime;
         unsigned int  trackProcessTime, numTLVs;
         unsigned short checksum;
    //    qDebug()<<"Capon3DHeader:dataIn"<<dataIn.toHex();
        while(true)
        {
             auto [magic_t, version_t, packetLength_t, platform_t, frameNum_t, subFrameNum_t, chirpMargin_t, frameMargin_t, uartSentTime_t, trackProcessTime_t, numTLVs_t, checksum_t] = pystruct::unpack(PY_STRING("Q9I2H"), dataIn.left(headerLength));
    //         qDebug()<<dataIn.size()<<"magic_t"<<magic_t<<version_t<<packetLength_t<<platform_t<<frameNum_t<<subFrameNum_t<<chirpMargin_t<<frameMargin_t<<uartSentTime_t<<uartSentTime_t<<trackProcessTime_t<<numTLVs_t<<checksum_t;
             if(magic_t != magicWord && dataIn.size()>0)
                {
                qDebug()<<"magicword_flase";
                dataIn = dataIn.mid(1);
                }
             else
                {
                qDebug()<<"magicword_true";
                magic = magic_t;
                version = version_t;
                packetLength = packetLength_t;
                platform = platform_t;
                frameNum = frameNum_t;
                subFrameNum = subFrameNum_t;
                chirpMargin =chirpMargin_t;
                frameMargin = frameMargin_t;
                uartSentTime = uartSentTime_t;
                trackProcessTime= trackProcessTime_t;
                numTLVs =numTLVs_t;
                checksum = checksum_t;
                break;
                }
        }

        qDebug()<<dataIn.size()<<"magic"<<magic<<version<<"packetLength"<<packetLength<<platform<<"frameNum"<<frameNum<<subFrameNum<<chirpMargin<<frameMargin<<uartSentTime<<uartSentTime<<trackProcessTime<<"numTLVs"<<numTLVs<<checksum;

        dataIn = dataIn.mid(headerLength);
        int remainingData = packetLength - dataIn.length() - headerLength;
    //check to ensure we have all of the data
        if (remainingData > 0)
        {
            qDebug()<<"(remainingData > 0)"<<endl;
            QByteArray newData = dataport->read(remainingData);
            remainingData = packetLength -dataIn.length() -headerLength - newData.length();
            dataIn += newData;
        }
    //     now check TLVs
        for(unsigned int i = 0;i < numTLVs; i++)
        {

            //todo
            auto[tlvType,tlvLength] = tlvHeaderDecode(dataIn.left(tlvHeaderLength));
            dataIn =dataIn.mid(tlvHeaderLength);
            int  dataLength = tlvLength - tlvHeaderLength;

                 if(tlvType==6)
    //                DPIF Polar Coordinates
                    parseCapon3DPolar(dataIn.left(dataLength), dataLength);
    //                else if(tlvType==6)
    //                else if(tlvType==6)
    //                else if(tlvType==6)

            qDebug()<<"numTLVs"<<numTLVs<<"tlvType"<<tlvType<<"tlvlength"<<tlvLength<<"datalength"<<dataLength<<"dataIn.len"<<dataIn.length()<<endl;
            dataIn = dataIn.mid(dataLength);

        }
        //check here GLOBAL frame
    //    if(frameNum +1 != frameNum)
    //        missedFrames += frameNum - (frameNum+1);
        return dataIn;
    }
nc::NdArray<float> readAndParseUart()
    {
        qint64 numBytes = 4666;
        fail = 0;
        QByteArray data = dataport->readAll();
        qDebug()<<"readAndParseuart:"<<data.size()<<data<<endl;

        if(byteData.isEmpty())
            byteData = data;
        else
    //        byteData += data;
            byteData = data;

        byteData = Capon3DHeader(byteData);
    //    Capon3DHeader(byteData);
        if(fail)
            return pcBufPing;

        qint64 parseEnd=QDateTime::currentMSecsSinceEpoch();
        qDebug()<<"parseEnd"<<parseEnd;

        return pcBufPing;

    }
//QVector<QVector<double>> zeros(int sizeX, int sizeY)
//{
//   QVector<QVector<double>> result;
//   for (int idx1 = 0; idx1 < sizeX; idx1++)
//   {
//      result.append(QVector<double>());
//      for (int idx2 = 0; idx2 < sizeY; idx2++)
//      {
////         result[idx1].append(double());
//           result[idx1].append(double(0));
//      }
//   }
//   return result;
//}
//QVector<QVector<double>> ones(int sizeX, int sizeY,int multiple)
//{
//    QVector<QVector<double>> result;
//    for (int idx1 = 0; idx1 < sizeX; idx1++)
//    {
//       result.append(QVector<double>());
//       for (int idx2 = 0; idx2 < sizeY; idx2++)
//       {
//    //         result[idx1].append(double());
//            result[idx1].append(double(multiple));
//       }
//    }
//    return result;
//}
tuple<unsigned int,unsigned int> tlvHeaderDecode(QByteArray data)
    {
    //    qDebug()<<"in tlvHeaderDecode,"<<data.length();
        auto[tlvType,tlvLength] = pystruct::unpack(PY_STRING("2I"),data);
    //    qDebug()<<tlvType<<tlvLength;
        return tuple(tlvType,tlvLength);
    }

//     support for Capoin 3D point cloud  支持 Capoin 3D 点云
//     decode Capon 3D point Cloud TLV  解码 Capon 3D 点云 TLV
void parseCapon3DPolar(QByteArray data,int tlvLength)
    {
        auto pUnitStruct = PY_STRING("5f"); //elev, azim, doppler, range, snr
        auto pUnitSize = pystruct::calcsize(pUnitStruct);
        auto pUnit = pystruct::unpack(pUnitStruct,data.left(pUnitSize));
        data = data.mid(pUnitSize);
        auto objStruct = PY_STRING("2bh2H");//2 int8, 1 int16, 2 uint16,hyw此处python与c++格式含义不同
        auto objSize = pystruct::calcsize(objStruct);
        numDetectedObj = int((tlvLength-pUnitSize)/objSize);

        for(int i = 0;i<numDetectedObj;++i)
        {
            auto[elev, az, doppler, ran, snr] = pystruct::unpack(PY_STRING("2bh2H"),data.left(objSize));
            data = data.mid(objSize);
            //get range, azimuth, doppler, snr
//           qDebug()<< pcPolar[0,i];
//           qDebug()<< get<3>(pUnit);
            pcPolar[0,i] = ran * get<3>(pUnit);//#range
             if(az >= 128)
               {
                   qDebug()<<"Az greater than 127"<<endl;
                   az -= 256;
               }
            if(elev >= 128)
              {
                  qDebug()<<"Elev greater than 127"<<endl;
                  elev -= 256;
              }
            if(doppler >= 32768)
              {
                  qDebug()<<"Doppler greater than 32768"<<endl;
                  doppler -= 65536;
              }

             pcPolar[1,i] = az * get<1>(pUnit);//azimuth
             pcPolar[2,i] = elev * get<0>(pUnit);//elevation
             pcPolar[3,i] = doppler * get<2>(pUnit);//doppler
             pcPolar[4,i] = snr * get<4>(pUnit);//snr
        }
        polar2Cart3D();
    }
//        #convert 3D people counting polar to 3D cartesian  将 3D 人口计数极坐标转换为 3D 笛卡尔坐标
void polar2Cart3D()
        {
            pcBufPing = nc::empty<float>(5,numDetectedObj);
            for(int n = 0;n<numDetectedObj;++n)
            {
                pcBufPing[2,n] = pcPolar[0,n]*sin(pcPolar[2,n]);//z
                pcBufPing[0,n] = pcPolar[0,n]*cos(pcPolar[2,n])*sin(pcPolar[1,n]);//x
                pcBufPing[1,n] = pcPolar[0,n]*cos(pcPolar[2,n])*cos(pcPolar[1,n]);//y

             }
            pcBufPing[3,pcBufPing.rSlice()] = pcPolar[3,nc::Slice(0, numDetectedObj)];//doppler
            pcBufPing[4,pcBufPing.rSlice()] = pcPolar[4,nc::Slice(0, numDetectedObj)];//snr
            std::cout <<"polar2Cart3D_pcbufping"<<pcBufPing;
         }

//    #decode 3D People Counting Target List TLV  解码 3D 人数统计目标列表 TLV

//    #3D Struct format  3D 结构格式

//    #uint32_t     tid;     /*! @brief   tracking ID */
//    #float        posX;    /*! @brief   Detected target X coordinate, in m  检测到的目标 X 坐标，单位为 m*/
//    #float        posY;    /*! @brief   Detected target Y coordinate, in m  检测到的目标 Y 坐标，单位为 m */
//    #float        posZ;    /*! @brief   Detected target Z coordinate, in m  检测到的目标 Z 坐标，单位为 m*/
//    #float        velX;    /*! @brief   Detected target X velocity, in m/s  检测到的目标 X 速度，单位为 m/s*/
//    ##float       velY;    /*! @brief   Detected target Y velocity, in m/s  检测到的目标 Y 速度，单位为 m/s*/
//    #float        velZ;    /*! @brief   Detected target Z velocity, in m/s  检测到的目标 Z 速度，单位为 m/s*/
//    #float        accX;    /*! @brief   Detected target X acceleration, in m/s2 检测到的目标 X 加速度，单位为 m/s2*/
//    #float        accY;    /*! @brief   Detected target Y acceleration, in m/s2 检测到的目标 Y 加速度，单位为 m/s2*/
//    #float        accZ;    /*! @brief   Detected target Z acceleration, in m/s2 检测到的目标 Z 加速度，单位为 m/s2*/
//    #float        ec[16];  /*! @brief   Target Error covarience matrix, [4x4 float], in row major order, range, azimuth, elev, doppler */
//    #目标误差协方差矩阵，[4x4 浮点数]，按行主序、距离、方位角、高程、多普勒
//    #float        g;
//    #float        confidenceLevel;    /*! @brief   Tracker confidence metric 跟踪器置信度指标*/
void parseDetectedTracksSDK3x(QByteArray data,int tlvLength)
{
    auto targetStruct = PY_STRING("I27f");
    auto targetSize = pystruct::calcsize(targetStruct);
    numDetectedTarget = int(tlvLength/targetSize);
    auto targets = nc::empty<float>(16,numDetectedTarget);
    auto rotTarget = {0,0,0};
    for(int i = 0;i<=numDetectedTarget;++i)
    {
        auto targetData = pystruct::unpack(targetStruct,data.left(targetSize));
        //tid,pos x,pos y
        targets[nc::Slice(0,3),i]=get<0>(targetData),get<1>(targetData),get<2>(targetData),get<3>(targetData);
        //pos z
        targets[3,i] = get<3>(targetData);
      std::cout<<"parseDetectedTracksSDK3x"<<get<0>(targetData)<<get<26>(targetData)<<endl;
    }
}
int main(int argc, char *argv[])
    {
        QCoreApplication a(argc, argv);
        qDebug()<<"byteData"<<byteData<<endl;
        std::cout <<"pcBufPing"<<pcBufPing<<targetBufPing<< std::endl;
        qDebug()<<"falldetection_version:"<<YIANKANG_MAJOR_VER<<"."<<YIANKANG_MINOR_VER_;

        serialstart();

        return a.exec();
    }
