#include "RM.h"

void start()
{
    SDA_High();
    SCL_High();
    SDA_Low();
    SCL_Low();
}

void stop()
{
    SCL_Low();
    SDA_Low();
    SCL_High();
    SDA_High();
}

void SendByte(char n)
{
    for(int i=0;i<8;i++)
    {
        SCL_Low();
        if(n & 0x80)
        SDA_High();
        else
        SDA_Low();
        SCL_High();
        n = n<<1;
    }
}

char ReceiveAck()
{
    char ack;
    SCL_Low();
    SDA_High();
    SCL_High();
    if(SDA_Read())
    ack = 1;
    else
    ack = 0;
    return ack;
}

void SendData(char data)
{
    start();
    SendByte(data);
    ReceiveAck();
    stop();
}
 