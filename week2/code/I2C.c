#include "RM.h"
#include "stm32f1xx_hal_conf.h"
void start()
{
    SDA_High();
    SCL_High();
    HAL_Delay(5);

    SDA_Low();
    HAL_Delay(5);
    SCL_Low();
    HAL_Delay(5);
}

void stop()
{
    SCL_Low();
    SDA_Low();
    HAL_Delay(5);

    SCL_High();
    HAL_Delay(5);
    SDA_High();
    HAL_Delay(5);
}

void SendByte(char n)
{
    for(int i=0;i<8;i++)
    {
        SCL_Low();
        HAL_Delay(5);
        if(n & 0x80)
        {
        SDA_High();
        HAL_Delay(5);
        }
        else
        {
        SDA_Low();
        HAL_Delay(5);
        }
        SCL_High();
        n = n<<1;
        HAL_Delay(5);
    }
}

char ReceiveAck()
{
    char ack;
    SCL_Low();
    HAL_Delay(5);
    SDA_High();
    HAL_Delay(5);
    SCL_High();
    HAL_Delay(5);
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
 