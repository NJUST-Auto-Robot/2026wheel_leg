#include "CRSF.h"
#include "crc8.h"
#include "zf_common_headfile.h"

volatile int head = 0, tail = 0;
uint8_t Crsf_Rx_buffer[CRSF_MAX_PACKET_SIZE];

#define SERIAL_COUNT() ((head - tail + CRSF_MAX_PACKET_SIZE) % CRSF_MAX_PACKET_SIZE)
#define SERIAL_BUFF_PUSH(x) Crsf_Rx_buffer[head = ((head == CRSF_MAX_PACKET_SIZE - 1)?0:(head+1))] = x
#define SERIAL_BUFF_PULL() Crsf_Rx_buffer[tail = ((tail == CRSF_MAX_PACKET_SIZE - 1)?0:(tail+1))]


CRSF_CH_Struct CRSF_CH;

// >用于接收处理小白控数据帧的回调
int frame_ready = 0;
void CRSF_callback(void)
{
    static int i = 0;
    static int receive_count = 0;
    uint8_t receive_data; // 定义临时变量
    
    if (uart_query_byte(UART_5, &receive_data)) // 接收串口数据
    {
       //if(receive_data == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
    //    if(receive_data == 0xC8)
    //    {
    //        Crsf_Rx_buffer[0] = receive_data;
    //        receive_count = 1;
    //        frame_ready = 1;
    //    }

    //    else if (1 == frame_ready && receive_count < 26)
    //    {
    //        Crsf_Rx_buffer[receive_count++] = receive_data;
    //    }

    //    // 处理一帧的数据
    //    if (1 == frame_ready && receive_count == 26)
    //    {
    //        Crsf_Data_Read(&Crsf_Rx_buffer[0],Crsf_Rx_buffer[1]);

    //        frame_ready = 0;
    //        receive_count = 0;
    //    }
        SERIAL_BUFF_PUSH(receive_data);
    }
    Crsf_Data_procees();
}

// 单字节+循环缓冲区接收处理数据
void Crsf_Data_procees(void)
{
    static int i = 0;
    static int length = 0;
    CRSF_CH.ConnectState = SBUS_SIGNAL_FAILSAFE;
    //lenght = SERIAL_COUNT();
    while(SERIAL_COUNT() >= 26)
    {
        if(SERIAL_BUFF_PULL() == 0xC8)
        {   
            if(SERIAL_BUFF_PULL() == 0x18)
            {
                uint8_t temp_buffer[26] = {0};
                temp_buffer[0]= 0xC8;
                temp_buffer[1]= 0x18;
                for(int i=2; i<26; i++)
                {
                    temp_buffer[i] = SERIAL_BUFF_PULL();
                
                }
            Crsf_Data_Read(temp_buffer,temp_buffer[1]);
            }
        }   
    }
}


void Crsf_Data_Read(uint8_t *data, uint8_t len)
{
    uint8_t inCrc = data[2 + len - 1];
    // CRC计算
    uint8_t crc = Crc8_calc(&data[2], len - 1);


    if( (data[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)&&(inCrc == crc) )
    {
        CRSF_CH.CH1  = ((int16_t)data[ 3] >> 0 | ((int16_t)data[ 4] << 8 )) & 0x07FF;
        CRSF_CH.CH2  = ((int16_t)data[ 4] >> 3 | ((int16_t)data[ 5] << 5 )) & 0x07FF;
        CRSF_CH.CH3  = ((int16_t)data[ 5] >> 6 | ((int16_t)data[ 6] << 2 ) | (int16_t)data[ 7] << 10 ) & 0x07FF;
        CRSF_CH.CH4  = ((int16_t)data[ 7] >> 1 | ((int16_t)data[ 8] << 7 )) & 0x07FF;
        CRSF_CH.CH5  = ((int16_t)data[ 8] >> 4 | ((int16_t)data[ 9] << 4 )) & 0x07FF;
        CRSF_CH.CH6  = ((int16_t)data[ 9] >> 7 | ((int16_t)data[10] << 1 ) | (int16_t)data[11] << 9 ) & 0x07FF;
        CRSF_CH.CH7  = ((int16_t)data[11] >> 2 | ((int16_t)data[12] << 6 )) & 0x07FF;
        CRSF_CH.CH8  = ((int16_t)data[12] >> 5 | ((int16_t)data[13] << 3 )) & 0x07FF;
        CRSF_CH.CH9  = ((int16_t)data[14] << 0 | ((int16_t)data[15] << 8 )) & 0x07FF;
        CRSF_CH.CH10 = ((int16_t)data[15] >> 3 | ((int16_t)data[16] << 5 )) & 0x07FF;
        CRSF_CH.CH11 = ((int16_t)data[16] >> 6 | ((int16_t)data[17] << 2 ) | (int16_t)data[18] << 10 ) & 0x07FF;
        CRSF_CH.CH12 = ((int16_t)data[18] >> 1 | ((int16_t)data[19] << 7 )) & 0x07FF;
        CRSF_CH.CH13 = ((int16_t)data[19] >> 4 | ((int16_t)data[20] << 4 )) & 0x07FF;
        CRSF_CH.CH14 = ((int16_t)data[20] >> 7 | ((int16_t)data[21] << 1 ) | (int16_t)data[22] << 9 ) & 0x07FF;
        CRSF_CH.CH15 = ((int16_t)data[22] >> 2 | ((int16_t)data[23] << 6 )) & 0x07FF;
        CRSF_CH.CH16 = ((int16_t)data[23] >> 5 | ((int16_t)data[24] << 3 )) & 0x07FF;

        CRSF_CH.ConnectState = SBUS_SIGNAL_OK;
    }

}

