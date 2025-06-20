/**
*@brief HotRC�ֱ��Ľ����ļ� SBUS.c
*@Usage main������дһ�� SB_USART_Start(&huart2, &hdma_usart2_rx); ����
                �ò�ͬ�Ĵ�����Ҫ�޸������ط�
                    1.main�����еĵ���
                    2.���ļ���43��
*@Channel ͨ��˵�����ܹ�8��ͨ��
                    0��1ͨ����Ӧ��ҡ��
                                                        0��x���ᣩ���򣬴�����1800 -> 200
                                                        1��y���������򣬴��µ���1800 -> 200
                    2��3ͨ����Ӧ��ҡ��
                                                        2��y���������򣬴��µ���1800 -> 200
                                                        3��x���ᣩ���򣬴�����1800 -> 200
                    4��7ͨ����ӦSWA5��SWD8������״̬���أ����µ���ֵ�ֱ�Ϊ��1792��992��192
                    5��6ͨ����ӦSWB6��SWC7����˫״̬���أ����µ���ֵ�ֱ�Ϊ��1792��     192
                    �м��ĸ���ť���ڵ�������ҡ�˵ĳ�ʼλ�ö�Ӧ�ĳ�ʼֵ
                    �������ĸ����ؿ��Կ���ͨ��0-3������������һ���ǻ�ؿ��أ�Ĭ������/���м�
                * ��־λfailsafe_status��0��ʾ����������1��ʾ���ӶϿ�
*/

#include "SBUS.h"
#include "string.h"

#define SBUS_START 0x0F // SBUS���ݰ�ͷ
#define SBUS_END 0x00   // SBUS���ݰ�β

uint8_t RxTemp_2[25] = {0};

// Initialize USART  ��ʼ������
void SB_USART_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)&RxTemp_2, 25);
    __HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
}

// ���ݻ���ͱ�־λ
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;
int16_t g_sbus_channels[8] = {0};

// �ֱ����ݴ���
void SBUS_Rcv_Prcs(uint8_t *data)
{
    if (data[0] == SBUS_START && data[24] == SBUS_END) // ���ݰ�ͷβ��ȷ��ʼ����
    {
        g_sbus_channels[0] = ((RxTemp_2[1] | RxTemp_2[2] << 8) & 0x07FF);
        g_sbus_channels[1] = ((RxTemp_2[2] >> 3 | RxTemp_2[3] << 5) & 0x07FF);
        g_sbus_channels[2] = ((RxTemp_2[3] >> 6 | RxTemp_2[4] << 2 | RxTemp_2[5] << 10) & 0x07FF);
        g_sbus_channels[3] = ((RxTemp_2[5] >> 1 | RxTemp_2[6] << 7) & 0x07FF);
        g_sbus_channels[4] = ((RxTemp_2[6] >> 4 | RxTemp_2[7] << 4) & 0x07FF);
        g_sbus_channels[5] = ((RxTemp_2[7] >> 7 | RxTemp_2[8] << 1 | RxTemp_2[9] << 9) & 0x07FF);
        g_sbus_channels[6] = ((RxTemp_2[9] >> 2 | RxTemp_2[10] << 6) & 0x07FF);
        g_sbus_channels[7] = ((RxTemp_2[10] >> 5 | RxTemp_2[11] << 3) & 0x07FF);

        // ��ȫ��⣬����Ƿ�ʧ���������ݴ���
        // Security detection to check for lost connections or data errors
        failsafe_status = SBUS_SIGNAL_OK;
        if (RxTemp_2[23] & (1 << 2))
        {
            failsafe_status = SBUS_SIGNAL_LOST;

            // lost contact errors  ң����ʧ������
        }
        else if (RxTemp_2[23] & (1 << 3))
        {
            failsafe_status = SBUS_SIGNAL_FAILSAFE;

            // data loss error  ���ݶ�ʧ����
        }
    }
}
