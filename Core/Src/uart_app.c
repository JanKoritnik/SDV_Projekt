#include "uart_app.h"
#include "string.h"
#include "regulator.h"

extern CAN_HandleTypeDef  hcan1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;

extern uint8_t RS485_RxBuffer[RS485_BUFFER_SIZE];
extern uint8_t AllocBuffer[RS485_BUFFER_SIZE];

static CAN_TxHeaderTypeDef pTxHeader;
static CAN_RxHeaderTypeDef pRxHeader;
static uint32_t            pTxMailbox;
static uint8_t             received_data[8];

static char    RxUARTBuffer[256];
static uint8_t RxUARTLength = 0;
static uint8_t RxSingleByte;
static uint8_t onFlag = 0;

void UART_App_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart2, &RxSingleByte, 1);
    HAL_UART_Receive_DMA(&huart5, RS485_RxBuffer, 10);
}

void serialWrite(char data[])
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), 10);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n", 1, 10);
}

static void serialProcessRxData(void)
{
    switch (RxUARTBuffer[0])
    {
        case '1':
            onFlag = 1;
            HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, &onFlag, &pTxMailbox);
            break;
        case '0':
            onFlag = 0;
            HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, &onFlag, &pTxMailbox);
            break;
        case 'H':
            serialWrite("Ok");
            break;
        default:
            break;
    }
    RxUARTLength = 0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, received_data);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (RxSingleByte == '\n')
            serialProcessRxData();
        else
            RxUARTBuffer[RxUARTLength++] = RxSingleByte;

        HAL_UART_Receive_IT(&huart2, &RxSingleByte, 1);
    }

    if (huart->Instance == UART5)
    {
        memcpy(AllocBuffer, RS485_RxBuffer, RS485_BUFFER_SIZE);

        uint8_t  id  = AllocBuffer[0];
        //uint16_t  rpm_r = ((uint16_t)AllocBuffer[4] << 8) | ((uint16_t)AllocBuffer[5]);
        //int16_t rpm = (int16_t)rpm_r;
        int16_t rpm = (int16_t)(((uint16_t)AllocBuffer[5]&255 << 8) | ((uint16_t)AllocBuffer[4]&255));

        if      (id == 0x10) g_rpm_left  = rpm;
        else if (id == 0x30) g_rpm_right = rpm;

        HAL_UART_Receive_DMA(&huart5, RS485_RxBuffer, 10);
    }
}
