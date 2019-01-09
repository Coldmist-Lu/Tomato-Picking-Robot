#include "stm32f10x.h"
#include "serialport.h"

/** @addtogroup SERIALPORT
 * @{
 */

#define SERIALPORT_PACKETLENGTH        ((uint32_t)4)

uint8_t TxBuffer[SERIALPORT_PACKETLENGTH + 2] = {0};
uint8_t RxBuffer[SERIALPORT_PACKETLENGTH + 2] = {0};

/**
 * @brief Initialize serialport with dma.
 */
void SERIALPORT_Init()
{
    USART_InitTypeDef USART_InitStructrue;
    GPIO_InitTypeDef GPIO_InitStructrue;
    DMA_InitTypeDef  DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    
    GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructrue.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructrue);
    GPIO_InitStructrue.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructrue.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructrue);
    
    //rx dma
    DMA_InitStructure.DMA_BufferSize = SERIALPORT_PACKETLENGTH + 2;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RxBuffer;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1 -> DR));
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, ENABLE);
    
    //tx dma
    DMA_InitStructure.DMA_BufferSize = SERIALPORT_PACKETLENGTH;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    
    USART_InitStructrue.USART_BaudRate = 115200;
    USART_InitStructrue.USART_WordLength = USART_WordLength_8b;
    USART_InitStructrue.USART_StopBits = USART_StopBits_1;
    USART_InitStructrue.USART_Parity = USART_Parity_No;
    USART_InitStructrue.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructrue.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructrue);
    USART_OverSampling8Cmd(USART1, ENABLE);
    USART_Cmd(USART1, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_GetFlagStatus(USART1, USART_FLAG_TC);
}
#ifdef SERIALPORT_MASTER
/**
 * @brief Send command to slow the car.
 * @param speed Target speed when slowing down, value less than zero reverses direction of the car.
 * @note Work only in master controller.
 */
void SERIALPORT_SlowDown(int16_t speed)
{
    while((USART1->SR & USART_FLAG_TC) == RESET);
    USART1->SR &= ~USART_FLAG_TC;
    DMA_ClearFlag(DMA1_FLAG_TC4);
    TxBuffer[0] = 'S';
    TxBuffer[1] = 'L';
    TxBuffer[2] = (uint8_t)(speed >> 8);
    TxBuffer[3] = (uint8_t)speed;
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA1_Channel4->CNDTR = SERIALPORT_PACKETLENGTH;
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
}

/**
 * @brief Send command to restore speed.
 * @note Work only in master controller.
 */
void SERIALPORT_RestoreSpeed()
{
    while((USART1->SR & USART_FLAG_TC) == RESET);
    USART1->SR &= ~USART_FLAG_TC;
    TxBuffer[0] = 'R';
    TxBuffer[1] = 'S';
    TxBuffer[2] = 0;
    TxBuffer[3] = 0;
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA1_Channel4->CNDTR = SERIALPORT_PACKETLENGTH;
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
}
#endif

#ifdef SERIALPORT_SLAVE
/**
 * @brief Send command to turn on camera(s).
 * @note Work only in slave controller.
 */
void SERIALPORT_CameraStart()
{
    while((DMA1_Channel4->SR & USART_FLAG_TC) == RESET);
    USART1->SR &= ~USART_FLAG_TC;
    TxBuffer[0] = '<';
    TxBuffer[1] = 'C';
    TxBuffer[2] = 'A';
    TxBuffer[3] = 0;
    TxBuffer[4] = 0;
    TxBuffer[5] = '>';
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA1_Channel4->CNDTR = SERIALPORT_PACKETLENGTH;
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
}

/**
 * @brief Send command to turn off camera(s).
 * @note Work only in slave controller.
 */
void SERIALPORT_CameraStop()
{
    while((DMA1_Channel4->SR & USART_FLAG_TC) == RESET);
    USART1->SR &= ~USART_FLAG_TC;
    TxBuffer[0] = '<';
    TxBuffer[1] = 'C';
    TxBuffer[2] = 'S';
    TxBuffer[3] = 0;
    TxBuffer[4] = 0;
    TxBuffer[5] = '>';
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA1_Channel4->CNDTR = SERIALPORT_PACKETLENGTH;
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
#endif

/**
 * @brief Receieve and analyze command.
 * @param parameter Parameter of the receieved command.
 * @return Receieved command in @ref SERIALPORT_CommandTypedef.
 *         Master:
 *          -SERIALPORT_CommandCameraStart, no parameter.
 *          -SERIALPORT_CommandCameraStop, no parameter.
 *         Slave:
 *          -SERIALPORT_CommandSlowDown, speed as parameter.
 *          -SERIALPORT_CommandRestoreSpeed, no parameter.
 *         Master & Slave:
 *          -SERIALPORT_CommandUnknown, receieved command as parameter.
 *          -SERIALPORT_CommandNull, no parameter.
 */
SERIALPORT_CommandTypedef SERIALPORT_ReceiveCommand(int16_t *parameter)
{
    FlagStatus flag = DMA_GetFlagStatus(DMA1_FLAG_TC5);
    *parameter = 0;
    if(flag == SET)
    {
        DMA1_Channel5->CCR &= ~DMA_CCR5_EN;
        DMA_ClearFlag(DMA1_FLAG_TC5);
        DMA1_Channel5->CNDTR = SERIALPORT_PACKETLENGTH + 2;
        DMA1_Channel5->CCR |= DMA_CCR5_EN;
        if(RxBuffer[0] == '<' && RxBuffer[5] == '>')
        {
            if(RxBuffer[1] == 'C' && RxBuffer[2] == 'A')
            {
                return SERIALPORT_CommandCameraStart;
            }
            else if(RxBuffer[1] == 'C' && RxBuffer[2] == 'S')
            {
                return SERIALPORT_CommandCameraStop;
            }
        }
        *parameter = ((int16_t)RxBuffer[3] << 8) | RxBuffer[4];
        return SERIALPORT_CommandUnknown;
        
    }
    return SERIALPORT_CommandNull;
}

/**
 * @brief Restart dma.
 */
void SERIALPORT_RestartDma()
{
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR5_EN;
    
    DMA_ClearFlag(DMA1_FLAG_TC4);
    DMA_ClearFlag(DMA1_FLAG_TC5);

    DMA1_Channel4->CNDTR = SERIALPORT_PACKETLENGTH;
    DMA1_Channel5->CNDTR = SERIALPORT_PACKETLENGTH + 2;
    
    DMA1_Channel5->CCR |= DMA_CCR5_EN;
}
/**
 * @}
 */
