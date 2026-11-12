#include "bsp_rc.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);


}
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
void RC_restart(uint16_t dma_buf_num)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t wait_count = 10000U;

    __disable_irq();
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while ((hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) != 0U && wait_count > 0U)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
        wait_count--;
    }
    if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_EN) != 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,
                         DMA_FLAG_FEIF1_5 |
                         DMA_FLAG_DMEIF1_5 |
                         DMA_FLAG_TEIF1_5 |
                         DMA_FLAG_HTIF1_5 |
                         DMA_FLAG_TCIF1_5);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

    if (primask == 0U)
    {
        __enable_irq();
    }
}







