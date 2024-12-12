/**
 ****************************************************************************************************
 * @file        dma.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2022-01-09
 * @brief       DMA ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20220109
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "dma.h"
#include "delay.h"


/**
 * @brief       ����TX DMA��ʼ������
 *   @note      ����Ĵ�����ʽ�ǹ̶���, ���Ҫ���ݲ�ͬ��������޸�
 *              �Ӵ洢�� -> ����ģʽ/8λ���ݿ��/�洢������ģʽ
 *
 * @param       dma_streamx : DMA������,DMA1_Stream0~7/DMA2_Stream0~7
 * @param       ch         :  DMAͨ��ѡ��,��Χ:1~115(���<<STM32H7xx�ο��ֲ�>>16.3.2��,Table 116)
 * @param       par         : �����ַ
 * @param       mar         : �洢����ַ
 * @retval      ��
 */
void dma_usart_tx_config(DMA_Stream_TypeDef *dma_streamx, uint8_t ch, uint32_t par, uint32_t mar)
{
    /* DMA�������� ����������/ͨ��/�����ַ/�洢����ַ�� */
    dma_basic_config(dma_streamx, ch, par, mar, 0);

    /* DMA�ض������������� */
    dma_streamx->CR |= 1 << 6;      /* �洢��������ģʽ */
    dma_streamx->CR |= 0 << 8;      /* ��ѭ��ģʽ(��ʹ����ͨģʽ) */
    dma_streamx->CR |= 0 << 9;      /* ���������ģʽ */
    dma_streamx->CR |= 1 << 10;     /* �洢������ģʽ */
    dma_streamx->CR |= 0 << 11;     /* �������ݳ���:8λ */
    dma_streamx->CR |= 0 << 13;     /* �洢�����ݳ���:8λ */
    dma_streamx->CR |= 1 << 16;     /* �е����ȼ� */
    dma_streamx->CR |= 0 << 21;     /* ����ͻ�����δ��� */
    dma_streamx->CR |= 0 << 23;     /* �洢��ͻ�����δ��� */
    
    //dma_streamx->FCR = 0X21;      /* FIFO���ƼĴ��� */
}

/**
 * @brief       DMA��������
 *   @note      �������DMA���һЩ�����Ե�����, ����: DMAʱ��ʹ�� / ���������ַ �� �洢����ַ
 *              �������ò���(CCR�Ĵ���), ���û��Լ�����ʵ��
 *
 * @param       dma_streamx : DMA��������, DMA1_Stream0 ~ DMA1_Stream7, DMA2_Stream0 ~ DMA1_Stream7
 * @param       ch          : ͨ��x, 0~7
 *                            ����ĳ�������Ӧ�ĸ�DMA, �ĸ�������, �ĸ�ͨ��, ��ο�<<STM32F4xx���Ĳο��ֲ�>> 9.3.3��
 *                            ����������ȷ��DMA��ͨ��, ��������ʹ��! 
 * @param       par         : �����ַ
 * @param       m0ar        : �洢��0��ַ
 * @param       m1ar        : �洢��1��ַ, ʹ��˫�����ʱ��Ż��õ�
 * @retval      ��
 */
void dma_basic_config(DMA_Stream_TypeDef *dma_streamx,uint32_t ch, uint32_t par, uint32_t m0ar, uint32_t m1ar)
{
    if (dma_streamx > DMA1_Stream7)     /* ���� DMA1_Stream7, ��ΪDMA2��ͨ���� */
    {
        RCC->AHB1ENR |= 1 << 22;        /* ����DMA2ʱ�� */
    }
    else
    {
        RCC->AHB1ENR |= 1 << 21;        /* ����DMA1ʱ�� */
    }

    delay_ms(5);                        /* �ȴ�DMAʱ���ȶ� */

    dma_streamx->CR = (ch & 7) << 25;   /* ������ͨ��ѡ��, 0 ~ 7 */
    dma_streamx->PAR = par;             /* DMA �����ַ */
    dma_streamx->M0AR = m0ar;           /* DMA �洢��0��ַ */
    dma_streamx->M1AR = m1ar;           /* DMA �洢��1��ַ */
    dma_streamx->NDTR = 0;              /* DMA ���䳤������, ������dma_enable�������� */
}

/**
 * @brief       ����һ��DMA����
 * @param       dma_streamx : DMA������,DMA1_Stream0~7/DMA2_Stream0~7
 * @param       ndtr        : ���ݴ�����
 * @retval      ��
 */
void dma_enable(DMA_Stream_TypeDef *dma_streamx, uint16_t ndtr)
{
    dma_streamx->CR &= ~(1 << 0);   /* �ر�DMA���� */

    while (dma_streamx->CR & 0X1);  /* ȷ��DMA���Ա����� */

    dma_streamx->NDTR = ndtr;       /* Ҫ�������������Ŀ */
    dma_streamx->CR |= 1 << 0;      /* ����DMA���� */
}



























