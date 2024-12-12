/**
 ****************************************************************************************************
 * @file        dma.h
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

#ifndef __DMA_H
#define	__DMA_H

#include "sys.h"


void dma_enable(DMA_Stream_TypeDef *dma_streamx, uint16_t ndtr);                                                /* ʹ��һ��DMA���� */
void dma_usart_tx_config(DMA_Stream_TypeDef *dma_streamx, uint8_t ch, uint32_t par, uint32_t mar);              /* ����1 TX DMA��ʼ�� */
void dma_basic_config(DMA_Stream_TypeDef *dma_streamx,uint32_t ch, uint32_t par, uint32_t m0ar, uint32_t m1ar); /* DMA�������� */

#endif






























