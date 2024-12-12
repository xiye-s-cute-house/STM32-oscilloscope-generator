/**
 ****************************************************************************************************
 * @file        adc.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.1
 * @date        2022-01-09
 * @brief       ADC ��������
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
 * V1.1 20220109
 * 1,֧��ADC��ͨ��DMA�ɼ� 
 * 2,����adc_dma_init��adc_dma_enable����
 *
 ****************************************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#include "sys.h"


/******************************************************************************************/
/* ADC������ ���� */

#define ADC_ADCX_CHY_GPIO_PORT              GPIOA
#define ADC_ADCX_CHY_GPIO_PIN               SYS_GPIO_PIN5 
#define ADC_ADCX_CHY_GPIO_CLK_ENABLE()      do{ RCC->AHB1ENR |= 1 << 0; }while(0)   /* PA��ʱ��ʹ�� */

#define ADC_ADCX                            ADC1 
#define ADC_ADCX_CHY                        5                                       /* ͨ��Y,  0 <= Y <= 19 */ 
#define ADC_ADCX_CHY_CLK_ENABLE()           do{ RCC->APB2ENR |= 1 << 8; }while(0)   /* ADC1 ʱ��ʹ�� */

/* ADC��ͨ��/��ͨ�� DMA�ɼ� DMA��������� ���� 
 * ע��: �������ǵ�ͨ������ʹ������Ķ���.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4
#define ADC_ADCX_DMASx_Channel              0                                       /* ͨ��0 */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

#define ADC_ADCX_DMASx_IS_TC()              ( DMA2->HISR & (1 << 5) )   /* �ж� DMA2_Stream4 ������ɱ�־, ����һ���ٺ�����ʽ,
                                                                         * ���ܵ�����ʹ��, ֻ������if��������� 
                                                                         */
#define ADC_ADCX_DMASx_CLR_TC()             do{ DMA2->HIFCR |= 1 << 5; }while(0)    /* ��� DMA2_Stream4 ������ɱ�־ */

/******************************************************************************************/


void adc_init(void);                    /* ADC��ʼ�� */
void adc_channel_set(ADC_TypeDef *adcx, uint8_t ch, uint8_t stime); /* ADCͨ������ */
uint32_t adc_get_result(uint8_t ch);/* ���ĳ��ͨ��ֵ  */
uint32_t adc_get_result_average(uint8_t ch, uint8_t times); /* �õ�ĳ��ͨ����������������ƽ��ֵ */
void  adc_dma_init(uint32_t mar);       /* ADC DMA�ɼ���ʼ�� */
void adc_dma_enable( uint16_t ndtr);    /* ʹ��һ��ADC DMA�ɼ����� */

#endif 















