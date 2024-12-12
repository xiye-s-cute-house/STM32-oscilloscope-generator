///**
// ****************************************************************************************************
// * @file        adc.c
// * @author      ����ԭ���Ŷ�(ALIENTEK)
// * @version     V1.1
// * @date        2022-01-09
// * @brief       ADC ��������
// * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
// ****************************************************************************************************
// * @attention
// *
// * ʵ��ƽ̨:����ԭ�� STM32F407������
// * ������Ƶ:www.yuanzige.com
// * ������̳:www.openedv.com
// * ��˾��ַ:www.alientek.com
// * �����ַ:openedv.taobao.com
// *
// * �޸�˵��
// * V1.0 20220109
// * ��һ�η���
// * V1.1 20220109
// * 1,֧��ADC��ͨ��DMA�ɼ� 
// * 2,����adc_dma_init��adc_dma_enable����
// *
// ****************************************************************************************************
// */

//#include "adc_reg.h"
//#include "dma.h"
//#include "delay.h"


//uint8_t g_adc_dma_sta = 0;  /* DMA����״̬��־, 0,δ���; 1, ����� */

///**
// * @brief       ADC��ʼ������
// *   @note      ������֧��ADC1/ADC2����ͨ��, ���ǲ�֧��ADC3
// *              ����ʹ��12λ����, ADC����ʱ��=21M, ת��ʱ��Ϊ: �������� + 12��ADC����
// *              ��������������: 480, ��ת��ʱ�� = 492 ��ADC���� = 21.87us
// * @param       ��
// * @retval      ��
// */
//void adc_init(void)
//{
//    ADC_ADCX_CHY_GPIO_CLK_ENABLE(); /* IO��ʱ��ʹ�� */
//    ADC_ADCX_CHY_CLK_ENABLE();      /* ADCʱ��ʹ�� */
//    
//    sys_gpio_set(ADC_ADCX_CHY_GPIO_PORT, ADC_ADCX_CHY_GPIO_PIN,
//                 SYS_GPIO_MODE_AIN, SYS_GPIO_OTYPE_PP, SYS_GPIO_SPEED_MID, SYS_GPIO_PUPD_PU);   /* AD�ɼ�����ģʽ����,ģ������ */

//    RCC->APB2RSTR |= 1 << 8;        /* ADC1 & ADC2 & ADC3 ��λ, ע��, ���︴λ������ADC!!! */
//    RCC->APB2RSTR &= ~(1 << 8);     /* ��λ���� */

//    /* ADCʱ������ APB2, ��PCLK2, Ƶ��Ϊ84Mhz, ADC���ʱ��һ�㲻Ҫ����36M
//     * ��84M PCLK2������, ����ʹ��4��Ƶ, �õ�PCLK2 / 4 = 21Mhz ��ADCʱ��
//     */
//    ADC->CCR &= ~(3 << 16);         /* ADCPRE[1:0] ADCʱ��Ԥ��Ƶ���� */
//    ADC->CCR |= 1 << 16;            /* ����ADCʱ��Ԥ��Ƶϵ��Ϊ 4, �� PCLK2 / 4 = 21Mhz */

//    ADC_ADCX->CR1 = 0;              /* CR1���� */
//    ADC_ADCX->CR2 = 0;              /* CR2���� */

//    ADC_ADCX->CR1 |= 0 << 8;        /* ��ɨ��ģʽ */
//    ADC_ADCX->CR1 |= 0 << 24;       /* 12λģʽ */

//    ADC_ADCX->CR2 |= 0 << 1;        /* ����ת��ģʽ */
//    ADC_ADCX->CR2 |= 0 << 11;       /* �Ҷ��� */
//    ADC_ADCX->CR2 |= 0 << 28;       /* ������� */
//    
//    ADC_ADCX->SQR1 &= ~(0XF << 20); /* L[3:0]���� */
//    ADC_ADCX->SQR1 |= 0 << 20;      /* 1��ת���ڹ��������� Ҳ����ֻת����������1 */

//    ADC_ADCX->CR2 |= 1 << 0;        /* ����ADת���� */
//}

///**
// * @brief       ����ADCͨ������ʱ��
// * @param       adcx : adc�ṹ��ָ��, ADC1 / ADC2
// * @param       ch   : ͨ����, 0~18
// * @param       stime: ����ʱ��  0~7, ��Ӧ��ϵΪ:
// *   @arg       000, 3��ADCʱ������          001, 15��ADCʱ������
// *   @arg       010, 28��ADCʱ������         011, 56��ADCʱ������
// *   @arg       100, 84��ADCʱ������         101, 112��ADCʱ������
// *   @arg       110, 144��ADCʱ������        111, 480��ADCʱ������ 
// * @retval      ��
// */
//void adc_channel_set(ADC_TypeDef *adcx, uint8_t ch, uint8_t stime)
//{
//    if (ch < 10)    /* ͨ��0~9,ʹ��SMPR2���� */
//    { 
//        adcx->SMPR2 &= ~(7 << (3 * ch));        /* ͨ��ch ����ʱ����� */
//        adcx->SMPR2 |= 7 << (3 * ch);           /* ͨ��ch ������������,����Խ�߾���Խ�� */
//    }
//    else            /* ͨ��10~19,ʹ��SMPR2���� */
//    { 
//        adcx->SMPR1 &= ~(7 << (3 * (ch - 10))); /* ͨ��ch ����ʱ����� */
//        adcx->SMPR1 |= 7 << (3 * (ch - 10));    /* ͨ��ch ������������,����Խ�߾���Խ�� */
//    } 
//}

///**
// * @brief       ���ADCת����Ľ�� 
// * @param       ch: ͨ����, 0~18
// * @retval      ��
// */
//uint32_t adc_get_result(uint8_t ch)
//{
//    adc_channel_set(ADC_ADCX, ch, 7);   /* ����ADCX��Ӧͨ������ʱ��Ϊ480��ʱ������ */

//    ADC_ADCX->SQR3 &= ~(0X1F << 5 * 0); /* ��������1ͨ������ */
//    ADC_ADCX->SQR3 |= ch << (5 * 0);    /* ��������1 ͨ�� = ch */
//    ADC_ADCX->CR2 |= 1 << 30;           /* ��������ת��ͨ�� */

//    while (!(ADC_ADCX->SR & 1 << 1));   /* �ȴ�ת������ */

//    return ADC_ADCX->DR;                /* ����adcֵ */
//}

///**
// * @brief       ��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ��
// * @param       ch      : ͨ����, 0~19
// * @param       times   : ��ȡ����
// * @retval      ͨ��ch��times��ת�����ƽ��ֵ
// */
//uint32_t adc_get_result_average(uint8_t ch, uint8_t times)
//{
//    uint32_t temp_val = 0;
//    uint8_t t;

//    for (t = 0; t < times; t++) /* ��ȡtimes������ */
//    {
//        temp_val += adc_get_result(ch);
//        delay_ms(5);
//    }

//    return temp_val / times;    /* ����ƽ��ֵ */
//}

///**
// * @brief       ADC DMA�ɼ��жϷ�����
// * @param       �� 
// * @retval      ��
// */
//void ADC_ADCX_DMASx_IRQHandler(void)
//{
//    if (ADC_ADCX_DMASx_IS_TC())     /* DMA�������? */
//    {
//        g_adc_dma_sta = 1;          /* ���DMA������� */
//        ADC_ADCX_DMASx_CLR_TC();    /* ���DMA ������ ��������жϱ�־ */
//    }
//}

///**
// * @brief       ADC DMA��ȡ ��ʼ������
// *   @note      ����������ʹ��adc_init��ADC���д󲿷�����,�в���ĵط��ٵ�������
// * @param       mar         : �洢����ַ 
// * @retval      ��
// */
//void adc_dma_init(uint32_t mar)
//{
//    adc_init(); /* �ȳ�ʼ��ADC */

//    adc_channel_set(ADC_ADCX, ADC_ADCX_CHY, 7); /* ����ADCX��Ӧͨ������ʱ��Ϊ480��ʱ������ */
//    
//    /* ����ת������ */
//    /* ����ADC����ת��, DMA����ADC���� */
//    ADC_ADCX->CR2 |= 1 << 8;                    /* DMA = 1, DMA����ʹ�� */
//    ADC_ADCX->CR2 |= 1 << 9;                    /* DDS = 1, ֻҪ����ת��������DMA */
//    ADC_ADCX->CR2 |= 1 << 1;                    /* CONT = 1, ����ת��ģʽ */

//    ADC_ADCX->SQR3 &= ~(0X1F << 5 * 0);         /* ��������1ͨ������ */
//    ADC_ADCX->SQR3 |= ADC_ADCX_CHY << (5 * 0);  /* ��������1 ͨ�� = ADC_ADCX_CHY */

//    /* DMA��������, ������/ͨ��/�����ַ/�洢����ַ�� */
//    dma_basic_config(ADC_ADCX_DMASx, ADC_ADCX_DMASx_Channel, (uint32_t)&ADC_ADCX->DR, mar, 0);

//    /* ����DMA���� */
//    ADC_ADCX_DMASx->CR |= 0 << 6;   /* ���赽�洢��ģʽ */
//    ADC_ADCX_DMASx->CR |= 0 << 8;   /* ��ѭ��ģʽ(��ʹ����ͨģʽ) */
//    ADC_ADCX_DMASx->CR |= 0 << 9;   /* ���������ģʽ */
//    ADC_ADCX_DMASx->CR |= 1 << 10;  /* �洢������ģʽ */
//    ADC_ADCX_DMASx->CR |= 1 << 11;  /* �������ݳ���:16λ */
//    ADC_ADCX_DMASx->CR |= 1 << 13;  /* �洢�����ݳ���:16λ */
//    ADC_ADCX_DMASx->CR |= 1 << 16;  /* �е����ȼ� */
//    ADC_ADCX_DMASx->CR |= 0 << 21;  /* ����ͻ�����δ��� */
//    ADC_ADCX_DMASx->CR |= 0 << 23;  /* �洢��ͻ�����δ��� */
//    
//    ADC_ADCX_DMASx->CR |= 1 << 4;   /* TCIE = 1, DMA��������ж�ʹ�� */

//    sys_nvic_init(3, 3, ADC_ADCX_DMASx_IRQn, 2);    /* ��2��������ȼ� */
//}

///**
// * @brief       ʹ��һ��ADC DMA���� 
// * @param       cndtr: DMA����Ĵ���
// * @retval      ��
// */
//void adc_dma_enable(uint16_t ndtr)
//{
//    ADC_ADCX->CR2 &= ~(1 << 0);         /* �ȹر�ADC */
//    
//    dma_enable(ADC_ADCX_DMASx, ndtr);   /* ����ʹ��DMA���� */
//    
//    ADC_ADCX->CR2 |= 1 << 0;            /* ��������ADC */
//    ADC_ADCX->CR2 |= 1 << 30;           /* ��������ת��ͨ�� */
//}
































