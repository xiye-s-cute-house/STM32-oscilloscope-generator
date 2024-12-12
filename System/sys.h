#ifndef __SYS_H
#define __SYS_H	 
#include "stm32f4xx.h" 

//�޸�˵��
//��
////////////////////////////////////////////////////////////////////////////////// 


//0,��֧��ucos
//1,֧��ucos
#define SYSTEM_SUPPORT_OS		0		//����ϵͳ�ļ����Ƿ�֧��UCOS
																	    
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).M4ͬM3����,ֻ�ǼĴ�����ַ����.
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014     

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //��� 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //����

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  //��� 
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  //����

//����Ϊ��ຯ��
void WFI_SET(void);		//ִ��WFIָ��
void INTX_DISABLE(void);//�ر������ж�
void INTX_ENABLE(void);	//���������ж�
void MSR_MSP(u32 addr);	//���ö�ջ��ַ 



///* sys_nvic_ex_configר�ú궨�� */
//#define SYS_GPIO_FTIR               1       /* �½��ش��� */
//#define SYS_GPIO_RTIR               2       /* �����ش��� */
//#define SYS_GPIO_BTIR               3       /* ������ش��� */

///* GPIO����ר�ú궨�� */
//#define SYS_GPIO_MODE_IN            0       /* ��ͨ����ģʽ */
//#define SYS_GPIO_MODE_OUT           1       /* ��ͨ���ģʽ */
//#define SYS_GPIO_MODE_AF            2       /* AF����ģʽ */
//#define SYS_GPIO_MODE_AIN           3       /* ģ������ģʽ */

//#define SYS_GPIO_SPEED_LOW          0       /* GPIO�ٶ�(����,2M) */
//#define SYS_GPIO_SPEED_MID          1       /* GPIO�ٶ�(����,25M) */
//#define SYS_GPIO_SPEED_FAST         2       /* GPIO�ٶ�(����,50M) */
//#define SYS_GPIO_SPEED_HIGH         3       /* GPIO�ٶ�(����,100M) */

//#define SYS_GPIO_PUPD_NONE          0       /* ���������� */
//#define SYS_GPIO_PUPD_PU            1       /* ���� */
//#define SYS_GPIO_PUPD_PD            2       /* ���� */
//#define SYS_GPIO_PUPD_RES           3       /* ���� */

//#define SYS_GPIO_OTYPE_PP           0       /* ������� */
//#define SYS_GPIO_OTYPE_OD           1       /* ��©��� */

///* GPIO����λ�ú궨��  */
//#define SYS_GPIO_PIN0               1<<0
//#define SYS_GPIO_PIN1               1<<1
//#define SYS_GPIO_PIN2               1<<2
//#define SYS_GPIO_PIN3               1<<3
//#define SYS_GPIO_PIN4               1<<4
//#define SYS_GPIO_PIN5               1<<5
//#define SYS_GPIO_PIN6               1<<6
//#define SYS_GPIO_PIN7               1<<7
//#define SYS_GPIO_PIN8               1<<8
//#define SYS_GPIO_PIN9               1<<9
//#define SYS_GPIO_PIN10              1<<10
//#define SYS_GPIO_PIN11              1<<11
//#define SYS_GPIO_PIN12              1<<12
//#define SYS_GPIO_PIN13              1<<13
//#define SYS_GPIO_PIN14              1<<14
//#define SYS_GPIO_PIN15              1<<15


///*��������*******************************************************************************************/
///* ��̬����(����sys.c�����õ�) */
//static void sys_nvic_priority_group_config(uint8_t group);                      /* ����NVIC���� */


///* ��ͨ���� */
//void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);             /* �����ж�ƫ���� */
//void sys_nvic_init(uint8_t pprio, uint8_t sprio, uint8_t ch, uint8_t group);    /* ����NVIC */
//void sys_nvic_ex_config(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint8_t tmode);   /* �ⲿ�ж����ú���,ֻ���GPIOA~GPIOK */
//void sys_gpio_af_set(GPIO_TypeDef *gpiox, uint16_t pinx, uint8_t afx);          /* GPIO���ù���ѡ������  */
//void sys_gpio_set(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint32_t mode, 
//                  uint32_t otype, uint32_t ospeed, uint32_t pupd);              /*  GPIOͨ������ */

//void sys_gpio_pin_set(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint8_t status);    /* ����GPIOĳ�����ŵ����״̬ */
//uint8_t sys_gpio_pin_get(GPIO_TypeDef *p_gpiox, uint16_t pinx);                 /* ��ȡGPIOĳ�����ŵ�״̬ */
//void sys_standby(void);         /* �������ģʽ */
//void sys_soft_reset(void);      /* ϵͳ��λ */

//uint8_t sys_clock_set(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);      /* ʱ�����ú��� */
//void sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);  /* ϵͳʱ�ӳ�ʼ������ */
//void sys_qspi_enable_memmapmode(uint8_t ftype); /* QSPI�����ڴ�ӳ��ģʽ */

///* ����Ϊ��ຯ�� */
//void sys_wfi_set(void);             /* ִ��WFIָ�� */
//void sys_intx_disable(void);        /* �ر������ж� */
//void sys_intx_enable(void);         /* ���������ж� */
//void sys_msr_msp(uint32_t addr);    /* ����ջ����ַ */
#endif











