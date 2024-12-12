/*
author: CQU ��ΰ 
date:2024.11.24
content:weve create by stm32f407 week12 works
*/

//#include "tim_adc_dma_fft.h"
//#include "arm_math.h"   
 


#include "stm32f4xx.h"
#include <math.h>
#include "usart.h"
#include "delay.h"
#include "led.h"
#include "lcd.h"
#include "adc.h"
#include "dac.h"
#include "key.h"
//#include "dma.h"
#include "arm_math.h"   
#include "arm_const_structs.h"

// ADC�������߸���x��ʾ��Χ��30~250��y��ʾ��Χ��240~320
#define ADC_X_MIN 30
#define ADC_X_MAX 250
#define ADC_Y_MIN 240
#define ADC_Y_MAX 320
// DAC�������߸���x��ʾ��Χ��30~250��y��ʾ��Χ��360~440
#define DAC_X_MIN 30
#define DAC_X_MAX 250
#define DAC_Y_MIN 360
#define DAC_Y_MAX 440

#define SAMPLE_RATE 44100  // ���Ҳ�������
#define AMPLITUDE 40    // ���
#define FREQUENCY 1000     // Ƶ��

// FFt
/*ͨ��ADC�ɼ�ģ���źţ�Ȼ��ʹ��DMA���ɼ������ݴ��䵽�ڴ��У�����ͨ��FFT�㷨�����źŵ�Ƶ�ף����ͨ����������źŵĻ�����г����Ƶ�ʡ���ֵ����λ�����Ϣ*/
//#define sampledot  4096
//#define FFT_LENGTH		4096		//4096��FFT
//#define fft_arr 10     // ���ڼ���FFT����Ƶ�ʵ�ϵ��             
//#define fft_psc 84       // ���ڼ���FFT����Ƶ�ʵ�ϵ��            
//const u32  fft_sample_freq=84000000/(fft_arr*fft_psc);  // ����FFT����Ƶ��
//float fft_inputbuf[FFT_LENGTH*2];	 // FFT�������飬���ڴ�Ÿ���
//float fft_outputbuf[FFT_LENGTH];	 // FFT������飬��ŷ�ֵ
//arm_cfft_radix4_instance_f32 scfft;   // FFTʵ���ṹ��
//u32 sampledata[sampledot]={0};//���ڴ��ADC�������ݵ�����,��16λ����adc2 pa5�� ��16λ����adc1 pa6
//float phase_difference=0; // ���ڴ����λ��ı���
//float freamp[50];//���ڴ�Ÿ���г��Ƶ�ʺͷ�ֵ������
#define FFT_LENGTH        256         //FFT���ȣ�Ĭ����1024��FFT
float fft_inputbuf[FFT_LENGTH*2];    //FFT����������飬������Ϊarm_cfft_radix4_f32������������飬ǰһ��Ԫ��Ϊʵ������һ��Ϊ�鲿��ÿ����Ԫ�ش���һ����.
float fft_outputbuf[FFT_LENGTH];     //arm_cmplx_mag_f32() �����������
arm_cfft_radix4_instance_f32 scfft;
int fft_outputbuf_int[FFT_LENGTH];
int fft_outputbuf_point[FFT_LENGTH];

void sys_init(void);  // ����ϵͳ��ʼ��
void draw_home_page(void);  //���ڻ���������
void draw_ad_point(void); // ���ڻ滭ad�������ͼ
void draw_da_point(void); // ���ڻ滭da�������ͼ
void draw_ad_fft_point(void); // ���ڶ�y_adc[]�е�ֵ��fftƵ�׷���������ͼ
void y_adc_append(u16 adcx); //���Ա������������y_adc������õ�adc����ֵת��Ϊ���꣩�����ֵ
void y_dac_append(u16 dacval); // ͬ��
void lcdshow_adda_value(void); // ������ʾ��ǰ��dacval,ad da��ʵ�ʵ�ѹֵ����Ļ��

void Adc_Init_t(void);
void Dma_ADC_Init_t(void);
void TIM3_Init(u16 arr,u16 psc);
// usart
u8 t;
u8 len;  //�洢���ܵ�����Ϣ����	
u16 times=0;  // ���ڼ�ʱ��ÿ��5000

//ad da
u16 dacval=2048;  //dac���ֵ 0-4096
u16 adcx;  // adc����ֵ 0-4096
float temp; //��ʱ���ڴ洢����ֵ
int timer_flag;

//key
u8 key;

//draw
u16 t_adc[ADC_X_MAX - ADC_X_MIN] = {0};  // ���ڴ洢ADC���x����t������ 220
u16 y_adc[ADC_X_MAX - ADC_X_MIN] = {0};  // ���ڴ洢ADC���y���� 220��
u16 t_dac[DAC_X_MAX - DAC_X_MIN] = {0};  // ���ڴ洢DAC���x����t������ 220
u16 y_dac[DAC_X_MAX - DAC_X_MIN] = {0};  // ���ڴ洢DAC���y���� 220��
u16 y_adc_tmp[ADC_X_MAX - ADC_X_MIN] = {0};  // ���ڻ�����һ�ε����ݣ��滭����ʱ��
u16 y_dac_tmp[DAC_X_MAX - DAC_X_MIN] = {0};  // 
u16 y_adc_dma_tmp[ADC_X_MAX - ADC_X_MIN] = {0};  // ���ڴ洢dma���˵�adc��ֵ

u8 data_num = 220; // 250 - 30;  ����220������
int idx = 0; // ��������ֵ����ȡ��������Ԫ��
// ����ֵ�����ڻ�ͼʱ���㻭����ȷλ���ϣ�������һֱ����Ļ���Ͻ�
u16 x_adc_compensate  = 30; // ADC_X_MIN;  
u16 y_adc_compensate  = 320;  // ADC_Y_MAX
u16 x_dac_compensate  = 30; //DAC_X_MIN;
u16 y_dac_compensate  = 440;  // DAC_Y_MAX

//dma,adc1Ҫʹ��������0��ͨ��0
//#define SEND_BUF_SIZE 7044	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
//u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����
//const u8 TEXT_TO_SEND[]={"STM32F4 DMA"};	 
int test = 0;
int a = 0;

typedef enum
{
	damode,
	admode
}mode_flag;	

int main(void)
{ 	 
	int key2_num = 5;
	mode_flag modeflag = admode;
	u16 j,n,t,a;
	sys_init();
	for (j = 0; j < 220; j++) // ��t_adc��ֵ
	{
		t_adc[j] = j + 1;
	}
	for (n = 0; n < 220; n++) // ��t_dac��ֵ
	{
		t_dac[n] = n + 1;
	}
	
	for (t = 0; t < 220; t++)  //��y_adc��ֵ
	{	
		if (t < 80) y_adc[t] = t + 1;
		else {y_adc[t] = 80;}
	}
	
	for (a = 0; a < 220; a++)  // ��y_dac
	{	
//		if (a % 2 == 0) y_dac[a] = 80;  //����
//		else {y_dac[a] = 0;}
//		if (a < 80) {y_dac[a] = a;}  //���ǲ�
//		else if (a < 160 && a > 80){ y_dac[a] = a - 80;}
//		else if (a < 220 && a > 160) { y_dac[a] = a - 160;}
		double t = (double)a / SAMPLE_RATE;  //1khz���Ҳ�
        double angular_frequency = 2.0 * 3.14159265358979323846 * FREQUENCY;
        double sin_value = AMPLITUDE * sin(angular_frequency * t);
		
		if (sin_value < 0) { sin_value = AMPLITUDE - abs(sin_value); y_dac[a] = sin_value;}
        else {y_dac[a] = (u16)sin_value + AMPLITUDE;}  //���Ϸ�ֵ��������ʾ���������м�
		
	}

	
	DAC_SetChannel1Data(DAC_Align_12b_R,dacval);//��ʼֵ
//	MYDMA_Config(DMA2_Stream0,DMA_Channel_0,(u32)&ADC1->DR,(u32)y_adc,220);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:220
//	ADC_DMACmd(ADC1, ENABLE);
//	MYDMA_Enable(DMA2_Stream0, 220);
	draw_da_point();
	draw_home_page();

	while (1) 
	{
		// ���ڽ��ܵ���Ϣʱ
		if(USART_RX_STA&0x8000)  // ����״̬���λ
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			printf("\r\n\r\n");//���뻻��
			
			if (modeflag == admode) 
			{
				modeflag = damode;
					//
				LCD_Clear(WHITE);
				draw_home_page();
				lcdshow_adda_value();
				draw_ad_point();
				
				for (a = 0; a < 220; a++)  // ��y_dac
				{	
					if (a % 10 == 0) y_dac[a] = 80;  //����
					else {y_dac[a] = 0;}
			//		if (a < 80) {y_dac[a] = a;}  //���ǲ�
			//		else if (a < 160 && a > 80){ y_dac[a] = a - 80;}
			//		else if (a < 220 && a > 160) { y_dac[a] = a - 160;}
//					double t = (double)a / SAMPLE_RATE;  //1khz���Ҳ�
//					double angular_frequency = 2.0 * 3.14159265358979323846 * FREQUENCY;
//					double sin_value = AMPLITUDE * sin(angular_frequency * t);
					
//					if (sin_value < 0) { sin_value = AMPLITUDE - abs(sin_value); y_dac[a] = sin_value;}
//					else {y_dac[a] = (u16)sin_value + AMPLITUDE;}  //���Ϸ�ֵ��������ʾ���������м�
					
				}
				draw_da_point();
			}
			else if (modeflag == damode) 
			{
				modeflag = admode;
				LCD_Clear(WHITE);
				draw_home_page();
				lcdshow_adda_value();
				
				
				for (a = 0; a < 220; a++)  // ��y_dac
				{	
			//		if (a % 2 == 0) y_dac[a] = 80;  //����
			//		else {y_dac[a] = 0;}
			//		if (a < 80) {y_dac[a] = a;}  //���ǲ�
			//		else if (a < 160 && a > 80){ y_dac[a] = a - 80;}
			//		else if (a < 220 && a > 160) { y_dac[a] = a - 160;}
					double t = (double)a / SAMPLE_RATE;  //1khz���Ҳ�
					double angular_frequency = 2.0 * 3.14159265358979323846 * FREQUENCY;
					double sin_value = AMPLITUDE * sin(angular_frequency * t);
					
					if (sin_value < 0) { sin_value = AMPLITUDE - abs(sin_value); y_dac[a] = sin_value;}
					else {y_dac[a] = (u16)sin_value + AMPLITUDE;}  //���Ϸ�ֵ��������ʾ���������м�
					
				}
				draw_da_point();
			}
			USART_RX_STA=0;  // �ֶ����Զ�����ܱ��λ��Ϊ0
		}
		else  // ����û���յ�ʱ
		{
			if (modeflag == admode)
			{
				TIM_Cmd(TIM3, DISABLE);
	//			for (j = 0; j < 220; j++)
	//			{
	//				printf("dma adc value: %d\n",y_adc_dma_tmp[j]); 
	//			}
				times++;  //��ʱ��־+1
				idx = times % 220;  //��������
	//			printf("idx val:%d\r\n", idx);//test

	//			dacval = (u16)(y_dac[idx] / 80 * 4096);  //test
				dacval = (int)(4096 / 80) * y_dac[idx];
	//			printf("trans dacval:%d\r\n", dacval);//���뻻��

				DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//����DACֵ
		
				adcx=Get_Adc(ADC_Channel_5);
				
	//			printf("current adcx:%d\r\n\n", adcx);  //test
	//			printf("current dacval:%d\r\n", dacval);
				
				y_adc_append(adcx);  // ��Ӷ�ȡ����adcֵ
	//			if (times % 10000 == 0)  // ��Ļ����Ҫ̫��ˢ��������100�μ���ˢ��һ��
	//			{
				draw_ad_point();
	//			}
				
				key=KEY_Scan(0);			  // ����key_up��ͣ��ʾ
				if(key==WKUP_PRES)
				{	
	//				BACK_COLOR = BLACK;
					LCD_ShowString(70,190,200,16,16,"<cupture paused!>"); //��ʾ�Ѿ���ͣ
					lcdshow_adda_value();
	//				draw_ad_point();				
					delay_ms(300);
					while (1)				// �ٴΰ���key_up���Լ�����ʾ
					{
						key = KEY_Scan(0);
						if (key == WKUP_PRES || key == KEY1_PRES)  //������ְ�����ͣ���ٰ�key1����������������key = KEY1_PRES��ѭ����
						{
							delay_ms(300);
	//						BACK_COLOR = WHITE;
							LCD_ShowString(70,190,200,16,16,"                 "); //�����ʾ���Ѿ���ͣ	
							break;
						}
					}
				}
				if (key==KEY1_PRES)  //����key1����Ƶ�׷���
				{
					LED0=!LED0;	   //LED��˸.

					LCD_ShowString(70,210,200,16,16,"FFT-ADC-capture ");
					draw_ad_fft_point();
					delay_ms(300);
					while (1)				// �ٴΰ���key1���Լ�����ʾ
					{
						key = KEY_Scan(0);
						if (key == WKUP_PRES)
						{
							delay_ms(300);
	//						BACK_COLOR = WHITE;
							LCD_Clear(WHITE);
							LCD_ShowString(70,210,200,16,16,"ADC cupture axis");
								
							draw_home_page();
							lcdshow_adda_value();
							draw_da_point();
							
							break;
						}
					}
				}

				LED0=!LED0;	   //LED��˸.

			}

			else if (modeflag == damode)
			{
				TIM_Cmd(TIM3, ENABLE);
				
				key=KEY_Scan(0);			  // ����key_up��ͣ��ʾ
				if(key==KEY2_PRES)
				{
					switch (key2_num)
					{
						case 0:  //100hz
							key2_num += 1;
							TIM3_Init(104, 3999);
						break;
						
						case 1:  //300hz
							key2_num += 1;
							TIM3_Init(101, 1371);
						break;
																		
						case 2:  //500hz
							key2_num += 1;
							TIM3_Init(104, 799);
						break;
																								
						case 3:  //700hz
							key2_num += 1;
							TIM3_Init(101, 587);
						break;
																		
						case 4:  //900hz
							key2_num += 1;
							TIM3_Init(104, 443);
						break;
						
						case 5:  //1000hz
							key2_num = 0;
							TIM3_Init(104, 399);
						break;
												
						default:
						break;
					}
//	//				BACK_COLOR = BLACK;
//					LCD_ShowString(70,190,200,16,16,"<cupture paused!>"); //��ʾ�Ѿ���ͣ
//					lcdshow_adda_value();
//	//				draw_ad_point();				
//					delay_ms(300);
//					while (1)				// �ٴΰ���key_up���Լ�����ʾ
//					{
//						key = KEY_Scan(0);
//						if (key == WKUP_PRES || key == KEY1_PRES)  //������ְ�����ͣ���ٰ�key1����������������key = KEY1_PRES��ѭ����
//						{
//							delay_ms(300);
//	//						BACK_COLOR = WHITE;
//							LCD_ShowString(70,190,200,16,16,"                 "); //�����ʾ���Ѿ���ͣ	
//							break;
//						}
//					}
				}
			
			}
		}	    
//		delay_ms(10);	   
	}
}

void sys_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(115200);	//����1��ʼ��������Ϊ115200USB���� 
	LED_Init();
	LCD_Init();					//LCD��ʼ��
	Adc_Init(); 				//adc��ʼ��
	KEY_Init(); 				//������ʼ��
	Dac1_Init();				//DACͨ��1��ʼ��
	TIM3_Init(104, 399);//500us
//	Adc_Init_t();
//	Dma_ADC_Init_t();
//	my_adc_init();
//	my_dma_init(DMA2_Stream0,DMA_Channel_0,(u32)&ADC1->DR,(u32)y_adc,220);
}

void draw_home_page(void)
{	
//	LCD_Clear(WHITE);
	// ��ҳ��page_home
	POINT_COLOR=BLACK; 
	LCD_ShowString(60,20,200,16,16,"Chen wei E2021157 CQU");	
	LCD_ShowString(70,40,200,16,16,"Wave Create Anlysis");	
//	LCD_ShowString(30,90,200,16,16,"CQU");
//	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
//	LCD_ShowString(30,130,200,16,16,"WK_UP:+  KEY1:-");	 
	POINT_COLOR=BLUE;//��������Ϊ��ɫ      	 
	LCD_ShowString(30,100,200,16,16,"DAC VAL:");	      
	LCD_ShowString(30,120,200,16,16,"DAC VOL:0.000V");	      
	LCD_ShowString(30,140,200,16,16,"ADC VOL:0.000V");
//	// ��Ļ����ֻ��500*300������x < 300,y < 500
	POINT_COLOR = BLACK;
	
	// ADC������,����x��ʾ��Χ��30~250��y��ʾ��Χ��240~320
	LCD_Draw_Circle(60,220,5);  //����ԲȦ
	LCD_ShowString(70,210,200,16,16,"ADC cupture axis");
//	LCD_DrawRectangle(30, 240, 30, 320);  // ������
//	LCD_DrawRectangle(30, 320, 250, 320);  // ������
//	LCD_ShowString(28,238,200,16,16, "^");
	LCD_ShowString(246,312,200,16,16, ">");
	LCD_DrawLine(30, 240, 30, 320);							//������
	LCD_DrawLine(30, 320, 250, 320);							//������

	// DAC�����ᣬ����x��ʾ��Χ��30~250��y��ʾ��Χ��360~440
//	LCD_DrawRectangle(30, 360, 30, 440);  // ������
//	LCD_DrawRectangle(30, 440, 250, 440);  // ������
	LCD_ShowString(70,340,200,16,16, "DAC create axis");
//	LCD_ShowString(30,358,200,16,16, "^");
	LCD_ShowString(246,432,200,16,16, ">");
	LCD_DrawLine(30, 360, 30, 440);							//������30
	LCD_DrawLine(30, 440, 250, 440);							//������
}

// ���ڻ�ad�Ĳ�������
void draw_ad_point(void)
{
	u8 num = data_num;
	int i,j;
	// �Ȳ�����һ�λ��Ƶ�����,ͨ������һ�ε����ߣ���ɫΪ��ɫ�Ϳ��Ը��ǵ����Դﵽ������Ч��
	/*
	Ϊʲô��ʹ��LCD_Clear��ʹ�����������������
	*/
	for (j = 0; j < num; j++)
	{
		//��1 ��ԭ������Ϊ��y=0ʱ����x���غϣ�������������ܻ��
		LCD_Fast_DrawPoint(t_adc[j] + x_adc_compensate, y_adc_compensate - y_adc_tmp[j], WHITE);
	}
	
	// ����ÿһ���㣬�����ϲ�������Ļ����ֵ
	for (i = 0; i < num; i++)
	{
		//��1 ��ԭ������Ϊ��y=0ʱ����x���غϣ�������������ܻ��
		LCD_Fast_DrawPoint(t_adc[i] + x_adc_compensate, y_adc_compensate - y_adc[i], BLUE);
	}
	//draw_home_page();  //��������ں��棬��������Ը��ǵ�y=0�ǵ��غϲ��֣���Ȼ�ᱻ����
}

// ���ڻ�da���������
void draw_da_point(void)
{
	u8 num = data_num;
	u8 i,j;
	// �Ȳ�����һ�λ��Ƶ�����,ͨ������һ�ε����ߣ���ɫΪ��ɫ�Ϳ��Ը��ǵ����Դﵽ������Ч��
	for (j = 0; j < num; j++)
	{
		//��1 ��ԭ������Ϊ��y=0ʱ����x���غϣ�������������ܻ��
		LCD_Fast_DrawPoint(t_dac[j] + x_dac_compensate, y_dac_compensate - y_dac_tmp[j], WHITE);
	}
	
	for (i = 0; i < num; i++)
	{
		// ����ÿһ���㣬�����ϲ�������Ļ����ֵ
		LCD_Fast_DrawPoint(t_dac[i] + x_dac_compensate, y_dac_compensate - y_dac[i], RED);
	}
//	delay_ms(10);
	//draw_home_page();
}

// ������y_adc��������꣬���Բ���
void y_adc_append(u16 adcx)
{
	
	
	u8 num = data_num;
	u8 i,j;
	u16 y_adc_value;
//	test += 1;
	// �ڸ�ֵ֮ǰ���Ȱ���һ�ε����ݱ�����y_adc_tmp��
	for (j = 0; j < num; j++)
	{
		y_adc_tmp[j] = y_adc[j];
	}
	
	for (i = 1; i < num; i++)
	{
		y_adc[num - i] = y_adc[num - i - 1];
	}
	if ((times % 500) == 0) printf("append adcx:%d\r\n\n", (int)(80 * adcx / 4096));
	// ��һ�������ӼĴ�������ֵadcx��0-4096��ת������Ļ�������ϵĶ�Ӧֵ0-80���������y�����ع���80����
	y_adc_value = (int)(80 * adcx / 4096); //(float)80 * (adcx / 4096);
	
	y_adc[0] = y_adc_value; //���һ����Ҫ���µõ���ֵ�ŵ���һ��λ����
}

// ������y_dac��������꣬���Բ���
void y_dac_append(u16 dacval)
{
	u8 num = data_num;
	u8 i,j;
	u16 y_dac_value;

	// �ڸ�ֵ֮ǰ���Ȱ���һ�ε����ݱ�����y_dac_tmp��
	for (j = 0; j < num; j++)
	{
		y_dac_tmp[j] = y_dac[j];
	}
	
	for (i = 1; i < num; i++)
	{
		y_dac[num - i] = y_dac[num - i - 1];
	}
	if ((times % 500) == 0) printf("append dacval:%d\r\n\n", (int)(80 * dacval / 4096));
	// ��һ����������Ĵ�����ֵdacval��0-4096��ת������Ļ�������ϵĶ�Ӧֵ0-80���������y�����ع���80����
	y_dac_value = (int)(80 * dacval / 4096);
	
	y_dac[0] = y_dac_value; //���һ����Ҫ���µõ���ֵ�ŵ���һ��λ����
}



//����Բɼ����ź���fftƵ�׷�������ͼ
void draw_ad_fft_point(void)
{
	char str[10]; // Ƶ���ַ���
	int freq = 0;
	u8 num = data_num;
	int cnt = 0;
	int index_start = 0;
	int index_end = 0; //�������Ƶ��ʱ����
	int i,j;
	uint32_t maxIndex, minIndex;
	float32_t maxValue, minValue;
	float32_t scale; //��һ��ϵ��
	
	//ֻ����adc������
	LCD_Clear(WHITE);
	draw_home_page();
	lcdshow_adda_value();
	draw_da_point();
	
//	arm_cfft_radix4_instance_f32 scfft;
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//��ʼ��scfft�ṹ�壬�趨FFT��ز���
//	arm_cfft_radix4_init_f32(&scfft, 256, 0, 1);
	
	for(j=0;j < FFT_LENGTH;j++)  // �����ź�����
	{
		if (j <= 220)
		{
			fft_inputbuf[2*j]= (float)y_adc[j];//����ʵ��
		}
		else if (j > 220)
		{
			fft_inputbuf[2*j]= 0;//����ʵ��
		}
		else if (j == 255) {fft_inputbuf[2*j+1]=0;break;}
		fft_inputbuf[2*j+1]=0;//�鲿ȫ��Ϊ0
	}
	

//    for(i=0;i<FFT_LENGTH;i++)//�����ź�����
//    {
//		  fft_inputbuf[2*i]=10+4.5*arm_sin_f32(2*PI*i*200/FFT_LENGTH)+\
//							   7.5*arm_sin_f32(2*PI*i*350/FFT_LENGTH);
//		 
//		  fft_inputbuf[2*i+1]=0;//�鲿ȫ��Ϊ0
//	 }

//	arm_cfft_radix4_f32(&scfft,fft_inputbuf);    //FFT���㣨��4��
//	arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);    //��������������ģ�÷�ֵ 
	//arm_cfft_sR_f32_len1024���ñ�����Ϊ"arm_const_structs.h"�ṩ�����ñ���������ͷ�ļ���ֱ�ӵ��ü��ɡ�
//	arm_cfft_f32(&arm_cfft_sR_f32_len1024,fft_inputbuf,0,1);
//	arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);    //��������������ģ�÷�ֵ
//				 
	arm_cfft_radix4_f32(&scfft, fft_inputbuf);                      /* FFT���㣨��4�� */
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, 256);     /* ��������������ģ�÷�ֵ */
//	for (i = 0; i < FFT_LENGTH; i++)
//	{
//		printf("%f \n", fft_outputbuf[i]);
//	}

	//��һ������
//	for (i = 0; i < FFT_LENGTH; i++)  //ת����
//	{
//		fft_outputbuf_int[i] = (int)fft_inputbuf[i];
//	}
	arm_max_f32(fft_outputbuf, FFT_LENGTH, &maxValue, &maxIndex);
	arm_min_f32(fft_outputbuf, FFT_LENGTH, &minValue, &minIndex);
	// �����һ��ϵ��
	
    scale = (80 - 0) / (maxValue - minValue);
 // ��һ�����鲢���浽 fft_outputbuf_point
    for (i = 0; i < FFT_LENGTH; i++)
    {
        fft_outputbuf_point[i] = (int)((fft_outputbuf[i] - minValue) * scale + 0);
    }

    // ��ӡ��һ���������
//    for (i = 0; i < FFT_LENGTH; i++)
//    {
//        printf("%d\n", fft_outputbuf_point[i]);
//    }
	// ��ʼ����
	for (i = 0; i < num; i++)
	{
		//��1 ��ԭ������Ϊ��y=0ʱ����x���غϣ�������������ܻ��
		LCD_DrawLine(t_adc[i] + x_adc_compensate, y_adc_compensate - fft_outputbuf_point[i], t_adc[i] + x_adc_compensate, 320);
	}
	
	//����Ƶ��
	for (i = 0; i < 220; i++) printf("%d\n", y_adc[i]);
	for (i = 0; i < 220; i++)
	{
		if(y_adc[i] > 70 && index_start < 1 && cnt == 0)
		{
			cnt += 1;
			index_start = i;
//			if (cnt == 0) {cnt += 1; index_start = i;}  //�з���ļ��
		}
		if(y_adc[i] < 20  && index_end < 1 && cnt == 1)
		{
			cnt = 0;
			index_end = i;
//			if (cnt == 1) {cnt -= 1; index_end = i;}
			break;
		}
	}
	
	//index_start - index_end ���ǰ�����ڵĳ��ȣ���һ������147/220=0.66818181...��ʱ����Ƶ��
	freq = (int)(1000/(2*(index_end - index_start)*0.6681));
	printf("freq:%d index_s:%d end:%d\n", freq, index_start, index_end);
    index_start = 0;
	index_end = 0;

//	freq = (int)(freq/1000);
    sprintf(str, "%d", freq);

    // ���� str �д洢������ num ת������ַ���
    printf("Converted string: %s\n", str);
	LCD_ShowString(170,140,200,16,16,"frequence:    ");
	//��ʾ ����Ƶ��
	LCD_ShowString(270,140,200,16,16,str);

}

void lcdshow_adda_value(void)
{
	dacval=DAC_GetDataOutputValue(DAC_Channel_1);//���ڶ�ȡdac�����ֵ
//	dacval=DAC_GetDataOutputValue(DAC_Channel_1);//���ڶ�ȡdac�����ֵ
	LCD_ShowxNum(94,100,dacval,4,16,0);     	   //��ʾDAC�Ĵ���ֵ
	temp=(float)dacval*(3.3/4096);			         //ת���ɵ�ѹֵ
	dacval=temp;
	LCD_ShowxNum(94,120,dacval,1,16,0);     	   //��ʾ��ѹֵ��������
	temp-=dacval;
	temp*=1000;
	LCD_ShowxNum(110,120,temp,3,16,0X80); 	   //��ʾ��ѹֵ��С������
// 	adcx=Get_Adc_Average(ADC_Channel_5,10);		//�õ�ADCת��ֵ	  
	temp=(float)adcx*(3.3/4096);			        //�õ�ADC��ѹֵ
	adcx=temp;
	LCD_ShowxNum(94,140,temp,1,16,0);     	  //��ʾ��ѹֵ��������
	temp-=adcx;
	temp*=1000;
	LCD_ShowxNum(110,140,temp,3,16,0X80); 	  //��ʾ��ѹֵ��С������
}




void TIM3_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitStruc;
	
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV2;
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStrue.TIM_Period=arr;
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //ʹ�ܶ�ʱ��ʱ��
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	NVIC_InitStruc.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStruc.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruc.NVIC_IRQChannelPreemptionPriority=0X01;
	NVIC_InitStruc.NVIC_IRQChannelSubPriority=0X03;
	NVIC_Init(&NVIC_InitStruc);
	
	TIM_Cmd(TIM3,DISABLE);
}



//��ʱ���ж�
void TIM3_IRQHandler(void)
 {
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
//		a += 1;
//		if (test == 220) {printf("%d", a);while(1);} //294��
		if (timer_flag == 0)
		{
			timer_flag = 1;
			dacval = 4094;
		}
		else if (timer_flag == 1)
		{
			timer_flag = 0;
			dacval = 0;
		}
		//(int)(4096 / 80) * y_dac[idx];
//			printf("trans dacval:%d\r\n", dacval);//���뻻��

		DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//����DACֵ
		
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}

// ADC��ʼ������
//void Adc_Init_t()
//{
//	GPIO_InitTypeDef  GPIO_InitStructure;
//	ADC_CommonInitTypeDef ADC_CommonInitStructure;
//	ADC_InitTypeDef       ADC_InitStructure;
//	
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
//  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
//	
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5;  //adc 1��2 ��ͨ��
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	 GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE); //����
//	
//	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_InjecSimult;
//    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
//    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;
//    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; 
//    ADC_CommonInit(&ADC_CommonInitStructure);
//	
//	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	
//    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
//    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;  
//    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
//    ADC_InitStructure.ADC_NbrOfConversion =1;  //ͨ����
//	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T3_TRGO;
//    ADC_Init(ADC1, &ADC_InitStructure);
//    ADC_Init(ADC2, &ADC_InitStructure);
//    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_3Cycles);
//		
//   ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE); //��·ת����󴥷�dma
//   
//	ADC_DMACmd(ADC1, ENABLE); 
//	
//	ADC_Cmd(ADC1, ENABLE);
//    ADC_Cmd(ADC2, ENABLE);
//}
 
 
// DMA��ʼ������������ADC���ݵĲɼ�
//void Dma_ADC_Init_t()
//{
//	
//	
//	DMA_InitTypeDef  DMA_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//	
// 
//	DMA_DeInit(DMA2_Stream0);
//	
//	DMA_InitStructure.DMA_BufferSize= data_num;  //���ݳ���220
//	DMA_InitStructure.DMA_Channel=DMA_Channel_0; 
//	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;	
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
//    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;	
//	
//	DMA_InitStructure.DMA_Memory0BaseAddr= (uint32_t)&y_adc_dma_tmp ;//Ҫ�����ֵ
//	
//	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_MemoryDataSize= DMA_MemoryDataSize_Word;
//	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;		
//	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;
//	
//		
//	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)0x40012308; //adc��ַ
//	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Word;
//	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority=DMA_Priority_High;
//	
// 
// 
//  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
////  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);  //ʹ���ж�
//  DMA_Cmd(DMA2_Stream0, ENABLE);
//	 
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;  //DMA2_Stream0�ж�
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  //��ռ���ȼ�1
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;        //�����ȼ�1
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQͨ��ʹ��
//  NVIC_Init(&NVIC_InitStructure);
//}
 
//// ���ݳ�ʼ�����������ڳ�ʼ��ADC��DMA�����ںͶ�ʱ��
//void Data_Init()
//{
//	u32 idex;
//	float temp;	
//	Adc_Init();
//	Dma_ADC_Init();
//	uart_init(115200);
//	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//��ʼ��scfft�ṹ�壬�趨FFT��ز���     //FFT_LENGTH 4096
//	Tim3_Init(fft_arr-1,fft_psc-1);
//}
// 
// 
//// DMA�жϷ����������ڴ���ADC���ݲɼ���ɺ�Ĳ���
//void DMA2_Stream0_IRQHandler(void)  
//{
//	u32 idex;	//���ڽ��ɼ��������ݸ�ֵ��fft_inputbuf[2*idex]�ļ���	
//  float bias_voltage2,HZ2,amplitude2,phase2,bias_voltage1,HZ1,amplitude1,phase1;
// 
// 
//	u8 temp[40];
//	int i;
//	u16   freamplen; // freamp���ȵ�һ��
//	
//	
//	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))  //�ж�DMA��������ж�  
//    {
//		
//		TIM_Cmd(TIM3,DISABLE);//�ر�ʱ�ӣ����м���			
//		//adc2 pa5
//		for(idex=0;idex<sampledot;idex++) //��16λfft��adc2 fft1 //sampledot==4096
//		{			
//			fft_inputbuf[2*idex]=(u16)(sampledata[idex]>>16)*(3.3/4096);    //���������ź�ʵ��
//			fft_inputbuf[2*idex+1]=0;//�鲿ȫ��Ϊ0
//		}
//		arm_cfft_radix4_f32(&scfft,fft_inputbuf);  //fft����
//		arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//��������������ģ�÷�ֵ	
//		freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2);//Ѱ�һ�����г��	
//		
//		bias_voltage2=fft_outputbuf[0]/FFT_LENGTH;//ֱ�� 
//		HZ2=freamp[0];//Ƶ��
//		amplitude2=freamp[1];//����
//		phase2=freamp[2];//��λ
//		freamp[0]=0;freamp[1]=0;freamp[2]=0;
// 
//		//adc1 pa6
//		for(idex=0;idex<sampledot;idex++) //��16λfft ��adc1 fft2
//		{
//			 fft_inputbuf[2*idex]=(u16)(sampledata[idex])*(3.3/4096);    //���������ź�ʵ��
//			 fft_inputbuf[2*idex+1]=0;//�鲿ȫ��Ϊ0	��
//			
//		}	
//		arm_cfft_radix4_f32(&scfft,fft_inputbuf);  //fft����
//		arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//��������������ģ�÷�ֵ
//		freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2); //Ѱ�һ�����г��	
//		
//		bias_voltage1=fft_outputbuf[0]/FFT_LENGTH;//ƫ�õ�ѹ      
//		HZ1=freamp[0];//Ƶ��
//		amplitude1=freamp[1];//����
//		phase1=freamp[2];//��λ
//		freamp[0]=0;freamp[1]=0;freamp[2]=0;
//		
//		phase_difference=phase2-phase1;
//	  if(phase_difference>180) phase_difference=phase_difference-180;
//	  if(phase_difference<-180) phase_difference=phase_difference+180;
//		
//		printf("\r\n");    //fft����Ƶ��
//		printf("fft_sample_freq:%d\r\n",fft_sample_freq);    //fft����Ƶ�� 		
//		printf("bias_voltage1:%.2f\r\n",bias_voltage1); //ƫ�õ�ѹ 
//		printf("bias_voltage2:%.2f\r\n",bias_voltage2); //ƫ�õ�ѹ
// 
//		printf("HZ1:%.2f\r\n",HZ1);   //Ƶ��
//		printf("HZ2:%.2f\r\n",HZ2);//Ƶ��
//		
//		printf("amplitude1:%.2f\r\n",amplitude1); //��ֵ 
//		printf("amplitude2:%.2f\r\n",amplitude2);//��ֵ  
//		
//		printf("phase_difference:%.2f\r\n",phase_difference);//��λ��        
//		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
//		
//	}	
//}
// ��ȡFFT��ֵ
//int fft_getpeak(float *inputx,float *input,float *output,u16 inlen,u8 x,u8 N,float y) //  intlen �������鳤�ȣ�xѰ�ҳ���
//{                                                                           
//	int i,i2;
//	u32 idex;  //��ͬ����һ�������еģ���Ϊ�����ڲ�ͬ�ĺ����б�����
//	float datas;
//	float sum;
//	int outlen=0;
//	for(i=0;i<inlen-x;i+=x)
//	{
//		arm_max_f32(input+i,x,&datas,&idex);   
//		if( (input[i+idex]>=input[i+idex+1])&&(input[i+idex]>=input[i+idex-1])&&( (2*datas)/FFT_LENGTH )>y)   
//		   {
//			   sum=0;   
//			   for(i2=i+idex-N;i2<i+idex+N;i2++)   
//			   {
//				   sum+=input[i2];          
//			   }        
//			   if(1.5*sum/(2*N)<datas)       
//			   {                                                                                             
//				     output[3*outlen+2] = atan2(inputx[2*(i+idex+1)+1],inputx[2*(i+idex+1)])*180/3.1415926f;	//������λ		   
//				     output[3*outlen+1] = 1.0*(2*datas)/FFT_LENGTH;   //�������
//					   output[3*outlen] = 1.0*fft_sample_freq*(i+idex+1)/FFT_LENGTH;//����Ƶ��
//					   outlen++;				   
//			   }                                                                                               
//               else continue;			   
//		   }
//			
//		else continue;
//		
//	}
//	return outlen;
//	
//	
//}





/*dma
#define SEND_BUF_SIZE 8200	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.

u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����
const u8 TEXT_TO_SEND[]={"ALIENTEK Explorer STM32F4 DMA ����ʵ��"};	 

  
int main(void)
{ 
	u16 i;
	u8 t=0;
	u8 j,mask=0;
	float pro=0;//����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);     //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
 	LCD_Init();					//LCD��ʼ�� 
	KEY_Init(); 				//������ʼ�� 
 	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"DMA TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
	LCD_ShowString(30,130,200,16,16,"KEY0:Start");	 
	POINT_COLOR=BLUE;//��������Ϊ��ɫ      	 
 //��ʾ��ʾ��Ϣ	
	j=sizeof(TEXT_TO_SEND);	   
	for(i=0;i<SEND_BUF_SIZE;i++)//���ASCII�ַ�������
    {
		if(t>=j)//���뻻�з�
		{
			if(mask)
			{
				SendBuff[i]=0x0a;
				t=0;
			}else 
			{
				SendBuff[i]=0x0d;
				mask++;
			}	
		}else//����TEXT_TO_SEND���
		{
			mask=0;
			SendBuff[i]=TEXT_TO_SEND[t];
			t++;
		}   	   
    }		 
	POINT_COLOR=BLUE;//��������Ϊ��ɫ	  
	i=0;
	while(1)
	{
		t=KEY_Scan(0);
		if(t==KEY0_PRES)  //KEY0����
		{
			printf("\r\nDMA DATA:\r\n"); 	    
			LCD_ShowString(30,150,200,16,16,"Start Transimit....");
			LCD_ShowString(30,170,200,16,16,"   %") ;     //��ʾ�ٷֺ�       
      USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
			MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE);     //��ʼһ��DMA���䣡	  
		    //�ȴ�DMA������ɣ���ʱ������������һЩ�£����
		    //ʵ��Ӧ���У����������ڼ䣬����ִ�����������
		    while(1)
		    {
				if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
				{ 
					DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
					break; 
		        }
				pro=DMA_GetCurrDataCounter(DMA2_Stream7);//�õ���ǰ��ʣ����ٸ�����
				pro=1-pro/SEND_BUF_SIZE;//�õ��ٷֱ�	  
				pro*=100;      			    //����100��
				LCD_ShowNum(30,170,pro,3,16);	  
		    }			    
			LCD_ShowNum(30,170,100,3,16);//��ʾ100%	  
			LCD_ShowString(30,150,200,16,16,"Transimit Finished!");//��ʾ�������
		}
		i++;
		delay_ms(10);
		if(i==20)
		{
			LED0=!LED0;//��ʾϵͳ��������	
			i=0;
		}		   
	}		    
}

*/

/*
	u8 t;
	u8 len;	
	u16 times=0;  // ���ڼ�ʱ��ÿ��5000
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(115200);	//���ڳ�ʼ��������Ϊ115200 
	LED_Init();
	while(1)
	{
		if(USART_RX_STA&0x8000)  // ����״̬���λ
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			printf("\r\n�����͵���ϢΪ:\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);         //�򴮿�1��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			}
			printf("\r\n\r\n");//���뻻��
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\nALIENTEK ̽����STM32F407������ ����ʵ��\r\n");
				printf("����ԭ��@ALIENTEK\r\n\r\n\r\n");
			}
			if(times%200==0)printf("����������,�Իس�������\r\n");  
			if(times%30==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.
			delay_ms(10);   
		}
	}
	*/
	
	/*
	u16 adcx;
	float temp;
 	u8 t=0;	 
	u16 dacval=0;  //dac����ֵ
	u8 key;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);      //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	
	LED_Init();					//��ʼ��LED 
 	LCD_Init();					//LCD��ʼ��
	Adc_Init(); 				//adc��ʼ��
	KEY_Init(); 				//������ʼ��
	Dac1_Init();		 		//DACͨ��1��ʼ��	
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"DAC TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
	LCD_ShowString(30,130,200,16,16,"WK_UP:+  KEY1:-");	 
	POINT_COLOR=BLUE;//��������Ϊ��ɫ      	 
	LCD_ShowString(30,150,200,16,16,"DAC VAL:");	      
	LCD_ShowString(30,170,200,16,16,"DAC VOL:0.000V");	      
	LCD_ShowString(30,190,200,16,16,"ADC VOL:0.000V");
 	
  DAC_SetChannel1Data(DAC_Align_12b_R,dacval);//��ʼֵΪ0	
	while(1)
	{
		t++;
		key=KEY_Scan(0);			  
		if(key==WKUP_PRES)
		{		 
			if(dacval<4000)dacval+=200;
			DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//����DACֵ
		}else if(key==2)	
		{
			if(dacval>200)dacval-=200;
			else dacval=0;
			DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//����DACֵ
		}	 
		if(t==10||key==KEY1_PRES||key==WKUP_PRES) 	//WKUP/KEY1������,���߶�ʱʱ�䵽��
		{	  
 			adcx=DAC_GetDataOutputValue(DAC_Channel_1);//��ȡǰ������DAC��ֵ
			LCD_ShowxNum(94,150,adcx,4,16,0);     	   //��ʾDAC�Ĵ���ֵ
			temp=(float)adcx*(3.3/4096);			         //�õ�DAC��ѹֵ
			adcx=temp;
 			LCD_ShowxNum(94,170,temp,1,16,0);     	   //��ʾ��ѹֵ��������
 			temp-=adcx;
			temp*=1000;
			LCD_ShowxNum(110,170,temp,3,16,0X80); 	   //��ʾ��ѹֵ��С������
 			adcx=Get_Adc_Average(ADC_Channel_5,10);		//�õ�ADCת��ֵ	  
			temp=(float)adcx*(3.3/4096);			        //�õ�ADC��ѹֵ
			adcx=temp;
 			LCD_ShowxNum(94,190,temp,1,16,0);     	  //��ʾ��ѹֵ��������
 			temp-=adcx;
			temp*=1000;
			LCD_ShowxNum(110,190,temp,3,16,0X80); 	  //��ʾ��ѹֵ��С������
			LED0=!LED0;	   
			t=0;
		}	    
		delay_ms(10);	 
	}
		*/
//}
