/*
author: CQU 陈伟 
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

// ADC坐标曲线该轴x显示范围：30~250，y显示范围：240~320
#define ADC_X_MIN 30
#define ADC_X_MAX 250
#define ADC_Y_MIN 240
#define ADC_Y_MAX 320
// DAC坐标曲线该轴x显示范围：30~250，y显示范围：360~440
#define DAC_X_MIN 30
#define DAC_X_MAX 250
#define DAC_Y_MIN 360
#define DAC_Y_MAX 440

#define SAMPLE_RATE 44100  // 正弦波采样率
#define AMPLITUDE 40    // 振幅
#define FREQUENCY 1000     // 频率

// FFt
/*通过ADC采集模拟信号，然后使用DMA将采集的数据传输到内存中，接着通过FFT算法分析信号的频谱，最后通过串口输出信号的基波和谐波的频率、幅值和相位差等信息*/
//#define sampledot  4096
//#define FFT_LENGTH		4096		//4096点FFT
//#define fft_arr 10     // 用于计算FFT采样频率的系数             
//#define fft_psc 84       // 用于计算FFT采样频率的系数            
//const u32  fft_sample_freq=84000000/(fft_arr*fft_psc);  // 计算FFT采样频率
//float fft_inputbuf[FFT_LENGTH*2];	 // FFT输入数组，用于存放复数
//float fft_outputbuf[FFT_LENGTH];	 // FFT输出数组，存放幅值
//arm_cfft_radix4_instance_f32 scfft;   // FFT实例结构体
//u32 sampledata[sampledot]={0};//用于存放ADC采样数据的数组,高16位保存adc2 pa5， 低16位保存adc1 pa6
//float phase_difference=0; // 用于存放相位差的变量
//float freamp[50];//用于存放各次谐波频率和幅值的数组
#define FFT_LENGTH        256         //FFT长度，默认是1024点FFT
float fft_inputbuf[FFT_LENGTH*2];    //FFT输入输出数组，此数组为arm_cfft_radix4_f32的输入输出数组，前一个元素为实部，后一个为虚部，每两个元素代表一个点.
float fft_outputbuf[FFT_LENGTH];     //arm_cmplx_mag_f32() 幅度输出数组
arm_cfft_radix4_instance_f32 scfft;
int fft_outputbuf_int[FFT_LENGTH];
int fft_outputbuf_point[FFT_LENGTH];

void sys_init(void);  // 用于系统初始化
void draw_home_page(void);  //用于绘制坐标轴
void draw_ad_point(void); // 用于绘画ad轴的坐标图
void draw_da_point(void); // 用于绘画da轴的坐标图
void draw_ad_fft_point(void); // 用于对y_adc[]中的值做fft频谱分析，并绘图
void y_adc_append(u16 adcx); //线性表操作，用于向y_adc（即获得的adc读出值转换为坐标）中添加值
void y_dac_append(u16 dacval); // 同理
void lcdshow_adda_value(void); // 用于显示当前的dacval,ad da的实际电压值在屏幕上

void Adc_Init_t(void);
void Dma_ADC_Init_t(void);
void TIM3_Init(u16 arr,u16 psc);
// usart
u8 t;
u8 len;  //存储接受到的消息长度	
u16 times=0;  // 用于计时，每加5000

//ad da
u16 dacval=2048;  //dac输出值 0-4096
u16 adcx;  // adc读出值 0-4096
float temp; //暂时用于存储计算值
int timer_flag;

//key
u8 key;

//draw
u16 t_adc[ADC_X_MAX - ADC_X_MIN] = {0};  // 用于存储ADC轴的x（即t）坐标 220
u16 y_adc[ADC_X_MAX - ADC_X_MIN] = {0};  // 用于存储ADC轴的y坐标 220个
u16 t_dac[DAC_X_MAX - DAC_X_MIN] = {0};  // 用于存储DAC轴的x（即t）坐标 220
u16 y_dac[DAC_X_MAX - DAC_X_MIN] = {0};  // 用于存储DAC轴的y坐标 220个
u16 y_adc_tmp[ADC_X_MAX - ADC_X_MIN] = {0};  // 用于缓存上一次的数据，绘画擦除时用
u16 y_dac_tmp[DAC_X_MAX - DAC_X_MIN] = {0};  // 
u16 y_adc_dma_tmp[ADC_X_MAX - ADC_X_MIN] = {0};  // 用于存储dma搬运的adc的值

u8 data_num = 220; // 250 - 30;  共有220对数据
int idx = 0; // 数组索引值用于取出数组中元素
// 补偿值，用于画图时将点画在正确位置上，而不是一直在屏幕左上角
u16 x_adc_compensate  = 30; // ADC_X_MIN;  
u16 y_adc_compensate  = 320;  // ADC_Y_MAX
u16 x_dac_compensate  = 30; //DAC_X_MIN;
u16 y_dac_compensate  = 440;  // DAC_Y_MAX

//dma,adc1要使用数据流0，通道0
//#define SEND_BUF_SIZE 7044	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
//u8 SendBuff[SEND_BUF_SIZE];	//发送数据缓冲区
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
	for (j = 0; j < 220; j++) // 给t_adc赋值
	{
		t_adc[j] = j + 1;
	}
	for (n = 0; n < 220; n++) // 给t_dac赋值
	{
		t_dac[n] = n + 1;
	}
	
	for (t = 0; t < 220; t++)  //给y_adc赋值
	{	
		if (t < 80) y_adc[t] = t + 1;
		else {y_adc[t] = 80;}
	}
	
	for (a = 0; a < 220; a++)  // 给y_dac
	{	
//		if (a % 2 == 0) y_dac[a] = 80;  //方波
//		else {y_dac[a] = 0;}
//		if (a < 80) {y_dac[a] = a;}  //三角波
//		else if (a < 160 && a > 80){ y_dac[a] = a - 80;}
//		else if (a < 220 && a > 160) { y_dac[a] = a - 160;}
		double t = (double)a / SAMPLE_RATE;  //1khz正弦波
        double angular_frequency = 2.0 * 3.14159265358979323846 * FREQUENCY;
        double sin_value = AMPLITUDE * sin(angular_frequency * t);
		
		if (sin_value < 0) { sin_value = AMPLITUDE - abs(sin_value); y_dac[a] = sin_value;}
        else {y_dac[a] = (u16)sin_value + AMPLITUDE;}  //加上幅值让曲线显示在坐标轴中间
		
	}

	
	DAC_SetChannel1Data(DAC_Align_12b_R,dacval);//初始值
//	MYDMA_Config(DMA2_Stream0,DMA_Channel_0,(u32)&ADC1->DR,(u32)y_adc,220);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:220
//	ADC_DMACmd(ADC1, ENABLE);
//	MYDMA_Enable(DMA2_Stream0, 220);
	draw_da_point();
	draw_home_page();

	while (1) 
	{
		// 串口接受到消息时
		if(USART_RX_STA&0x8000)  // 接受状态标记位
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("\r\n\r\n");//插入换行
			
			if (modeflag == admode) 
			{
				modeflag = damode;
					//
				LCD_Clear(WHITE);
				draw_home_page();
				lcdshow_adda_value();
				draw_ad_point();
				
				for (a = 0; a < 220; a++)  // 给y_dac
				{	
					if (a % 10 == 0) y_dac[a] = 80;  //方波
					else {y_dac[a] = 0;}
			//		if (a < 80) {y_dac[a] = a;}  //三角波
			//		else if (a < 160 && a > 80){ y_dac[a] = a - 80;}
			//		else if (a < 220 && a > 160) { y_dac[a] = a - 160;}
//					double t = (double)a / SAMPLE_RATE;  //1khz正弦波
//					double angular_frequency = 2.0 * 3.14159265358979323846 * FREQUENCY;
//					double sin_value = AMPLITUDE * sin(angular_frequency * t);
					
//					if (sin_value < 0) { sin_value = AMPLITUDE - abs(sin_value); y_dac[a] = sin_value;}
//					else {y_dac[a] = (u16)sin_value + AMPLITUDE;}  //加上幅值让曲线显示在坐标轴中间
					
				}
				draw_da_point();
			}
			else if (modeflag == damode) 
			{
				modeflag = admode;
				LCD_Clear(WHITE);
				draw_home_page();
				lcdshow_adda_value();
				
				
				for (a = 0; a < 220; a++)  // 给y_dac
				{	
			//		if (a % 2 == 0) y_dac[a] = 80;  //方波
			//		else {y_dac[a] = 0;}
			//		if (a < 80) {y_dac[a] = a;}  //三角波
			//		else if (a < 160 && a > 80){ y_dac[a] = a - 80;}
			//		else if (a < 220 && a > 160) { y_dac[a] = a - 160;}
					double t = (double)a / SAMPLE_RATE;  //1khz正弦波
					double angular_frequency = 2.0 * 3.14159265358979323846 * FREQUENCY;
					double sin_value = AMPLITUDE * sin(angular_frequency * t);
					
					if (sin_value < 0) { sin_value = AMPLITUDE - abs(sin_value); y_dac[a] = sin_value;}
					else {y_dac[a] = (u16)sin_value + AMPLITUDE;}  //加上幅值让曲线显示在坐标轴中间
					
				}
				draw_da_point();
			}
			USART_RX_STA=0;  // 手动将自定义接受标记位置为0
		}
		else  // 串口没接收到时
		{
			if (modeflag == admode)
			{
				TIM_Cmd(TIM3, DISABLE);
	//			for (j = 0; j < 220; j++)
	//			{
	//				printf("dma adc value: %d\n",y_adc_dma_tmp[j]); 
	//			}
				times++;  //计时标志+1
				idx = times % 220;  //计算索引
	//			printf("idx val:%d\r\n", idx);//test

	//			dacval = (u16)(y_dac[idx] / 80 * 4096);  //test
				dacval = (int)(4096 / 80) * y_dac[idx];
	//			printf("trans dacval:%d\r\n", dacval);//插入换行

				DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//设置DAC值
		
				adcx=Get_Adc(ADC_Channel_5);
				
	//			printf("current adcx:%d\r\n\n", adcx);  //test
	//			printf("current dacval:%d\r\n", dacval);
				
				y_adc_append(adcx);  // 添加读取到的adc值
	//			if (times % 10000 == 0)  // 屏幕不需要太高刷新率所以100次计数刷新一次
	//			{
				draw_ad_point();
	//			}
				
				key=KEY_Scan(0);			  // 按下key_up暂停显示
				if(key==WKUP_PRES)
				{	
	//				BACK_COLOR = BLACK;
					LCD_ShowString(70,190,200,16,16,"<cupture paused!>"); //显示已经暂停
					lcdshow_adda_value();
	//				draw_ad_point();				
					delay_ms(300);
					while (1)				// 再次按下key_up可以继续显示
					{
						key = KEY_Scan(0);
						if (key == WKUP_PRES || key == KEY1_PRES)  //如果出现按下暂停后再按key1的情况，则继续进入key = KEY1_PRES的循环中
						{
							delay_ms(300);
	//						BACK_COLOR = WHITE;
							LCD_ShowString(70,190,200,16,16,"                 "); //清除显示：已经暂停	
							break;
						}
					}
				}
				if (key==KEY1_PRES)  //按下key1进行频谱分析
				{
					LED0=!LED0;	   //LED闪烁.

					LCD_ShowString(70,210,200,16,16,"FFT-ADC-capture ");
					draw_ad_fft_point();
					delay_ms(300);
					while (1)				// 再次按下key1可以继续显示
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

				LED0=!LED0;	   //LED闪烁.

			}

			else if (modeflag == damode)
			{
				TIM_Cmd(TIM3, ENABLE);
				
				key=KEY_Scan(0);			  // 按下key_up暂停显示
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
//					LCD_ShowString(70,190,200,16,16,"<cupture paused!>"); //显示已经暂停
//					lcdshow_adda_value();
//	//				draw_ad_point();				
//					delay_ms(300);
//					while (1)				// 再次按下key_up可以继续显示
//					{
//						key = KEY_Scan(0);
//						if (key == WKUP_PRES || key == KEY1_PRES)  //如果出现按下暂停后再按key1的情况，则继续进入key = KEY1_PRES的循环中
//						{
//							delay_ms(300);
//	//						BACK_COLOR = WHITE;
//							LCD_ShowString(70,190,200,16,16,"                 "); //清除显示：已经暂停	
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口1初始化波特率为115200USB连接 
	LED_Init();
	LCD_Init();					//LCD初始化
	Adc_Init(); 				//adc初始化
	KEY_Init(); 				//按键初始化
	Dac1_Init();				//DAC通道1初始化
	TIM3_Init(104, 399);//500us
//	Adc_Init_t();
//	Dma_ADC_Init_t();
//	my_adc_init();
//	my_dma_init(DMA2_Stream0,DMA_Channel_0,(u32)&ADC1->DR,(u32)y_adc,220);
}

void draw_home_page(void)
{	
//	LCD_Clear(WHITE);
	// 主页面page_home
	POINT_COLOR=BLACK; 
	LCD_ShowString(60,20,200,16,16,"Chen wei E2021157 CQU");	
	LCD_ShowString(70,40,200,16,16,"Wave Create Anlysis");	
//	LCD_ShowString(30,90,200,16,16,"CQU");
//	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
//	LCD_ShowString(30,130,200,16,16,"WK_UP:+  KEY1:-");	 
	POINT_COLOR=BLUE;//设置字体为蓝色      	 
	LCD_ShowString(30,100,200,16,16,"DAC VAL:");	      
	LCD_ShowString(30,120,200,16,16,"DAC VOL:0.000V");	      
	LCD_ShowString(30,140,200,16,16,"ADC VOL:0.000V");
//	// 屏幕估计只有500*300，所以x < 300,y < 500
	POINT_COLOR = BLACK;
	
	// ADC坐标轴,该轴x显示范围：30~250，y显示范围：240~320
	LCD_Draw_Circle(60,220,5);  //画个圆圈
	LCD_ShowString(70,210,200,16,16,"ADC cupture axis");
//	LCD_DrawRectangle(30, 240, 30, 320);  // 画竖线
//	LCD_DrawRectangle(30, 320, 250, 320);  // 画横线
//	LCD_ShowString(28,238,200,16,16, "^");
	LCD_ShowString(246,312,200,16,16, ">");
	LCD_DrawLine(30, 240, 30, 320);							//画竖线
	LCD_DrawLine(30, 320, 250, 320);							//画横线

	// DAC坐标轴，该轴x显示范围：30~250，y显示范围：360~440
//	LCD_DrawRectangle(30, 360, 30, 440);  // 画竖线
//	LCD_DrawRectangle(30, 440, 250, 440);  // 画横线
	LCD_ShowString(70,340,200,16,16, "DAC create axis");
//	LCD_ShowString(30,358,200,16,16, "^");
	LCD_ShowString(246,432,200,16,16, ">");
	LCD_DrawLine(30, 360, 30, 440);							//画竖线30
	LCD_DrawLine(30, 440, 250, 440);							//画横线
}

// 用于画ad的捕获曲线
void draw_ad_point(void)
{
	u8 num = data_num;
	int i,j;
	// 先擦除上一次绘制的曲线,通过画上一次的曲线，颜色为白色就可以覆盖掉，以达到擦除的效果
	/*
	为什么不使用LCD_Clear，使用这句屏闪过于严重
	*/
	for (j = 0; j < num; j++)
	{
		//减1 的原因是因为当y=0时，和x轴重合，如果擦除，可能会把
		LCD_Fast_DrawPoint(t_adc[j] + x_adc_compensate, y_adc_compensate - y_adc_tmp[j], WHITE);
	}
	
	// 绘制每一个点，并加上补偿的屏幕坐标值
	for (i = 0; i < num; i++)
	{
		//减1 的原因是因为当y=0时，和x轴重合，如果擦除，可能会把
		LCD_Fast_DrawPoint(t_adc[i] + x_adc_compensate, y_adc_compensate - y_adc[i], BLUE);
	}
	//draw_home_page();  //这句必须放在后面，后画桌面可以覆盖当y=0是的重合部分，不然会被擦除
}

// 用于画da的输出曲线
void draw_da_point(void)
{
	u8 num = data_num;
	u8 i,j;
	// 先擦除上一次绘制的曲线,通过画上一次的曲线，颜色为白色就可以覆盖掉，以达到擦除的效果
	for (j = 0; j < num; j++)
	{
		//减1 的原因是因为当y=0时，和x轴重合，如果擦除，可能会把
		LCD_Fast_DrawPoint(t_dac[j] + x_dac_compensate, y_dac_compensate - y_dac_tmp[j], WHITE);
	}
	
	for (i = 0; i < num; i++)
	{
		// 绘制每一个点，并加上补偿的屏幕坐标值
		LCD_Fast_DrawPoint(t_dac[i] + x_dac_compensate, y_dac_compensate - y_dac[i], RED);
	}
//	delay_ms(10);
	//draw_home_page();
}

// 用于向y_adc中添加坐标，线性操作
void y_adc_append(u16 adcx)
{
	
	
	u8 num = data_num;
	u8 i,j;
	u16 y_adc_value;
//	test += 1;
	// 在赋值之前，先把上一次的数据保存在y_adc_tmp中
	for (j = 0; j < num; j++)
	{
		y_adc_tmp[j] = y_adc[j];
	}
	
	for (i = 1; i < num; i++)
	{
		y_adc[num - i] = y_adc[num - i - 1];
	}
	if ((times % 500) == 0) printf("append adcx:%d\r\n\n", (int)(80 * adcx / 4096));
	// 归一化，将从寄存器读出值adcx（0-4096）转换到屏幕坐标轴上的对应值0-80（坐标轴的y轴像素共有80个）
	y_adc_value = (int)(80 * adcx / 4096); //(float)80 * (adcx / 4096);
	
	y_adc[0] = y_adc_value; //最后一步需要将新得到的值放到第一个位置上
}

// 用于向y_dac中添加坐标，线性操作
void y_dac_append(u16 dacval)
{
	u8 num = data_num;
	u8 i,j;
	u16 y_dac_value;

	// 在赋值之前，先把上一次的数据保存在y_dac_tmp中
	for (j = 0; j < num; j++)
	{
		y_dac_tmp[j] = y_dac[j];
	}
	
	for (i = 1; i < num; i++)
	{
		y_dac[num - i] = y_dac[num - i - 1];
	}
	if ((times % 500) == 0) printf("append dacval:%d\r\n\n", (int)(80 * dacval / 4096));
	// 归一化，将输入寄存器的值dacval（0-4096）转换到屏幕坐标轴上的对应值0-80（坐标轴的y轴像素共有80个）
	y_dac_value = (int)(80 * dacval / 4096);
	
	y_dac[0] = y_dac_value; //最后一步需要将新得到的值放到第一个位置上
}



//用与对采集的信号做fft频谱分析并绘图
void draw_ad_fft_point(void)
{
	char str[10]; // 频率字符串
	int freq = 0;
	u8 num = data_num;
	int cnt = 0;
	int index_start = 0;
	int index_end = 0; //用与计算频率时计数
	int i,j;
	uint32_t maxIndex, minIndex;
	float32_t maxValue, minValue;
	float32_t scale; //归一化系数
	
	//只清理adc的曲线
	LCD_Clear(WHITE);
	draw_home_page();
	lcdshow_adda_value();
	draw_da_point();
	
//	arm_cfft_radix4_instance_f32 scfft;
	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数
//	arm_cfft_radix4_init_f32(&scfft, 256, 0, 1);
	
	for(j=0;j < FFT_LENGTH;j++)  // 生成信号序列
	{
		if (j <= 220)
		{
			fft_inputbuf[2*j]= (float)y_adc[j];//生成实部
		}
		else if (j > 220)
		{
			fft_inputbuf[2*j]= 0;//生成实部
		}
		else if (j == 255) {fft_inputbuf[2*j+1]=0;break;}
		fft_inputbuf[2*j+1]=0;//虚部全部为0
	}
	

//    for(i=0;i<FFT_LENGTH;i++)//生成信号序列
//    {
//		  fft_inputbuf[2*i]=10+4.5*arm_sin_f32(2*PI*i*200/FFT_LENGTH)+\
//							   7.5*arm_sin_f32(2*PI*i*350/FFT_LENGTH);
//		 
//		  fft_inputbuf[2*i+1]=0;//虚部全部为0
//	 }

//	arm_cfft_radix4_f32(&scfft,fft_inputbuf);    //FFT计算（基4）
//	arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);    //把运算结果复数求模得幅值 
	//arm_cfft_sR_f32_len1024，该变量即为"arm_const_structs.h"提供的配置变量，包含头文件后，直接调用即可。
//	arm_cfft_f32(&arm_cfft_sR_f32_len1024,fft_inputbuf,0,1);
//	arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);    //把运算结果复数求模得幅值
//				 
	arm_cfft_radix4_f32(&scfft, fft_inputbuf);                      /* FFT计算（基4） */
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, 256);     /* 把运算结果复数求模得幅值 */
//	for (i = 0; i < FFT_LENGTH; i++)
//	{
//		printf("%f \n", fft_outputbuf[i]);
//	}

	//归一化处理
//	for (i = 0; i < FFT_LENGTH; i++)  //转整型
//	{
//		fft_outputbuf_int[i] = (int)fft_inputbuf[i];
//	}
	arm_max_f32(fft_outputbuf, FFT_LENGTH, &maxValue, &maxIndex);
	arm_min_f32(fft_outputbuf, FFT_LENGTH, &minValue, &minIndex);
	// 计算归一化系数
	
    scale = (80 - 0) / (maxValue - minValue);
 // 归一化数组并保存到 fft_outputbuf_point
    for (i = 0; i < FFT_LENGTH; i++)
    {
        fft_outputbuf_point[i] = (int)((fft_outputbuf[i] - minValue) * scale + 0);
    }

    // 打印归一化后的数组
//    for (i = 0; i < FFT_LENGTH; i++)
//    {
//        printf("%d\n", fft_outputbuf_point[i]);
//    }
	// 开始画点
	for (i = 0; i < num; i++)
	{
		//减1 的原因是因为当y=0时，和x轴重合，如果擦除，可能会把
		LCD_DrawLine(t_adc[i] + x_adc_compensate, y_adc_compensate - fft_outputbuf_point[i], t_adc[i] + x_adc_compensate, 320);
	}
	
	//计算频率
	for (i = 0; i < 220; i++) printf("%d\n", y_adc[i]);
	for (i = 0; i < 220; i++)
	{
		if(y_adc[i] > 70 && index_start < 1 && cnt == 0)
		{
			cnt += 1;
			index_start = i;
//			if (cnt == 0) {cnt += 1; index_start = i;}  //有方向的检测
		}
		if(y_adc[i] < 20  && index_end < 1 && cnt == 1)
		{
			cnt = 0;
			index_end = i;
//			if (cnt == 1) {cnt -= 1; index_end = i;}
			break;
		}
	}
	
	//index_start - index_end 就是半个周期的长度，采一个点用147/220=0.66818181...的时间则频率
	freq = (int)(1000/(2*(index_end - index_start)*0.6681));
	printf("freq:%d index_s:%d end:%d\n", freq, index_start, index_end);
    index_start = 0;
	index_end = 0;

//	freq = (int)(freq/1000);
    sprintf(str, "%d", freq);

    // 现在 str 中存储了整数 num 转换后的字符串
    printf("Converted string: %s\n", str);
	LCD_ShowString(170,140,200,16,16,"frequence:    ");
	//显示 数字频率
	LCD_ShowString(270,140,200,16,16,str);

}

void lcdshow_adda_value(void)
{
	dacval=DAC_GetDataOutputValue(DAC_Channel_1);//用于读取dac的输出值
//	dacval=DAC_GetDataOutputValue(DAC_Channel_1);//用于读取dac的输出值
	LCD_ShowxNum(94,100,dacval,4,16,0);     	   //显示DAC寄存器值
	temp=(float)dacval*(3.3/4096);			         //转换成电压值
	dacval=temp;
	LCD_ShowxNum(94,120,dacval,1,16,0);     	   //显示电压值整数部分
	temp-=dacval;
	temp*=1000;
	LCD_ShowxNum(110,120,temp,3,16,0X80); 	   //显示电压值的小数部分
// 	adcx=Get_Adc_Average(ADC_Channel_5,10);		//得到ADC转换值	  
	temp=(float)adcx*(3.3/4096);			        //得到ADC电压值
	adcx=temp;
	LCD_ShowxNum(94,140,temp,1,16,0);     	  //显示电压值整数部分
	temp-=adcx;
	temp*=1000;
	LCD_ShowxNum(110,140,temp,3,16,0X80); 	  //显示电压值的小数部分
}




void TIM3_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	NVIC_InitTypeDef NVIC_InitStruc;
	
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV2;
	TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStrue.TIM_Period=arr;
	TIM_TimeBaseInitStrue.TIM_Prescaler=psc;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //使能定时器时钟
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStrue);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	NVIC_InitStruc.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStruc.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruc.NVIC_IRQChannelPreemptionPriority=0X01;
	NVIC_InitStruc.NVIC_IRQChannelSubPriority=0X03;
	NVIC_Init(&NVIC_InitStruc);
	
	TIM_Cmd(TIM3,DISABLE);
}



//定时器中断
void TIM3_IRQHandler(void)
 {
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{
//		a += 1;
//		if (test == 220) {printf("%d", a);while(1);} //294次
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
//			printf("trans dacval:%d\r\n", dacval);//插入换行

		DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//设置DAC值
		
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}

// ADC初始化函数
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
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_5;  //adc 1和2 的通道
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	 GPIO_Init(GPIOA, &GPIO_InitStructure);
//	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE); //重置
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
//    ADC_InitStructure.ADC_NbrOfConversion =1;  //通道数
//	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_T3_TRGO;
//    ADC_Init(ADC1, &ADC_InitStructure);
//    ADC_Init(ADC2, &ADC_InitStructure);
//    ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_3Cycles);
//		
//   ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE); //多路转化完后触发dma
//   
//	ADC_DMACmd(ADC1, ENABLE); 
//	
//	ADC_Cmd(ADC1, ENABLE);
//    ADC_Cmd(ADC2, ENABLE);
//}
 
 
// DMA初始化函数，用于ADC数据的采集
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
//	DMA_InitStructure.DMA_BufferSize= data_num;  //数据长度220
//	DMA_InitStructure.DMA_Channel=DMA_Channel_0; 
//	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;	
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
//    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;	
//	
//	DMA_InitStructure.DMA_Memory0BaseAddr= (uint32_t)&y_adc_dma_tmp ;//要存入的值
//	
//	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_MemoryDataSize= DMA_MemoryDataSize_Word;
//	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;		
//	DMA_InitStructure.DMA_Mode=DMA_Mode_Circular;
//	
//		
//	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)0x40012308; //adc地址
//	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Word;
//	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority=DMA_Priority_High;
//	
// 
// 
//  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
////  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);  //使能中断
//  DMA_Cmd(DMA2_Stream0, ENABLE);
//	 
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;  //DMA2_Stream0中断
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;  //抢占优先级1
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;        //子优先级1
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道使能
//  NVIC_Init(&NVIC_InitStructure);
//}
 
//// 数据初始化函数，用于初始化ADC、DMA、串口和定时器
//void Data_Init()
//{
//	u32 idex;
//	float temp;	
//	Adc_Init();
//	Dma_ADC_Init();
//	uart_init(115200);
//	arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);//初始化scfft结构体，设定FFT相关参数     //FFT_LENGTH 4096
//	Tim3_Init(fft_arr-1,fft_psc-1);
//}
// 
// 
//// DMA中断服务函数，用于处理ADC数据采集完成后的操作
//void DMA2_Stream0_IRQHandler(void)  
//{
//	u32 idex;	//用于将采集到的数据赋值给fft_inputbuf[2*idex]的计数	
//  float bias_voltage2,HZ2,amplitude2,phase2,bias_voltage1,HZ1,amplitude1,phase1;
// 
// 
//	u8 temp[40];
//	int i;
//	u16   freamplen; // freamp长度的一半
//	
//	
//	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))  //判断DMA传输完成中断  
//    {
//		
//		TIM_Cmd(TIM3,DISABLE);//关闭时钟，进行计算			
//		//adc2 pa5
//		for(idex=0;idex<sampledot;idex++) //高16位fft，adc2 fft1 //sampledot==4096
//		{			
//			fft_inputbuf[2*idex]=(u16)(sampledata[idex]>>16)*(3.3/4096);    //生成输入信号实部
//			fft_inputbuf[2*idex+1]=0;//虚部全部为0
//		}
//		arm_cfft_radix4_f32(&scfft,fft_inputbuf);  //fft运算
//		arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值	
//		freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2);//寻找基波和谐波	
//		
//		bias_voltage2=fft_outputbuf[0]/FFT_LENGTH;//直流 
//		HZ2=freamp[0];//频率
//		amplitude2=freamp[1];//幅度
//		phase2=freamp[2];//相位
//		freamp[0]=0;freamp[1]=0;freamp[2]=0;
// 
//		//adc1 pa6
//		for(idex=0;idex<sampledot;idex++) //低16位fft ，adc1 fft2
//		{
//			 fft_inputbuf[2*idex]=(u16)(sampledata[idex])*(3.3/4096);    //生成输入信号实部
//			 fft_inputbuf[2*idex+1]=0;//虚部全部为0	，
//			
//		}	
//		arm_cfft_radix4_f32(&scfft,fft_inputbuf);  //fft运算
//		arm_cmplx_mag_f32(fft_inputbuf,fft_outputbuf,FFT_LENGTH);	//把运算结果复数求模得幅值
//		freamplen=fft_getpeak(fft_inputbuf,fft_outputbuf+1,freamp,FFT_LENGTH/2,10,5,0.2); //寻找基波和谐波	
//		
//		bias_voltage1=fft_outputbuf[0]/FFT_LENGTH;//偏置电压      
//		HZ1=freamp[0];//频率
//		amplitude1=freamp[1];//幅度
//		phase1=freamp[2];//相位
//		freamp[0]=0;freamp[1]=0;freamp[2]=0;
//		
//		phase_difference=phase2-phase1;
//	  if(phase_difference>180) phase_difference=phase_difference-180;
//	  if(phase_difference<-180) phase_difference=phase_difference+180;
//		
//		printf("\r\n");    //fft采样频率
//		printf("fft_sample_freq:%d\r\n",fft_sample_freq);    //fft采样频率 		
//		printf("bias_voltage1:%.2f\r\n",bias_voltage1); //偏置电压 
//		printf("bias_voltage2:%.2f\r\n",bias_voltage2); //偏置电压
// 
//		printf("HZ1:%.2f\r\n",HZ1);   //频率
//		printf("HZ2:%.2f\r\n",HZ2);//频率
//		
//		printf("amplitude1:%.2f\r\n",amplitude1); //幅值 
//		printf("amplitude2:%.2f\r\n",amplitude2);//幅值  
//		
//		printf("phase_difference:%.2f\r\n",phase_difference);//相位差        
//		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
//		
//	}	
//}
// 获取FFT峰值
//int fft_getpeak(float *inputx,float *input,float *output,u16 inlen,u8 x,u8 N,float y) //  intlen 输入数组长度，x寻找长度
//{                                                                           
//	int i,i2;
//	u32 idex;  //不同于上一个函数中的，因为他们在不同的函数中被定义
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
//				     output[3*outlen+2] = atan2(inputx[2*(i+idex+1)+1],inputx[2*(i+idex+1)])*180/3.1415926f;	//计算相位		   
//				     output[3*outlen+1] = 1.0*(2*datas)/FFT_LENGTH;   //计算幅度
//					   output[3*outlen] = 1.0*fft_sample_freq*(i+idex+1)/FFT_LENGTH;//计算频率
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
#define SEND_BUF_SIZE 8200	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.

u8 SendBuff[SEND_BUF_SIZE];	//发送数据缓冲区
const u8 TEXT_TO_SEND[]={"ALIENTEK Explorer STM32F4 DMA 串口实验"};	 

  
int main(void)
{ 
	u16 i;
	u8 t=0;
	u8 j,mask=0;
	float pro=0;//进度
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);     //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED 
 	LCD_Init();					//LCD初始化 
	KEY_Init(); 				//按键初始化 
 	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"DMA TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
	LCD_ShowString(30,130,200,16,16,"KEY0:Start");	 
	POINT_COLOR=BLUE;//设置字体为蓝色      	 
 //显示提示信息	
	j=sizeof(TEXT_TO_SEND);	   
	for(i=0;i<SEND_BUF_SIZE;i++)//填充ASCII字符集数据
    {
		if(t>=j)//加入换行符
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
		}else//复制TEXT_TO_SEND语句
		{
			mask=0;
			SendBuff[i]=TEXT_TO_SEND[t];
			t++;
		}   	   
    }		 
	POINT_COLOR=BLUE;//设置字体为蓝色	  
	i=0;
	while(1)
	{
		t=KEY_Scan(0);
		if(t==KEY0_PRES)  //KEY0按下
		{
			printf("\r\nDMA DATA:\r\n"); 	    
			LCD_ShowString(30,150,200,16,16,"Start Transimit....");
			LCD_ShowString(30,170,200,16,16,"   %") ;     //显示百分号       
      USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
			MYDMA_Enable(DMA2_Stream7,SEND_BUF_SIZE);     //开始一次DMA传输！	  
		    //等待DMA传输完成，此时我们来做另外一些事，点灯
		    //实际应用中，传输数据期间，可以执行另外的任务
		    while(1)
		    {
				if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
				{ 
					DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
					break; 
		        }
				pro=DMA_GetCurrDataCounter(DMA2_Stream7);//得到当前还剩余多少个数据
				pro=1-pro/SEND_BUF_SIZE;//得到百分比	  
				pro*=100;      			    //扩大100倍
				LCD_ShowNum(30,170,pro,3,16);	  
		    }			    
			LCD_ShowNum(30,170,100,3,16);//显示100%	  
			LCD_ShowString(30,150,200,16,16,"Transimit Finished!");//提示传送完成
		}
		i++;
		delay_ms(10);
		if(i==20)
		{
			LED0=!LED0;//提示系统正在运行	
			i=0;
		}		   
	}		    
}

*/

/*
	u8 t;
	u8 len;	
	u16 times=0;  // 用于计时，每加5000
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(115200);	//串口初始化波特率为115200 
	LED_Init();
	while(1)
	{
		if(USART_RX_STA&0x8000)  // 接受状态标记位
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("\r\n您发送的消息为:\r\n");
			for(t=0;t<len;t++)
			{
				USART_SendData(USART1, USART_RX_BUF[t]);         //向串口1发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}else
		{
			times++;
			if(times%5000==0)
			{
				printf("\r\nALIENTEK 探索者STM32F407开发板 串口实验\r\n");
				printf("正点原子@ALIENTEK\r\n\r\n\r\n");
			}
			if(times%200==0)printf("请输入数据,以回车键结束\r\n");  
			if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
			delay_ms(10);   
		}
	}
	*/
	
	/*
	u16 adcx;
	float temp;
 	u8 t=0;	 
	u16 dacval=0;  //dac读出值
	u8 key;	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);      //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
	LED_Init();					//初始化LED 
 	LCD_Init();					//LCD初始化
	Adc_Init(); 				//adc初始化
	KEY_Init(); 				//按键初始化
	Dac1_Init();		 		//DAC通道1初始化	
	POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"Explorer STM32F4");	
	LCD_ShowString(30,70,200,16,16,"DAC TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2014/5/6");	 
	LCD_ShowString(30,130,200,16,16,"WK_UP:+  KEY1:-");	 
	POINT_COLOR=BLUE;//设置字体为蓝色      	 
	LCD_ShowString(30,150,200,16,16,"DAC VAL:");	      
	LCD_ShowString(30,170,200,16,16,"DAC VOL:0.000V");	      
	LCD_ShowString(30,190,200,16,16,"ADC VOL:0.000V");
 	
  DAC_SetChannel1Data(DAC_Align_12b_R,dacval);//初始值为0	
	while(1)
	{
		t++;
		key=KEY_Scan(0);			  
		if(key==WKUP_PRES)
		{		 
			if(dacval<4000)dacval+=200;
			DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//设置DAC值
		}else if(key==2)	
		{
			if(dacval>200)dacval-=200;
			else dacval=0;
			DAC_SetChannel1Data(DAC_Align_12b_R, dacval);//设置DAC值
		}	 
		if(t==10||key==KEY1_PRES||key==WKUP_PRES) 	//WKUP/KEY1按下了,或者定时时间到了
		{	  
 			adcx=DAC_GetDataOutputValue(DAC_Channel_1);//读取前面设置DAC的值
			LCD_ShowxNum(94,150,adcx,4,16,0);     	   //显示DAC寄存器值
			temp=(float)adcx*(3.3/4096);			         //得到DAC电压值
			adcx=temp;
 			LCD_ShowxNum(94,170,temp,1,16,0);     	   //显示电压值整数部分
 			temp-=adcx;
			temp*=1000;
			LCD_ShowxNum(110,170,temp,3,16,0X80); 	   //显示电压值的小数部分
 			adcx=Get_Adc_Average(ADC_Channel_5,10);		//得到ADC转换值	  
			temp=(float)adcx*(3.3/4096);			        //得到ADC电压值
			adcx=temp;
 			LCD_ShowxNum(94,190,temp,1,16,0);     	  //显示电压值整数部分
 			temp-=adcx;
			temp*=1000;
			LCD_ShowxNum(110,190,temp,3,16,0X80); 	  //显示电压值的小数部分
			LED0=!LED0;	   
			t=0;
		}	    
		delay_ms(10);	 
	}
		*/
//}
