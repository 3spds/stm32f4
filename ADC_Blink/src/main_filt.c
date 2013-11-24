#include <../inc/stm32f4xx.h>
#include <../inc/stm32f4xx_gpio.h>
#include <../inc/stm32f4xx_rcc.h>
#include <../inc/stm32f4xx_adc.h>
#include <../inc/stm32f4xx_dac.h>
#include <../inc/stm32f4xx_dma.h>
#include <../inc/stm32f4xx_tim.h>
#include <../inc/arm_math.h>
#include <../inc/stm32f4_discovery.h>
#include <../inc/stm32f4xx_conf.h>
#include <stdio.h>
#include <math.h>

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
#define DAC_DHR12R2_ADDRESS 0x40007414
#define VECSIZE 32
#define nfilters 128

__IO uint16_t ADC3ConvertedValue[4] = {0, 0, 0, 0};
__IO uint16_t DAC1ConvertedValue = 0;
__IO int i = 0;
__IO long counter = 0;
__IO uint16_t outvalue = 0;
uint16_t pre_filter = 0;
__IO float ADC3ConvertedVoltage[4];
__IO uint16_t vector_a[VECSIZE];
__IO uint16_t vector_b[VECSIZE];
__IO uint16_t vector_c[VECSIZE];
__IO uint16_t vector_d[VECSIZE];
__IO uint16_t sig[VECSIZE];
uint32_t nconversions = 4;
int j = 0;
int k = 0;
//float sinusoid = 0;
GPIO_InitTypeDef      GPIO_InitStruct;
ADC_InitTypeDef       ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef       DMA_InitStructure;
DAC_InitTypeDef       DAC_InitStructure;

typedef struct filter {
	float DOUTBUF[VECSIZE];		/* Delayed filter output feedback */
	float DINBUF[VECSIZE];		/* Delayed filter input */
	float f_b0, f_b1, f_b2;	/* filter cooefficients*/
	float f_a0, f_a1, f_a2, f_inv_a0;
	float f_f0, f_w0;		/* center frequency for the filter */
	float f_ff1, f_ff2, f_ff3; /* feed-forward coeffs */
	float f_fb1, f_fb2; /* feed-back coeffs */
	float f_inv_srate;	/* inverse sample rate */
	float f_alpha, f_rq;
} filter;

//filter ap_1;
//filter ap_2;
//filter ap_3;
//filter ap_4;

filter filters[nfilters];

void Delay(uint32_t nCount);
void init_TIM2(void);
void TIM2_IRQHandler(void);
void init_TIM3(void);
void TIM3_IRQHandler(void);
void init_TIM4(void);
void TIM4_IRQHandler(void);
void init_TIM6(void);
void TIM6_DAC_IRQHandler(void);
void init_GPIOA(void);
void init_GPIOC(void);
void init_GPIOD(void);
void init_ADC3(void);
void init_DMA1(void);
void init_DMA2(void);
void init_DAC(void);
void ap_filter_coefs(filter *x);
uint16_t apply_filter(uint16_t input, filter * x);
void init_filter(filter * x);

void Delay(__IO uint32_t nCount)
{
    while(nCount--)
    {
    }
}
//timer 2
void init_TIM2(void)
{
	//do it by registers!
	NVIC->ISER[0] |= 1<<(TIM2_IRQn); //Interrupt set-enable register, enable TIM2 IRQ
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable TIM2 clock
	TIM2->PSC = 0xFF; //prescaler
	TIM2->DIER |= TIM_DIER_UIE; //dma / interrupt enable register -> update interrupt enable
	TIM2->ARR = 0xFFFF; //auto reload register
	TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload preload and counter enable
	TIM2->EGR = 1; //tells TIM2 there's been an update
}
void TIM2_IRQHandler(void) //blinker
{
	//if status register & update interrupt flag are both enabled,
	if (TIM2->SR & TIM_SR_UIF)
	{
		GPIOD->ODR ^= (1 << 13);	//toggle D13
	}
	TIM2->SR = 0x0; //reset TIM2's status register
}
//timer 3
void init_TIM3(void)
{
	//do it by registers!!
	NVIC->ISER[0] |= 1<<(TIM3_IRQn); //Interrupt set-enable register, enable TIM3 IRQ
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 clock
	TIM3->PSC = 0xFF; //prescaler - count with peripheral clock
	TIM3->DIER |= TIM_DIER_UIE; //dma / interrupt enable register -> update interrupt enable
	TIM3->ARR = 0x2F00; //auto reload register
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload enable and counter enable
	TIM3->EGR = 1; //tells TIM3 there's been an update
}
void TIM3_IRQHandler(void) //adc float converter
{
	//if status register & update interrupt flag are both enabled,
	if (TIM3->SR & TIM_SR_UIF)
	{
		GPIOD->ODR ^= (1 << 15);	//toggle D15
//		for(j = 0; j<nconversions; j++)
//		{ //convert adc inputs to float
//			ADC3ConvertedVoltage[j] = ((float)(ADC3ConvertedValue[j])/4096);
//		}
//		(ap_1.f_w0) = ((float)ADC3ConvertedValue[0]*M_PI)/4096;
//		(ap_3.f_rq) = (ap_2.f_rq) = (ap_1.f_rq) = (ADC3ConvertedVoltage[1]);

//		ap_filter_coefs(&ap_1);
//		ap_filter_coefs(&ap_2);
//		ap_filter_coefs(&ap_3);
		for(k=0;k<nfilters;k++)
		{
			filters[k].f_rq = (float)ADC3ConvertedValue[1]/1024;
			filters[k].f_w0 = (float)ADC3ConvertedValue[0]/4096;
			ap_filter_coefs(&filters[k]);
		}

	}
	TIM3->SR = 0x0; //reset TIM3's status register
}
//timer 4
void init_TIM4(void)
{
	//do it by registers!
	NVIC->ISER[0] |= 1<<(TIM4_IRQn); //Interrupt set-enable register, enable TIM4 IRQ
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 clock
	TIM4->PSC = 0x0; //prescaler - count with peripheral clock
	TIM4->DIER |= TIM_DIER_UIE; //dma / interrupt enable register -> update interrupt enable
	TIM4->ARR = 0xFF; //auto reload register : counts to 255
	TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload enable and counter enable
	TIM4->EGR = 1; //tells TIM4 there's been an update
}
void TIM4_IRQHandler(void) //output processor
{
	//if status register & update interrupt flag are both enabled,
	if (TIM4->SR & TIM_SR_UIF){};
	TIM4->SR = 0x0; //reset TIM4's status register
}
//timer 6
void init_TIM6(void)
{
	//do it mostly by registers!
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
//	NVIC->ISER[1] |= (uint32_t)(1 << ((uint32_t)((int32_t)TIM6_DAC_IRQn) & (uint32_t)0x1F)); /* enable interrupt */
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //enable tim6 rcc
	TIM6->ARR = 0x01; //auto reload register
    TIM6->PSC = 0xF0; //prescaler
    TIM6->DIER |= TIM_DIER_UIE; // update interrupt enable
    TIM6->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload preload and counter enable
	TIM6->CR2 &= (uint16_t)~TIM_CR2_MMS; //reset mms
	TIM6->CR2 |=  TIM_TRGOSource_Update; //select trgo source
    TIM6->EGR = 1; //update
}
void TIM6_DAC_IRQHandler(void)
{
	//if sr & uif are both enabled
	if(TIM6->SR & TIM_SR_UIF)
	{
		GPIOD->ODR ^= (1<<14); //toggle D14
		vector_a[i] = ADC3ConvertedValue[0];
        vector_b[i] = ADC3ConvertedValue[1];
        vector_c[i] = ADC3ConvertedValue[2];
        vector_d[i] = ADC3ConvertedValue[3];
//		outvalue+=ADC3ConvertedValue[0]; //pseudorandom number generation
		outvalue+=ADC3ConvertedValue[0];
		outvalue%=ADC3ConvertedValue[1];
		outvalue+=ADC3ConvertedValue[2];
//		pre_filter = (outvalue%=4096);
		outvalue%=4096;
//		outvalue+=sig[i];
		outvalue>>=1;
//		outvalue = (uint32_t)(outvalue*0.5) + (uint32_t)(sig[(VECSIZE+i)%VECSIZE]*0.5);
//		outvalue = apply_filter(outvalue, &ap_1);
//		outvalue = apply_filter(outvalue, &ap_2);
//		outvalue = apply_filter(outvalue, &ap_3);
//		outvalue = apply_filter(outvalue, &ap_4);
		for(k = 0; k<nfilters; k++)
		{
			outvalue = apply_filter(outvalue, &filters[k]);
		}
//    	DAC1ConvertedValue = (uint16_t)(0.7*outvalue) + (uint16_t)(0.3*sig[(VECSIZE/9+i)%VECSIZE]);
		DAC1ConvertedValue = sig[i] = outvalue;
		i++;
        i%=VECSIZE;
	}
	TIM6->SR = 0x0;
}

void init_GPIOA(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//GPIOA init (outputs)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;		  // we want to configure PA04
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; 	  // we want it to be analog
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
}
void init_GPIOC(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	//GPIOC init (inputs)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; //set GPIO clock speed to 100MHz
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void init_GPIOD(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//GPIOD init (indicators)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // we want to configure all LED GPIO pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 	// this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
    GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
}

void init_ADC3(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
    //ADC common init
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    //ADC3 init
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = nconversions;
    ADC_InitStructure.ADC_ExternalTrigConv=0x00;
    ADC_Init(ADC3, &ADC_InitStructure);
    // ADC3 regular configuration
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 3, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 4, ADC_SampleTime_3Cycles);
	// Enable DMA request after last transfer
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
    ADC_DMACmd(ADC3, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
}

void init_DMA1(void)
{
	//memory -> DAC
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream6);
    DMA_InitStructure.DMA_Channel = DMA_Channel_7;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DAC1ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream6, ENABLE);
}
void init_DMA2(void)
{
	//ADC3 -> memory
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = nconversions;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);
}

void init_DAC(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    //DAC2
    //channel 2 config
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_2, &DAC_InitStructure);
    DAC_Cmd(DAC_Channel_2, ENABLE);
    DAC_DMACmd(DAC_Channel_2, ENABLE);
}

void ap_filter_coefs(filter *x)
{
	float cosw0, sinw0, alpha, w0;
		//x->f_rq = 2;
		//w0 = x->f_w0 = M_PI / 9;
		//	intermediate values
		cosw0 = cos(x->f_w0);
		sinw0 = sin(x->f_w0);
		alpha = sinw0*0.5*(x->f_rq);
		//caclulate filter coeffs
		x->f_b0 = 1-alpha;
		x->f_b1 = -2*cosw0;
		x->f_b2 = 1+alpha;
		x->f_a0 = 1+alpha;
		x->f_a1 = -2*cosw0;
		x->f_a2 = 1-alpha;

		x->f_ff1 = x->f_b0 / x->f_a0;
		x->f_ff2 = x->f_b1 / x->f_a0;
		x->f_ff3 = x->f_b2 / x->f_a0;
		x->f_fb1 = x->f_a1 / x->f_a0;
		x->f_fb2 = x->f_a2 / x->f_a0;
}

uint16_t apply_filter(uint16_t input, filter * x)
{
	x->DINBUF[i] = ((float)input/2048)-1.f;
	//x->DOUTBUF[i] = x->DINBUF[i];

	x->DOUTBUF[i] =
		(x->f_ff1) * x->DINBUF[i]
		+ (x->f_ff2) * x->DINBUF[(VECSIZE+i-1)%VECSIZE]
		+ (x->f_ff3) * x->DINBUF[(VECSIZE+i-2)%VECSIZE]
	- (x->f_fb1) * x->DOUTBUF[(VECSIZE+i-1)%VECSIZE]
	- (x->f_fb2) * x->DOUTBUF[(VECSIZE+i-2)%VECSIZE];
	return (uint16_t)((x->DOUTBUF[i]+1)*2048);
	//return input;
}

void init_filter(filter * x)
{
	j = VECSIZE;
	while(j--)
	{
		x->f_w0 = M_PI / 9;
		x->f_rq = 1;
		x->DINBUF[j] = x->DOUTBUF[j] = 0;
	}
}

int main(void)
{
//init:
    DAC_DeInit();
	init_GPIOA();
    init_GPIOC();
    init_GPIOD();
    init_DMA1();
    init_DMA2();
    init_TIM2();
    init_TIM3();
    init_TIM4();
    init_TIM6();
    init_ADC3();
	init_DAC();
//	init_filter(&ap_1);
//	init_filter(&ap_2);
//	init_filter(&ap_3);
//	init_filter(&ap_4);
	for(k = 0; k<nfilters; k++)
	{
		init_filter(&filters[k]);
	}
//	ap_filter_coefs(&ap_1);
//	ap_filter_coefs(&ap_2);
//	ap_filter_coefs(&ap_3);
//	ap_filter_coefs(&ap_4);
	for(k = 0; k<nfilters; k++)
	{
		ap_filter_coefs(&filters[k]);
	}
    ADC_SoftwareStartConv(ADC3);
    while (1)
    {
		counter++;
    }
}
