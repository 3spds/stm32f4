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
#include <stdlib.h>
#include <math.h>

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
#define DAC_DHR12R2_ADDRESS 0x40007414
#define VECSIZE 4
#define DELSIZE 6000
#define nfilters 800
#define RATE 65536
#define gsize 18

uint16_t ADC3ConvertedValue[4] = {0, 0, 0, 0};
uint16_t DAC1ConvertedValue[VECSIZE];
uint16_t DAC2ConvertedValue[VECSIZE];
long i = 0;
long counter = 0;
float32_t outvalue = 0;
float32_t fbvalue = 0;
float32_t fb = 0;
float32_t gate = 0.999999;
float32_t modrate = 1.0;
int gcounter = 0;
uint16_t outints[VECSIZE];
uint16_t pre_filter = 0;
float ADC3ConvertedVoltage[4];
float32_t vector_a[VECSIZE];
float32_t vector_b[VECSIZE];
float32_t vector_c[VECSIZE];
float32_t vector_d[DELSIZE];
uint16_t sig[VECSIZE];
float32_t coefTable[nfilters * 5];
float32_t gamut[gsize] = {1, 1.0667, 1.2, 1.25, 1.422, 1.5, 1.667, 1.8, 2, 2.1340, 2.2770, 2.5608, 2.6675, 3.0345, 3.5574, 3.8412, 4, 4.2680};
uint16_t gindex = 0;
uint16_t g2index = 0;
float32_t coef_single[5] = {0.707927287, -1.60492671, 1, 1.60492671, -0.707927287};
float32_t coef_single_prev[5] = {0.707927287, -1.60492671, 1, 1.60492671, -0.707927287};
float32_t biquadState[2*nfilters];

uint32_t nconversions = 4;
int j = 0;
int k = 0;

GPIO_InitTypeDef      GPIO_InitStruct;
ADC_InitTypeDef       ADC_InitStructure;
ADC_CommonInitTypeDef ADC_CommonInitStructure;
DMA_InitTypeDef       DMA_InitStructure;
DAC_InitTypeDef       DAC_InitStructure;
arm_biquad_cascade_df2T_instance_f32 S1;

typedef struct phasor {
    float32_t prev_add;
    float32_t slew;
	float32_t add;
	float32_t mul;
	float32_t out;
} phasor;


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

float32_t run_phasor(phasor *);

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
	TIM2->PSC = 0xFFFF; //prescaler
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
		run_fsr(&noise2);
		g2index = noise2.out;
		noise.add = g2index;
		noise3.add = g2index + 4 % gsize;
//		noise.add = (uint16_t)(ADC3ConvertedVoltage[2]*gsize);
	}
	TIM2->SR = 0x0; //reset TIM2's status register
}
//timer 3
void init_TIM3(void)
{
	//do it by registers!!
	NVIC->ISER[0] |= 1<<(TIM3_IRQn); //Interrupt set-enable register, enable TIM3 IRQ
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM3 clock
	TIM3->PSC = 0x10; //prescaler - count with peripheral clock
	TIM3->DIER |= TIM_DIER_UIE; //dma / interrupt enable register -> update interrupt enable
	TIM3->ARR = 0xC000; //auto reload register
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload enable and counter enable
	TIM3->EGR = 1; //tells TIM3 there's been an update
}
void TIM3_IRQHandler(void) //adc float converter
{
	//if status register & update interrupt flag are both enabled,
	if (TIM3->SR & TIM_SR_UIF)
	{
//		GPIOD->ODR ^= (1 << 15);	//toggle D15
		for(j = 0; j<nconversions; j++)
		{ //convert adc inputs to float
			ADC3ConvertedVoltage[j] = ((float)(ADC3ConvertedValue[j])/4096);
		}

//		(ap.f_w0) = ADC3ConvertedVoltage[1];
        modrate = ((1-(float32_t)ADC3ConvertedVoltage[0])*10)+5;
//        modrate = 10.1;
        ADC3ConvertedVoltage[1] += 0.3;
        ADC3ConvertedVoltage[1] *= ADC3ConvertedVoltage[1];
        (ap.f_w0) = (0.3*ADC3ConvertedVoltage[1]*sin((float32_t)i/modrate*2*3.14159)/2) + (ADC3ConvertedVoltage[1]*0.5);
		(ap.f_rq) = (0.7*ADC3ConvertedVoltage[2]) + 0.06;

		ap_filter_coefs(&ap);

		coef_single[0] = (0.8*ap.f_ff1)+(0.2*coef_single_prev[0]);
		coef_single[1] = (0.8*ap.f_ff2)+(0.2*coef_single_prev[1]);
		coef_single[2] = (0.8*ap.f_ff3)+(0.2*coef_single_prev[2]);
		coef_single[3] = (0.8*ap.f_fb1)+(0.2*coef_single_prev[3]);
		coef_single[4] = (0.8*ap.f_fb2)+(0.2*coef_single_prev[4]);

		for(j=0; j<5; j++)
		{
//			coef_single[j] = 0.5*(ap.f_ff1 + coef_single_prev[j]);
			coef_single_prev[j] = coef_single[j];
		}

		for(j = 0; j<(nfilters*5); j++) //repeat coefs nfilters times
		{
			coefTable[j] = coef_single[(j%5)];
		}
//        noise.add = (uint16_t)ADC3ConvertedVoltage[1]*60000;
//        noise.mul = (uint16_t)ADC3ConvertedVoltage[2]*60000;
//        noise.mod = (uint16_t)ADC3ConvertedVoltage[3]*60000;
//		noise.add = (uint16_t)(sin(((float32_t)i)/10000000)*30000 + 30000);
//		noise.mul = (uint16_t)(sin(((float32_t)i)/10040500)*30000 + 30000);
//		noise.mod = (uint16_t)(sin(((float32_t)i)/10500300)*30000 + 30000);

//		ap_filter_coefs(&ap_2);
//		ap_filter_coefs(&ap_3);
//		for(k=0;k<nfilters;k++)
//		{
//			filters[k].f_rq = (float32_t)ADC3ConvertedValue[1]/4096;
//			filters[k].f_w0 = (float32_t)ADC3ConvertedValue[0]/4096;
//			ap_filter_coefs(&filters[k]);
			//fb = (float32_t)ADC3ConvertedValue[2]/4096;
//		}


	}
	TIM3->SR = 0x0; //reset TIM3's status register
}
//timer 4
void init_TIM4(void)
{
	//do it by registers!
	NVIC->ISER[0] |= 1<<(TIM4_IRQn); //Interrupt set-enable register, enable TIM4 IRQ
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // enable TIM4 clock
	TIM4->PSC = 0xFF; //prescaler - count with peripheral clock
	TIM4->DIER |= TIM_DIER_UIE; //dma / interrupt enable register -> update interrupt enable
	TIM4->ARR = 0xFFFF; //auto reload register
	TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload enable and counter enable
	TIM4->EGR = 1; //tells TIM4 there's been an update
}
void TIM4_IRQHandler(void) //output processor
{
	//if status register & update interrupt flag are both enabled,
	if (TIM4->SR & TIM_SR_UIF){
		GPIOD->ODR ^= (1 << 15);	//toggle D15
        run_fsr(&noise);
        run_fsr(&noise3);
	}
	TIM4->SR = 0x0; //reset TIM4's status register
}
//timer 6
void init_TIM6(void)
{
	//do it mostly by registers!
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
//	NVIC->ISER[1] |= (uint32_t)(1 << ((uint32_t)((int32_t)TIM6_DAC_IRQn) & (uint32_t)0x1F)); /* enable interrupt */
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; //enable tim6 rcc
	TIM6->ARR = 0x02; //auto reload register
    TIM6->PSC = 0x100; //prescaler
    TIM6->DIER |= TIM_DIER_UIE; // update interrupt enable
    TIM6->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload preload and counter enable
	TIM6->CR2 &= (uint16_t)~TIM_CR2_MMS; //reset mms
	TIM6->CR2 |=  TIM_TRGOSource_Update; //select trgo source
    TIM6->EGR = 1; //update
}
void TIM6_DAC_IRQHandler(void)
{
	int l = 0;
	//if sr & uif are both enabled
	if(TIM6->SR & TIM_SR_UIF)
	{
		GPIOD->ODR ^= (1<<14); //toggle D14
//		vector_a[i] = ADC3ConvertedValue[0];
//      vector_b[i] = ADC3ConvertedValue[1];
//      vector_c[i] = ADC3ConvertedValue[2];
//      vector_d[i] = ADC3ConvertedValue[3];
//		outvalue+=ADC3ConvertedValue[0]; //pseudorandom number generation
//		outint+=ADC3ConvertedValue[0];
//		outint%=ADC3ConvertedValue[1];
//		outint+=ADC3ConvertedValue[2];
//		pre_filter = (outvalue%=4096);
//		outint%=4096;
//		outvalue+=sig[i];
//		outint>>=1;

//		outvalue = (uint32_t)(outvalue*0.5) + (uint32_t)(sig[(VECSIZE+i)%VECSIZE]*0.5);
//		outvalue = apply_filter(outvalue, &ap_1);
//		outvalue = apply_filter(outvalue, &ap_2);
//		outvalue = apply_filter(outvalue, &ap_3);
//		outvalue = apply_filter(outvalue, &ap_4);
		//outvalue = gate * run_fsr(&noise);
//		outvalue = gate;
		for(l = 0; l<VECSIZE; l++) //generate noise vector
		{
			vector_a[l] = (0.3*run_phasor(&saw))*((1-(ADC3ConvertedVoltage[0]*0.1)));
			vector_a[l] *= ((float32_t)(ADC3ConvertedValue[3])/4096);
//			vector_a[l] = (ADC3ConvertedValue[3]);

//			vector_a[l] *= (run_fsr(&noise));
//			vector_a[l] *= rand() / (RAND_MAX/2) - 1;
//			vector_c[l] = vector_a[l];
//			if(ADC3ConvertedValue[3] > 2048) GPIOD->ODR ^= (1<<12);
//			vector_a[l] += ((ADC3ConvertedVoltage[0])+0.1*vector_b[l]*sin(((float32_t)i/100.0)*3.14159*2));
			vector_a[l] += (((ADC3ConvertedVoltage[0])+0.5)*vector_d[i%DELSIZE]);
			vector_a[l] += (((ADC3ConvertedVoltage[0])+0.25)*vector_d[(i+(uint32_t)(DELSIZE*0.5))%DELSIZE]);
			vector_a[l] += (((ADC3ConvertedVoltage[0])+0.5)*vector_d[(i+(uint32_t)(DELSIZE*0.8))%DELSIZE]);
			vector_a[l] += (((1-ADC3ConvertedVoltage[0])+0.5)*vector_d[(i+(uint32_t)(DELSIZE*0.3))%DELSIZE]);
			vector_a[l] *= 0.6;
            if(vector_a[l]>1.5574) vector_a[l] = 1.5574;
			else if(vector_a[l]<-1.5574) vector_a[l] = -1.5574;
			vector_a[l] = atanf(vector_a[l]); //it's clipping
		}
		arm_biquad_cascade_df2T_f32(&S1, vector_a, vector_b, VECSIZE);
//		outvalue = vector_a[i];
		for(l = 0; l<VECSIZE; l++) //convert float vector to int vector
		{
		    vector_b[l] = vector_b[l]*(ADC3ConvertedVoltage[2]+0.7) + vector_a[l]*(1-ADC3ConvertedVoltage[2]);
		    vector_b[l] *= 0.25;
			if(vector_b[l]>1.5574) vector_b[l] = 1.5574;
			else if(vector_b[l]<-1.5574) vector_b[l] = -1.5574;
			vector_b[l] = atanf(vector_b[l]); //it's clipping
			vector_d[i%DELSIZE] = vector_b[l];
			DAC1ConvertedValue[l] = (uint16_t)(((vector_b[l])+1)*2047);
		}
		i++;

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
    ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 1, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 2, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_3Cycles);
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
    DMA_InitStructure.DMA_BufferSize = VECSIZE;
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
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream6, ENABLE);

    /* DMA1_Stream5 channel7 configuration **************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_7;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&DAC1ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = VECSIZE;
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
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5, ENABLE);
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
    //DAC1
    //channel 1 config
    DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
    DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &DAC_InitStructure);
    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void ap_filter_coefs(filter *x)
{
	float32_t cosw0, sinw0, alpha;
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
		x->f_fb1 = -1*(x->f_a1) / x->f_a0;
		x->f_fb2 = -1*(x->f_a2) / x->f_a0;
}

float32_t apply_filter(float32_t input, int i, filter *x)
{
	x->DINBUF[i] = input;
	x->DOUTBUF[i] =
		(x->f_ff1) * x->DINBUF[i]
		+ (x->f_ff2) * x->DINBUF[(VECSIZE+i-1)%VECSIZE]
		+ (x->f_ff3) * x->DINBUF[(VECSIZE+i-2)%VECSIZE]
	+ (x->f_fb1) * x->DOUTBUF[(VECSIZE+i-1)%VECSIZE]
	+ (x->f_fb2) * x->DOUTBUF[(VECSIZE+i-2)%VECSIZE];
	return x->DOUTBUF[i];
}

void init_filter(filter * x)
{
	j = VECSIZE;
	while(j--)
	{
		x->f_w0 = M_PI / 9;
		x->f_rq = 0.01;
		x->DINBUF[j] = x->DOUTBUF[j] = 0;
	}
}

float32_t run_fsr(fsr * x)
{
	uint16_t out, mod, add, mul;
	out = x->out;
	mod = x->mod;
	add = x->add;
	mul = x->mul;
	out += add;
	out %= mod;
    out *= mul;
	x->out = (out %= 4090);
//	x->out = out *= (4092 / mod);
	return ((((float32_t)out )/ 2048) - 1);
}

float32_t run_phasor(phasor * x)
{
    float32_t slew = x->slew;
    float32_t add = (slew)*(x->prev_add) + (1-slew)*(x->add);
    float32_t mul = x->mul;
    float32_t out = x->out;
    out += add;
    out = fmodf(out, 1);
    x->out = out;
    x->prev_add = add;
    return (out*2-1)*mul;
}

int main(void)
{
	WWDG_SetPrescaler(WWDG_Prescaler_8);
//init:
	for(k=0; k<VECSIZE; k++) //initialize output vector
	{
		DAC1ConvertedValue[k] = 0;
		DAC2ConvertedValue[k] = 0;
	}

	for(k = 0; k<VECSIZE; k++)
	{
		vector_a[k] = 0.0;
		vector_b[k] = 0.0;
	}

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
    ADC_SoftwareStartConv(ADC3);
    while (1)
    {
		counter++;
    }
}
