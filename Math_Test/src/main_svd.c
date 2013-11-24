#include <../inc/stm32f4xx.h>
#include <../inc/stm32f4xx_gpio.h>
#include <../inc/stm32f4xx_rcc.h>
#include <../inc/stm32f4xx_adc.h>
#include <../inc/stm32f4xx_dac.h>
#include <../inc/stm32f4xx_dma.h>
#include <../inc/stm32f4xx_tim.h>
#include <../inc/arm_math.h>
#include <../inc/math_helper.h>
#include <../inc/stm32f4_discovery.h>
#include <../inc/stm32f4xx_conf.h>
#include <math.h>

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
#define DAC_DHR12R2_ADDRESS 0x40007414

#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

int result = -1;
float w[4];
float v[4][4];
float d[4];
float e[4];

float B_data[4][4] =
{
	{1.0,	32.0, 	3.0,	128.0	},
	{1.0, 	32.0, 	64.0, 	2048.0	},
	{1.0, 	16.0,	4.0, 	64.0	},
	{1.0, 	16.0, 	64.0, 	1024.0	}
};
int status_bit = 0;

double pythag(double a, double b);
int dsvd(float **a, int m, int n, float *w, float **v);
int hshld(float a[4][4], int n, float d[4], float e[4]);
int accum_rl(float **a, int m, int n, float *w, float **v);
int diag_bd(float **a, int m, int n, float *w, float **v);

//int hshld(arm_matrix_instance_f32 **A, int m, int n);

double pythag(double a, double b)
{
    double at = (double)fabs(a), bt = (double)fabs(b), ct, result;
    if (at > bt)       { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
    else if (bt > 0.0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
    else result = 0.0;
    return result;
}

int covariance_fast(float array[4][4], int m, int n)
{
	int i, j, k;
	float output[n][n];
	k = 0;
		for(i=0; i<n; i++) //initialize output
	{
		for(j=0; j<m; j++)
		{
			output[i][j] = 0.0;
		}
	}
	for(k=0; k<n; k++)
	{
		for(i=0; i<n; i++)
		{
			for(j=i; j<m; j++)
			{
				output[j][i] = output[i][j] += (array[i][k] * array[j][k]);
			}
		}
	}
	for(i=0; i<n; i++)
	{
		for(j=i; j<n; j++)
		{
			array[i][j] = array[j][i] = output[i][j];
		}
	}
	return 0;
}

int hshld(float a[4][4], int n, float d[4], float e[4])
{
	int l, k, j, i;
	float scale, hh, h, g, f;
	for(i=n-1;i>=1;i--)
	{
		l=i;
		h=scale=0.0;
		if(l>1)
		{
			for(k=0;k<l;k++)
			{
				scale += fabs(a[i][k]);
			}
			//arm_abs_f32(&(a[i]), &(a[i]), (uint32_t)l);

			if(scale==0.0)
			{
				//skip transformation
				e[i]=a[i][l];
			}
			else
			{
				for(k=0;k<l;k++)
				{
					a[i][k] /= scale;	//use scaled a's for transformation
					h += a[i][k] * a[i][k];	//form sigma in h
				}
				f = a[i][l];
				g = (f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i] = scale*g;
				h-=f*g;	//now h = 1/2 abs(u)^2
				a[i][l] = f-g; //store u in ith row of a
				f=0.0;
				for(j=0; j<l; j++)
				{
					//find eigenvectors in the next line
					a[j][i]=a[i][j]/h; //store u/h in ith column of a
					g=0.0; //form an element of A *. u in g
					for (k=0; k<j; k++)
					{
						g+= a[j][k]*a[i][k];
					}
					for (k=j; k<l; k++)
					{
						g+= a[k][j]*a[i][k];
					}
					e[j] = g/h; //form element of p in temporarily unused element of e
					f+= e[j]*a[i][j];
				}
				hh=f/(h+h); //form k = (ut *. p) / 2h
				for(j=0; j<l; j++) //form q and store in e overwriting p
				{
					f=a[i][j];
					e[j]=g=e[j]-hh*f;
					for(k=0;k<j;k++)
					{	//reduce a' = a - q *. ut - u *. qt
						a[j][k] -= (f*e[k]+ g*a[i][k]);
					}
				}
			}
		}else
		{
			e[i] = a[i][l];
		}
		d[i] = h;
	}
		//find eigenvectors next statement
	d[0]=0.0;
	e[0]=0.0;
	//find eigenvectors
	for (i=0; i<n; i++)
	{
		l=i; //begin accumulation of transformation matrices
		if(d[i])
		{
			for(j=0; j<l; j++)
			{
				g = 0.0;
				for(k=0; k<l; k++) //use u and u / h , stored in a, to form p *. q
				{
					g += a[i][k] * a[k][j];
				}
				for(k=0; k<l; k++)
				{
					a[k][j] -= g*a[k][i];
				}
			}

		}
		d[i]=a[i][i]; //eigenvalues
		a[i][i] = 1.0; //reset row and column of a to identity for next iteration
		for(j=0; j<l; j++) a[j][i] = a[i][j] = 0.0;
	}
	return 0;
}

//timer 2
void init_TIM2(void)
{
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
		//GPIOD->ODR ^= (1 << 13);	//toggle D13
//		result = hshld(B_data, 4, 4, w, v);
		GPIOD->ODR ^= (1 << 13);	//toggle D13
	}
	TIM2->SR = 0x0; //reset TIM2's status register
}

//timer 3
void init_TIM3(void)
{
	NVIC->ISER[0] |= 1<<(TIM3_IRQn); //Interrupt set-enable register, enable TIM2 IRQ
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable TIM2 clock
	TIM3->PSC = 0xFF; //prescaler
	TIM3->DIER |= TIM_DIER_UIE; //dma / interrupt enable register -> update interrupt enable
	TIM3->ARR = 0xFFFF; //auto reload register
	TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; //auto reload preload and counter enable
	TIM3->EGR = 1; //tells TIM2 there's been an update
}
void TIM3_IRQHandler(void) //blinker
{
	//if status register & update interrupt flag are both enabled,
	if (TIM3->SR & TIM_SR_UIF)
	{
		//GPIOD->ODR ^= (1 << 13);	//toggle D13
//		dsvd(B_data, 4, 4, w, v);
		result = covariance_fast(v, 4, 4);
		result = hshld(v, 4, d, e);
		GPIOD->ODR ^= (1 << 12);	//toggle D12
		WWDG_ClearFlag();
	}
	TIM3->SR = 0x0; //reset TIM2's status register
}
/*
void WWDG_IRQHandler(void)
{
  if (WWDG_GetFlagStatus())
  {
    WWDG_SetCounter(0x7f);
    WWDG_ClearFlag();
	GPIOD->ODR ^= (1 << 12);	//toggle D12
  }
}

void UsageFault_Handler(void)
{
	GPIOD->ODR ^= (1 << 14);	//toggle D12
}
*/



void init_GPIOD(void)
{
	GPIO_InitTypeDef      GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	//GPIOD init (indicators)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // we want to configure all LED GPIO pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 	// this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
    GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
}


int main(void)
{
	WWDG_SetPrescaler(WWDG_Prescaler_8);
	/*
	long period = 0;
	float A_data[16]=
	{
		1.0, 		32.0, 		4.0, 		128.0,
		1.0, 		32.0, 		64.0, 		2048.0,
		1.0, 		16.0, 		4.0, 		64.0,
		1.0, 		16.0, 		64.0, 		1024.0,
	};
	*/
	//arm_matrix_instance_f32 A;
	//arm_mat_init_f32(&A, 4, 4, A_data);
	//dsvd(B_data, 4, 4, w, v);
	init_GPIOD();
	init_TIM2();

//	init_TIM3();
	result = covariance_fast(v, 4, 4);
	result = hshld(v, 4, d, e);
    while (1)
    {
    };
    return result;
}
