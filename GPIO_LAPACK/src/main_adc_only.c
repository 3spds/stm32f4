#include <../inc/stm32f4xx.h>
#include <../inc/stm32f4xx_gpio.h>
#include <../inc/stm32f4xx_rcc.h>
#include <../inc/stm32f4xx_adc.h>
#include <../inc/stm32f4xx_dac.h>
#include <../inc/stm32f4xx_dma.h>
#include <../inc/stm32f4_discovery.h>
#include <stdio.h>

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
/* You can monitor the converted value by adding the variable "ADC3ConvertedValue"
   to the debugger watch window */
__IO uint16_t ADC3ConvertedValue = 0;
__IO uint32_t ADC3ConvertedVoltage = 0;
__IO uint32_t i = 0;

void Delay(__IO uint32_t nCount)
{
    while(nCount--)
    {
    }
}

void Blink(__IO uint32_t nCount)
{
    GPIOD->BSRRL = 0xF000; // set PD12 thru PD15
    Delay(nCount);		 // wait a short period of time
    GPIOD->BSRRH = 0xF000; // reset PD12 thru PD15
    Delay(nCount);
}

/**
  * @brief  ADC3 channel12 with DMA configuration
  * @param  None
  * @retval None
  */
/* This funcion shows how to initialize
 * the GPIO pins on GPIOD and how to configure
 * them as inputs and outputs
 */
void init_GPIO(void)
{

    /* This TypeDef is a structure defined in the
     * ST's library and it contains all the properties
     * the corresponding peripheral has, such as output mode,
     * pullup / pulldown resistors etc.
     *
     * These structures are defined for every peripheral so
     * every peripheral has it's own TypeDef. The good news is
     * they always work the same so once you've got a hang
     * of it you can initialize any peripheral.
     *
     * The properties of the periperals can be found in the corresponding
     * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
     */
    GPIO_InitTypeDef      GPIO_InitStruct;

    /* This enables the peripheral clock to the GPIOD IO module
     * Every peripheral's clock has to be enabled
     *
     * The STM32F4 Discovery's User Manual and the STM32F407VGT6's
     * datasheet contain the information which peripheral clock has to be used.
     *
     * It is also mentioned at the beginning of the peripheral library's
     * source file, e.g. stm32f4xx_gpio.c
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* In this block of instructions all the properties
     * of the peripheral, the GPIO port in this case,
     * are filled with actual information and then
     * given to the Init function which takes care of
     * the low level stuff (setting the correct bits in the
     * peripheral's control register)
     *
     *
     * The LEDs on the STM324F Discovery are connected to the
     * pins PD12 thru PD15
     */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; // we want to configure all LED GPIO pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
    GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
    /* This enables the peripheral clock to
     * the GPIOA IO module
     */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);


    /* Here the GPIOA module is initialized.
     * We want to use PA0 as an input because
     * the USER button on the board is connected
     * between this pin and VCC.
     */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;		  // we want to configure PA0
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
    GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff


    /* Configure ADC3 Channel12 pin as analog input ******************************/
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; //set GPIO clock speed to 100MHz
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/*-- Setup Peripherals: DMA GPIO ADC
    DMA - Channel 2,
*/
void init_DMA_GPIO_ADC(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef       DMA_InitStructure;

    /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    /* DMA2 Stream0 channel0 configuration **************************************/
    DMA_InitStructure.DMA_Channel = DMA_Channel_2;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC3_DR_ADDRESS;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
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
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    /* GPIO Init **********************************************************/
    init_GPIO();   //see above definition

    /* ADC Common Init **********************************************************/
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC3 Init ****************************************************************/
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC3, &ADC_InitStructure);

    /* ADC3 regular channel12 configuration *************************************/
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);

    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

    /* Enable ADC3 DMA */
    ADC_DMACmd(ADC3, ENABLE);

    /* Enable ADC3 */
    ADC_Cmd(ADC3, ENABLE);
}



int main(void)
{
    //initializer function...
    /* ADC3 configuration *******************************************************/
    /*  - Enable peripheral clocks                                              */
    /*  - DMA2_Stream0 channel2 configuration                                   */
    /*  - Configure ADC Channel12 pin as analog input                           */
    /*  - Configure ADC3 Channel12                                              */
    init_DMA_GPIO_ADC();
    /* Start ADC3 Software Conversion */
    ADC_SoftwareStartConv(ADC3);

    /* This flashed the LEDs on the board once
     * Two registers are used to set the pins (pin level is VCC)
     * or to reset the pins (pin level is GND)
     *
     * BSRR stands for bit set/reset register
     * it is seperated into a high and a low word (each of 16 bit size)
     *
     * A logical 1 in BSRRL will set the pin and a logical 1 in BSRRH will
     * reset the pin. A logical 0 in either register has no effect
     */
    GPIOD->BSRRL = 0xF000; // set PD12 thru PD15
    Delay(1000000L);		 // wait a short period of time
    GPIOD->BSRRH = 0xF000; // reset PD12 thru PD15

    // this counter is used to count the number of button presses
 //   uint8_t i = 0;

    while (1)
    {

        /* Every GPIO port has an input and
         * output data register, ODR and IDR
         * respectively, which hold the status of the pin
         *
         * Here the IDR of GPIOA is checked whether bit 0 is
         * set or not. If it's set the button is pressed
         */
        /*
        		if(GPIOA->IDR & 0x0001){
        // if the number of button presses is greater than 4, reset the counter (we start counting from 0!)

        if(i > 3){
        	i = 0;
        }
        else{ // if it's smaller than 4, switch the LEDs

        	switch(i){

        		case 0:
        			GPIOD->BSRRL = 0x1000; // this sets LED1 (green)
        			GPIOD->BSRRH = 0x8000; // this resets LED4 (blue)
        			break;

        		case 1:
        			GPIOD->BSRRL = 0x2000; // this sets LED2 (orange)
        			GPIOD->BSRRH = 0x1000; // this resets LED1
        			break;

        		case 2:
        			GPIOD->BSRRL = 0x4000; // this sets LED3 (red)
        			GPIOD->BSRRH = 0x2000; // this resets LED2
        			break;

        		case 3:
        			GPIOD->BSRRL = 0x8000; // this sets LED4
        			GPIOD->BSRRH = 0x4000; // this resets LED3
        			break;
        		}

        	i++; // increase the counter every time the switch is pressed
        }
        Delay(3000000L); // add a small delay to debounce the switch
        		}*/
        /* convert the ADC value (from 0 to 0xFFF) to a voltage value (from 0V to 3.3V)*/
        ADC3ConvertedVoltage = ADC3ConvertedValue *3300/0xFFF;

        if(GPIOA->IDR & 0x0001)
        {
            Blink(3000000L);
            i++;
        }
        ;
    }
}




