#include "stm32l552xx.h"

void setClks(void);
void initZTimer(void);
void initXTimer(void);
void initYTimer(void);
void initDMA(void);
void initADC(void);

#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &  1) //Checks the bit number <idx> -- 0 means clear; !0 means set.


volatile uint32_t zStepPeriod_us = 0, xStepPeriod_us = 0, yStepPeriod_us = 0; // Initial step period in microseconds for a high signal
volatile uint32_t zNoStepPeriod = 10000, xNoStepPeriod = 10000, yNoStepPeriod = 10000; // Initial step pediod in microseconds for a low signal
volatile float zThrottlePosition = 0.5f, xThrottlePosition = 0.5f, yThrottlePosition = 0.5f; // Intial value for throttle position read by ADC


// Define memory buffer size
#define BUFFER_SIZE 3
volatile uint16_t adc_buffer[BUFFER_SIZE]; // Create a buffer to store ADC results in. Each index is a channel


int main() {
	initDMA();
	initADC();

	//AFTER DMA INIT ENABLE ADC
	ADC1->CR |= 1; // Enable ADC
	// Wait until ADC is Ready (ADRDY)
	while(bitcheck(ADC1->ISR, 0)==0);
	bitset(ADC1->CR, 2); // Start Conversion (won't actally do the conversion! It will wait for external trigger instead)
	TIM1->CR1 = 1; // Enable timer 1

	setClks();
    initZTimer();
    initXTimer();
    initYTimer();


    while (1) {

    	if (zThrottlePosition > 0.7){
    		zStepPeriod_us = 1000;
    		bitset(GPIOE->ODR, 6);
    		zNoStepPeriod = 5000 - (4000 * zThrottlePosition);
    	} else if (zThrottlePosition < 0.3){
    		zStepPeriod_us = 1000;
    		bitclear(GPIOE->ODR, 6);
    		zNoStepPeriod = 5000 - (4000 * ( 0.5 + (0.5 - zThrottlePosition)));
    	} else {
    		zStepPeriod_us = 0;
    	}

    	if (xThrottlePosition > 0.7){
    		xStepPeriod_us = 1000;
    		bitset(GPIOE->ODR, 11);
    		xNoStepPeriod = 5000 - (4000 * xThrottlePosition);
    	} else if (xThrottlePosition < 0.3){
    		xStepPeriod_us = 1000;
    		bitclear(GPIOE->ODR, 11);
    		xNoStepPeriod = 5000 - (4000 * ( 0.5 + (0.5 - xThrottlePosition)));
    	} else {
    		xStepPeriod_us = 0;
    	}

    	if (yThrottlePosition > 0.7){
    		yStepPeriod_us = 1000;
    		bitset(GPIOA->ODR, 8);
    		yNoStepPeriod = 5000 - (4000 * yThrottlePosition);
    	} else if (yThrottlePosition < 0.3){
    		yStepPeriod_us = 1000;
    		bitclear(GPIOA->ODR, 8);
    		yNoStepPeriod = 5000 - (4000 * ( 0.5 + (0.5 - yThrottlePosition)));
    	} else {
    		yStepPeriod_us = 0;
    	}
    }
}


void TIM3_IRQHandler(void) {
    if (TIM3->SR & 1) { // Check if update interrupt flag is set
        TIM3->SR &= ~1; // Clear update interrupt flag
        if (zStepPeriod_us != 0) {
            bitflip(GPIOE->ODR, 3); // Toggle step signal (PE3)
            if (!bitcheck(GPIOE->ODR, 3)) { // High signal
                TIM3->ARR = zStepPeriod_us - 1; // Set auto-reload value for high signal
            } else { // Low signal
                TIM3->ARR = zNoStepPeriod - 1; // Set auto-reload value for low signal
            }
        }
    }
}

void TIM4_IRQHandler(void) {
    if (TIM4->SR & 1) { // Check if update interrupt flag is set
        TIM4->SR &= ~1; // Clear update interrupt flag
        if (xStepPeriod_us != 0) {
        	bitflip(GPIOE->ODR, 9); // Toggle step signal (PE9)
            if (!bitcheck(GPIOE->ODR, 9)) { // High signal
                TIM4->ARR = xStepPeriod_us - 1; // Set auto-reload value for high signal
            } else { // Low signal
                TIM4->ARR = xNoStepPeriod - 1; // Set auto-reload value for low signal
            }
        }
    }
}

void TIM7_IRQHandler(void) {
    if (TIM7->SR & 1) { // Check if update interrupt flag is set
        TIM7->SR &= ~1; // Clear update interrupt flag
        if (yStepPeriod_us != 0) {
        	bitflip(GPIOA->ODR, 0);
            if (!bitcheck(GPIOA->ODR, 0)) { // High signal
                TIM7->ARR = yStepPeriod_us - 1; // Set auto-reload value for high signal
            } else { // Low signal
                TIM7->ARR = yNoStepPeriod - 1; // Set auto-reload value for low signal
            }
        }
    }
}

void initZTimer(void) {
	bitset(RCC->APB1ENR1, 1);    // Enable TIM3 clock

	// Configure TIM3
    TIM3->PSC = 16 - 1;          // Divided 16MHz source clk by 16, for 1us tick
    TIM3->ARR = zStepPeriod_us - 1; // Initial auto-reload value for high signal
    TIM3->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    NVIC_EnableIRQ(TIM3_IRQn);   // Enable TIM3 IRQ in NVIC
    TIM3->CR1 |= 1;    // Start TIM3
}

void initXTimer(void) {
	bitset(RCC->APB1ENR1, 2);    // Enable TIM5 clock

    // Configure TIM4
    TIM4->PSC = 16 - 1;          // Divided 16MHz source clk by 16, for 1us tick
    TIM4->ARR = xStepPeriod_us - 1; // Initial auto-reload value for high signal
    TIM4->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    NVIC_EnableIRQ(TIM4_IRQn);   // Enable TIM4 IRQ in NVIC
    TIM4->CR1 |= 1;    // Start TIM4
}


void initYTimer(void) {
	bitset(RCC->APB1ENR1, 5);    // Enable TIM7 clock

    // Configure TIM2
    TIM7->PSC = 16 - 1;          // Divided 16MHz source clk by 16, for 1us tick
    TIM7->ARR = yStepPeriod_us - 1; // Initial auto-reload value for high signal
    TIM7->DIER |= TIM_DIER_UIE;  // Enable update interrupt
    NVIC_EnableIRQ(TIM7_IRQn);   // Enable TIM7 IRQ in NVIC
    TIM7->CR1 |= 1;    // Start TIM7
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) { // Check if transfer complete flag is set
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Clear transfer complete flag

        // Process ADC conversion results from the buffer
        zThrottlePosition = (adc_buffer[0]*(3.3/4096))/3.3; // 12-bit ADC resolution (4096 levels)
        xThrottlePosition = (adc_buffer[1]*(3.3/4096))/3.3; // 12-bit ADC resolution (4096 levels)
        yThrottlePosition = (adc_buffer[2]*(3.3/4096))/3.3; // 12-bit ADC resolution (4096 levels)

        // Restart ADC conversion
        ADC1->CR |= ADC_CR_ADSTART;
    }
}

void initADC() {
    bitset(RCC->APB2ENR, 11); // enable TIM1 clock
    TIM1->PSC = 16000 - 1; // Divided 16MHz source clk by 16000, for 1ms tick
    TIM1->ARR = 5 - 1; // Count 100ms period times
    TIM1->CCMR1 = 0x30; // Set output to toggle on match (Output Compare Mode)
    TIM1->CCR1 = 1; // Output will toggle when CNT==CCR1
    bitset(TIM1->BDTR, 15); // Main output enable
    TIM1->CCER |= 1; // Enable CH1 compare mode
    TIM1->CNT = 0; // Clear counter

    // Configure GPIOC pins (z axis pot)
    bitset(RCC->AHB2ENR, 2);
    bitset(GPIOC->MODER, 0); // Setup PC0 to 0b11 (Analog input)
    bitset(GPIOC->MODER, 1);

    // Configure GPIOC pins (x axis pot)
    bitset(GPIOC->MODER, 2); // Setup PC1 to 0b11 (Analog input)
    bitset(GPIOC->MODER, 3);

    // Configure GPIOC pins (y axis pot)
    bitset(GPIOC->MODER, 4); // Setup PC2 to 0b11 (Analog input)
    bitset(GPIOC->MODER, 5);



    // Enable ADC Clock
    bitset(RCC->AHB2ENR, 13); // Enable ADC clock
    RCC->CCIPR1 |= 0x3<<28; // Route SYSCLK (HCLK) to ADC

    // Turn on ADC Voltage Regulator
    bitclear(ADC1->CR, 29); // Get out of deep power down mode
    bitset(ADC1->CR, 28);

    // External Trigger Enable and Polarity; EXTEN= 0b01 for rising edge
    bitset(ADC1->CFGR, 10);
    bitclear(ADC1->CFGR, 11);


    //External trigger (EXT0) is connected to TIM1_CH1; EXTSEL=0000
    bitclear(ADC1->CFGR, 6); // 0b0000
    bitclear(ADC1->CFGR, 7);
    bitclear(ADC1->CFGR, 8);
    bitclear(ADC1->CFGR, 9);


    // Set up ADC to use DMA
    bitset(ADC1->CFGR, 0); // Bit0 : DMA EN
    bitset(ADC1->CFGR, 13); // Bit13 : CONTINUOUS MODE

    // Buy a little bit of time for ADC
    for (int i = 0; i < 10000; i++) {}

    bitset(ADC1->SQR1, 1); // Three conversions in the sequence

    bitset(ADC1->SQR1, 6); // Channel 0 (PC0) first in the sequence (1)

    bitclear(ADC1->SQR1, 12); // Channel 1 (PC1) second in the sequence (10)
    bitset(ADC1->SQR1, 13); // Channel 1 (PC1) second in the sequence (10)

    bitset(ADC1->SQR1, 18); // Channel 1 (PC2) second in the sequence (11)
    bitset(ADC1->SQR1, 19); // Channel 1 (PC2) second in the sequence (11)

}



void initDMA(void) {

	// Enable DMA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN_Msk; // Enable the DMA controller clock
    DMA1->IFCR = 0xF; // Clear any interrupt flags of channel 1 just in case

    // Enable DMA interrupt
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// Setup for DMA1
	DMA1_Channel1->CCR  &= ~1;           /* disable DMA1 channel 1 */
    while (DMA1_Channel1->CCR & 1) {}    /* wait until DMA1 channel 1 is disabled */


    // Configure DMA for ADC (Channel 1)
    DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR)); // ADC data register address
    DMA1_Channel1->CM0AR = (uint32_t)adc_buffer; // Memory buffer address
    DMA1_Channel1->CNDTR = BUFFER_SIZE; // Number of data to transfer

    DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | (1<<8) | DMA_CCR_TCIE | DMA_CCR_CIRC; // Configure DMA channel control register
    // MINC=1byte (mem addr increment), PSIZE=01 (16 bit perif source size), MSIZE=01 (2 byte size of memory data (int16)


    // Setup for DMAMUX1
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    DMAMUX1_Channel0->CCR = 5;     // To route the ADC1 to the DMA1_Channel 1, you need to program the mux to 5 per Page 512

    // Enable DMA channel
    DMA1_Channel1->CCR |= 1;
}


void setClks(void) {
    bitset(RCC->AHB2ENR, 4); // Enable GPIOE clock
    bitset(RCC->AHB2ENR, 0); // Enable GPIOA clock


    // Clear GPIO pin modes
    GPIOE->MODER &= ~((3 << 6) | (3 << 12) | (3 << 18) | (3 << 22)); // Clear mode bits for PE3, PE6, PE9, PE11
    GPIOA->MODER &= ~((3 << 0) | (3<<16)); // Clear mode bits for PA0, PA8

    // Set GPIO pin modes
    GPIOE->MODER |= (1 << 6) | (1 << 12) | (1 << 18) | (1 << 22); // Set PE3, PE6, PE9, PE11 as general purpose output mode
    GPIOA->MODER |= (1 << 0) | (1 << 16); // Set PA0, PA8 as general purpose output mode

}
