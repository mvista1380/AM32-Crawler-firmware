/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

#include "targets.h"
#include "IO.h"
#include "serial_telemetry.h"
#include "functions.h"

char ic_timer_prescaler = 1;
char output_timer_prescaler;
//int servorawinput;
int buffersize = 32;
int smallestnumber = 20000;
uint32_t dma_buffer[64];
char out_put = 0;
char buffer_divider = 44;

void changeToOutput(){

	LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

#ifdef USE_TIMER_2_CHANNEL_3

	  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2);           // de-init timer 2
	  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
	  IC_TIMER_REGISTER->CCMR2 = 0x60;
	  IC_TIMER_REGISTER->CCER = 0x200;
#endif

#ifdef USE_TIMER_3_CHANNEL_1
	  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);           // de-init timer 2
	  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
	  IC_TIMER_REGISTER->CCMR1 = 0x60;
	  IC_TIMER_REGISTER->CCER = 0x3;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
	  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15);
	  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
	  LL_TIM_OC_SetIdleState(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_HIGH);
	  IC_TIMER_REGISTER->CCMR1 = 0x60;                         // channel 1 tim 15
	  IC_TIMER_REGISTER->CCER = 0x3;
	//  LL_TIM_OC_SetPolarity(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_LOW);

#endif

	  IC_TIMER_REGISTER->PSC = output_timer_prescaler;
	  IC_TIMER_REGISTER->ARR = 80;
	  out_put = 1;
	  LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
}

void changeToInput(){
	  LL_DMA_SetDataTransferDirection(DMA1, INPUT_DMA_CHANNEL, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

#ifdef USE_TIMER_2_CHANNEL_3
	  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2);           // de-init timer 2
	  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);
	  IC_TIMER_REGISTER->CCMR2 = 0x1;
	  IC_TIMER_REGISTER->CCER = 0xa00;
#endif

#ifdef USE_TIMER_3_CHANNEL_1
	  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM3);           // de-init timer 2
	  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM3);
	  IC_TIMER_REGISTER->CCMR1 = 0x1;
	  IC_TIMER_REGISTER->CCER = 0xa;
#endif
#ifdef USE_TIMER_15_CHANNEL_1
	  LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_TIM15);
	  LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_TIM15);
	  IC_TIMER_REGISTER->CCMR1 = 0x1;
	  IC_TIMER_REGISTER->CCER = 0xa;
#endif
	  IC_TIMER_REGISTER->PSC = ic_timer_prescaler;
	  IC_TIMER_REGISTER->ARR = 0xFFFF;
	  LL_TIM_GenerateEvent_UPDATE(IC_TIMER_REGISTER);
	  out_put = 0;

}

void detectInput(){
	smallestnumber = 20000;
	servoPwm = 0;
	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 31; j++){

		if((dma_buffer[j] - lastnumber) < smallestnumber){ // blank space
			smallestnumber = dma_buffer[j] - lastnumber;

		}
		lastnumber = dma_buffer[j];
	}

	if ((smallestnumber > 3)&&(smallestnumber < 32)){
		ic_timer_prescaler= 0;
		output_timer_prescaler=0;
		buffer_divider = 65;
		armed_count_threshold = 10000;
		buffersize = 32;
	}
	if ((smallestnumber > 32 )&&(smallestnumber < 80)){
		ic_timer_prescaler=1;
		output_timer_prescaler=1;
		IC_TIMER_REGISTER->CNT = 0xffff;
		buffer_divider = 65;
		armed_count_threshold = 10000;
		buffersize = 32;
	}
	if (smallestnumber > 300 ){
		servoPwm = 1;
		ic_timer_prescaler=63;
		armed_count_threshold = 100;
		buffersize = 4;
	}

	if (smallestnumber == 20000 ){
		inputSet = 0;
	}else{

		inputSet = 1;
	}

}




















