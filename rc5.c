/*
 * rc5.c
 *
 *  Created on: 23 lis 2016
 *      Author: szostakm
 */
#include "rc5.h"

const char* RC5ButtonNames[] = {
	[POWER] = "POWER",
	[SOURCE] = "SOURCE",
	[STEREO] = "STEREO",
	[TXT] = "TXT",
	[ACTIVE_CONTROL] = "ACTIVE CONTROL",
	[DEMO] = "DEMO",
	[RED] = "RED",
	[GREEN] = "GREEN",
	[YELLOW] = "YELLOW",
	[BLUE] = "BLUE",
	[SURROUND] = "SURROUND",
	[FORMAT] = "FORMAT",
	[MENU] = "MENU",
	[MODE] = "MODE",
	[UP] = "UP",
	[RIGHT] = "RIGHT",
	[DOWN] = "DOWN",
	[LEFT] = "LEFT",
	[OK] = "OK",
	[VOL_UP] = "VOLUME UP",
	[VOL_DOWN] = "VOLUME DOWN",
	[CH_UP] = "CHANNEL UP",
	[CH_DOWN] = "CHANNEL DOWN",
	[MUTE] = "MUTE",
	[AV] = "AV",
	[STATUS] = "STATUS",
	[0x0] = "#0",
	[0x1] = "#1",
	[0x2] = "#2",
	[0x3] = "#3",
	[0x4] = "#4",
	[0x5] = "#5",
	[0x6] = "#6",
	[0x7] = "#7",
	[0x8] = "#8",
	[0x9] = "#9"

};

volatile uint16_t rc_buffor = 0xFFFF;

volatile int cooldown_cnt = -1;
volatile uint16_t duration = 1778;
volatile uint16_t raw_duration;
volatile uint8_t auto_duration = 1;
volatile uint8_t amount_of_bits_to_gather=0;


void SysTick_Handler(){
	if(cooldown_cnt > 0) cooldown_cnt--;
}

volatile enum State state = STANDBY;

void RC5_TIMER_IRQ_HANDLER(){
	static uint8_t cnt=0;
	if (TIM_GetITStatus(RC5_TIMER, TIM_IT_Update) == SET){
		TIM_ClearITPendingBit(RC5_TIMER, TIM_IT_Update);
	}
	if (TIM_GetITStatus(RC5_TIMER, TIM_IT_CC1) == SET){
		if(state == START){
			rc_buffor <<= 1;
			rc_buffor |= GPIO_ReadInputDataBit(IR_RECIVER_GPIO_PORT, IR_RECIVER_GPIO_PIN);
			++cnt;
			state = IN_PROGRESS;
			TIM_SetCounter(RC5_TIMER, 0);
		}
		TIM_ClearITPendingBit(RC5_TIMER, TIM_IT_CC1);
	}
	if (TIM_GetITStatus(RC5_TIMER, TIM_IT_CC2) == SET){
		if(state == IN_PROGRESS){
			if(cnt < amount_of_bits_to_gather){
				rc_buffor <<= 1;
				rc_buffor |= GPIO_ReadInputDataBit(IR_RECIVER_GPIO_PORT, IR_RECIVER_GPIO_PIN);
				++cnt;
			}else{
				state = READY;
				cnt=0;
				NVIC_EnableIRQ(IR_RECIVER_EXTI_IRQ);
				TIM_Cmd(RC5_TIMER, DISABLE);
			}
			TIM_SetCounter(RC5_TIMER, 0);
		}
		TIM_ClearITPendingBit(RC5_TIMER, TIM_IT_CC2);
	}
}

void IR_RECIVER_EXTI_IRQ_HANDLER(){
	if (EXTI_GetITStatus(IR_RECIVER_EXTI_LINE)) {
		EXTI_ClearITPendingBit(IR_RECIVER_EXTI_LINE);
		if(state == STANDBY){
			rc_buffor |= 1;
			if(auto_duration == 1){
				TIM_SetCounter(RC5_TIMER, 0);
				TIM_Cmd(RC5_TIMER, ENABLE);
				state = MEASURE;
			}else{
				amount_of_bits_to_gather = 14;
				state = START;
				TIM_SetCompare1(RC5_TIMER, duration/4*3);
			}
		}else if(state == MEASURE){
			TIM_Cmd(RC5_TIMER, DISABLE);
			raw_duration = TIM_GetCounter(RC5_TIMER)*RATIO;
			if(raw_duration >= F1_LOW_VALUE && raw_duration <= F1_HIGH_VALUE){//1
				duration = raw_duration;
				amount_of_bits_to_gather = 13;
				rc_buffor <<= 1;
				rc_buffor |= 1;
				state = START;
				TIM_SetCompare1(RC5_TIMER, duration/4*3);
			}else if(raw_duration >= F0_C0_LOW_VALUE && raw_duration <= F0_C0_HIGH_VALUE){//3
				duration = raw_duration/3*2-24;
				amount_of_bits_to_gather = 12;
				rc_buffor <<= 2;
				state = START;
				TIM_SetCompare1(RC5_TIMER, duration/4*5);
			}else if(raw_duration >= F0_C1_LOW_VALUE && raw_duration <= F0_C1_HIGH_VALUE){//2
				duration = raw_duration/2;
				amount_of_bits_to_gather = 12;
				rc_buffor <<= 2;
				rc_buffor |= 1;
				state = START;
				TIM_SetCompare1(RC5_TIMER, duration/4*3);
			}else{
				state = STANDBY;
			}
		}
		if(state == START){
			TIM_SetCompare2(RC5_TIMER, duration);
			TIM_SetCounter(RC5_TIMER, 0);
			TIM_Cmd(RC5_TIMER, ENABLE);
			NVIC_DisableIRQ(IR_RECIVER_EXTI_IRQ);
		}

	}
}

void RC5TimerInit(){
	TIM_TimeBaseInitTypeDef tim;
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Prescaler = 64 - 1;
	tim.TIM_Period = MAX_DURATION - 1;
	TIM_TimeBaseInit(RC5_TIMER, &tim);
	TIM_ITConfig(RC5_TIMER, TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2, ENABLE);
}

void RC5Init(){
	EXTI_InitTypeDef exti;
	NVIC_InitTypeDef nvic;
	GPIO_InitTypeDef gpio;

	RCC_APB1PeriphClockCmd(RCC_TIMER, ENABLE);

	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = IR_RECIVER_GPIO_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IR_RECIVER_GPIO_PORT, &gpio);

	RC5TimerInit();

	EXTI_StructInit(&exti);
	exti.EXTI_Line = IR_RECIVER_EXTI_LINE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);
	GPIO_EXTILineConfig(IR_RECIVER_GPIO_SOURCE_PORT, IR_RECIVER_GPIO_SOURCE_PIN);

	nvic.NVIC_IRQChannel = IRQ_TIMER;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	nvic.NVIC_IRQChannel = IR_RECIVER_EXTI_IRQ;
	NVIC_Init(&nvic);


	SysTick_Config(SystemCoreClock / 1000);
}

int getNumber(struct RC5Struct* s, uint8_t digits){
	int number=0;
	uint8_t cnt = 0;
	while(cnt!=digits){
		while(getRC5Signal(s, CONTROL)!=1);
		if(s->data_bits>=0 && s->data_bits<=9){
			number*=10;
			number+=s->data_bits;
			cnt++;
		}

	}
	return number;
}

int getState(){
	return state;
}

uint8_t getRC5Signal(struct RC5Struct* s, enum RC5Mode mode){
	static volatile uint8_t first_press = 0;
	static volatile uint8_t last_control_bit = 0;
	static volatile uint8_t last_command = 0;

	if(state == READY){
		if((rc_buffor & 0x1) == 0x0){
			state = STANDBY;
			return 0;
		}
		rc_buffor>>=1;
		state = STANDBY;
		s->raw = rc_buffor&0x3FFF;
		s->philips_bit = (rc_buffor>>12)&0x1;
		s->control_bit = (rc_buffor>>11)&0x1;
		s->address_bits = (rc_buffor>>6)&0x1F;
		s->data_bits = (rc_buffor&0x3F)+((s->philips_bit==0x0)?0x40:0x0);
		s->duration = duration;

		if(mode >= CONTROL && last_control_bit == s->control_bit && last_command == s->data_bits && first_press==1){
			if(mode == COOLDOWN){
				if(cooldown_cnt==-1){
					cooldown_cnt = COOLDOWN_TIME;
				}else if(cooldown_cnt==0){
					cooldown_cnt = COOLDOWN_DELAY;
					return 1;
				}
			}
			return 0;
		}else cooldown_cnt=-1;
		first_press = 1;
		last_control_bit = s->control_bit;
		last_command = s->data_bits;

		return 1;
	}

	return 0;
}

const char* RC5ToBinaryString(uint16_t raw){
	static char b[15];
	b[0] = '\0';
	uint16_t i;
	for(i = 8192; i > 0; i>>=1){
		strcat(b, ((raw & i) == i)? "1" : "0");
	}
	return b;
}

const char* RC5toString(struct RC5Struct s){
	static char buffer[256];
	sprintf(buffer, "Raw: 0x%X (0b%s) Field: %d Control: %d, Button: %s (0x%X) Duration: %d.", s.raw, RC5ToBinaryString(s.raw), s.philips_bit, s.control_bit, RC5ButtonNames[s.data_bits], s.data_bits, s.duration);
	return buffer;
}

void RC5EnableAutoFrequency(){
	auto_duration = 1;
}
void RC5DisableAutoFrequency(){
	auto_duration = 0;
}
void RC5SetFrequency(uint16_t f){
	RC5DisableAutoFrequency();
	duration = f;
}
