#ifndef __RC5_H
#define __RC5_H

#include "stm32f10x.h"

/** Timer RCC, which will be measuring and receiving bits. */
#define RCC_TIMER RCC_APB1Periph_TIM2

/** External interrupt line, where the IR receiver is connected. */
#define IR_RECIVER_EXTI_LINE EXTI_Line0

/** External interrupt IRQ handler, where the IR receiver is connected. */
#define IR_RECIVER_EXTI_IRQ_HANDLER EXTI0_IRQHandler

/** External interrupt port, where the IR receiver is connected. */
#define IR_RECIVER_GPIO_SOURCE_PORT GPIO_PortSourceGPIOC

/** External interrupt pin, where the IR receiver is connected. */
#define IR_RECIVER_GPIO_SOURCE_PIN GPIO_PinSource0

/** GPIO port, where the IR receiver is connected. */
#define IR_RECIVER_GPIO_PORT GPIOC

/** GPIO pin, where the IR receiver is connected. */
#define IR_RECIVER_GPIO_PIN GPIO_Pin_0

/** Timer IRQ channel. */
#define IRQ_TIMER TIM2_IRQn

/** External interrupt IRQ channel. */
#define IR_RECIVER_EXTI_IRQ EXTI0_IRQn

/** Timer, which will be measuring and receiving bits. */
#define RC5_TIMER TIM2

/** Timer IRQ handler, which will be measuring and receiving bits. */
#define RC5_TIMER_IRQ_HANDLER TIM2_IRQHandler

/** Maxiumum period of timer. */
#define MAX_DURATION 4000

/** Typical duration of single RC5 bit. 1/36kHz = 27.78us, 27.78us*64 = 1778us */
#define TYPICAL_BIT_DURATION_TIME 1778

/** Margin of error */
#define TOLERANCE_VALUE 100

/** Factor, which change duration time. Cheap remotes send a first bit in lower frequency then other. */
#define RATIO 1
//#define RATIO 0.978

/** Repeat rate. */
#define COOLDOWN_TIME 850

/** Repeat delay. */
#define COOLDOWN_DELAY 50

/** Lower limit of frequency in case when field bit is 1.  */
#define F1_LOW_VALUE TYPICAL_BIT_DURATION_TIME-TOLERANCE_VALUE

/** Upper limit of frequency in case when field bit is 1.  */
#define F1_HIGH_VALUE TYPICAL_BIT_DURATION_TIME+TOLERANCE_VALUE

/** Lower limit of frequency in case when field bit is 0 and control bit is 1.  */
#define F0_C1_LOW_VALUE (F1_LOW_VALUE)*2

/** Upper limit of frequency in case when field bit is 0 and control bit is 1.  */
#define F0_C1_HIGH_VALUE (F1_HIGH_VALUE)*2

/** Lower limit of frequency in case when field bit is 0 and control bit is 0.  */
#define F0_C0_LOW_VALUE (F1_LOW_VALUE)/2*3

/** Upper limit of frequency in case when field bit is 0 and control bit is 0.  */
#define F0_C0_HIGH_VALUE (F1_HIGH_VALUE)/2*3


/** \brief State of reciving frame 
 *
 * Enum, which describes phases of receiving RC5 frame.
 */
enum State{
	STANDBY, /**< EXTI waiting for the start bit.  */
	MEASURE, /**< The phase when AutoFrequency mode is enabled and a duraction of single bit is measuring by timer. */
	START, /**< Bits receiving starts. */
	IN_PROGRESS, /**< Receiving in progress. */
	READY, /**<  Frame is received and ready to interpret. */
	INCORRECT /**< Never used. */
};

/** RC5 Frame buffor */
extern volatile uint16_t rc_buffor;

/** Current phase of receiving RC5 frame */
extern volatile enum State state; 

/** Raw duration measured by timer */
extern volatile uint16_t raw_duration;

/** Duration of signle bit in current RC5 frame. */
extern volatile uint16_t duration;

/** Flag, which enabling AutoFrequency mode. */
extern volatile uint8_t auto_duration;

/** Number of bits to receive */
extern volatile uint8_t amount_of_bits_to_gather;

/** \brief Example button codes of TV remote. 
 *
 * Some of this command codes are common.
 */
enum RC5Buttons{
	POWER = 0xC,
	SOURCE = 0x38,
	STEREO = 0x23,
	TXT = 0x3C,
	ACTIVE_CONTROL = 0x61,
	DEMO = 0x3E,
	RED = 0x6B,
	GREEN = 0x6C,
	YELLOW = 0x6D,
	BLUE = 0x6E,
	SURROUND = 0x24,
	FORMAT = 0x7E,
	MENU = 0x52,
	MODE = 0xA,
	UP = 0x50,
	RIGHT = 0x56,
	DOWN = 0x51,
	LEFT = 0x55,
	OK = 0x57,
	VOL_UP = 0x10,
	VOL_DOWN = 0x11,
	CH_UP = 0x20
};

/** Array, which contains string values of buttons */
extern const char* RC5ButtonNames[];

/** \brief Modes of receiving button signal from IR remote. 
 *
 * Mode indicates how function #getRC5Signal interprets repeating signal from IR remote. 
 */
enum RC5Mode{
	NONE = 0, /**< A mode without any control mechanism. It don't block repeating signals and recive many redundant signals per second. */
	CONTROL, /**< This mode controls and blocks repeating signals when you press and hold button on the IR remote. This method is based on control bit in RC5 frame.  */
	COOLDOWN /**< This mode works similar like a PC keyboard. Repeating of signal is delayed by SysTick. */
};

/** \brief RC5 frame struct. 
 *
 *  Struct, which contains data about one received RC5 frame. 
 */
struct RC5Struct{
	uint8_t control_bit; /**< A control bit, which toggles with each button press. */
	uint8_t address_bits; /**< A five-bit system address. */
	uint8_t data_bits; /**< A six-bit command. */
	uint8_t philips_bit; /*!< A field bit, which denotes whether the command sent is in the lower field or the upper field. A part of extended protocal version. */
	uint32_t raw; /**< Raw value */
	uint16_t duration; /**< A duration of bit in the frame. It can be measured or fixed. */
};


/** \brief Inicialization of timer, which measures duration of single bit and receaving all of them.  */
void RC5TimerInit();

/** \brief Inicliaziation of EXTI, NVIC, GPIO, RCC, timer  and Systick. 
 *
 *  It also calls #RC5TimerInit. 
 */
void RC5Init(); 

/** \brief Function what tries to get RC5 frame. 
 *
 *  Function reads from #rc_buffor bits and writes data to structure #RC5Struct using proper masks. 
 *  @param s output paramater, data from received frame will be saved in this structure.
 *  @param mode mode of receivng, look #RC5Mode.
 *  @return 1 when RC5 frame is received and ready, otherwise 0.
 */
uint8_t getRC5Signal(struct RC5Struct* s, uint8_t mode); 

/** \brief Function gets number value consisting of given digits amount from IR remote  
 *
 *  Function reads digits and creates number based on them. It uses CONTROL #RC5Mode.
 *  @param s output paramater, data from received frame will be saved in this structure.
 *  @param digits number of digits to read.
 *  @return created number.
 */
int getNumber(struct RC5Struct* s, uint8_t digits);

/** \brief Function creates string representation of #RC5Struct.  
 *
 *  Function reads digits and creates number based on them. It uses CONTROL #RC5Mode.
 *  @param s structure, which contens data of RC5 frame.
 *  @return string representaton of structure.
 */
const char* RC5toString(struct RC5Struct s);

/** \brief Function creates string with binary value of RC5 frame.   
 *
 *  @param raw raw value of RC5 frame.
 *  @return binary string.
 */
const char* RC5sToBinaryString(uint16_t raw);

/** \brief Function enables AutoFrequency mode. */
void RC5EnableAutoFrequency();

/** \brief Function disables AutoFrequency mode. */
void RC5DisableAutoFrequency();

/** \brief Function sets frequency to fixed value and disables AutoFrequency mode.
 *  @param f frequency.
 */
void RC5SetFrequency(uint16_t f);


#endif
