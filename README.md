# stm32-rc5
My implementation of remote control communication protocol (RC5-extended) on the STM32 NUCLEO-F103RB board. I use in this project a TSOP22 IR receiver module. 

## How it works?
My implementation of RC5 is based on the EXTI (external interrupt) and the timer. EXTI will be executing on falling edge:
* when start bit is received (10),
* when next falling edge is detected.

This two points is used to measuring duration of a single bit. We consider three cases:
* when a start bit is 1 (10) and a field bit is 1 (10):

![case1](https://user-images.githubusercontent.com/10513420/42171911-d4963ee2-7e1a-11e8-96a3-79e745521b76.png)

* when a start bit is 1 (10), a field bit is 0 (01) and a control bit is 1 (10):

![case2](https://user-images.githubusercontent.com/10513420/42172230-b75809a4-7e1b-11e8-876c-fe7e42716114.png)

* when a start bit is 1 (10), a field bit is 0 (01) and a control bit is 0 (01):

![case3](https://user-images.githubusercontent.com/10513420/42172522-8423351c-7e1c-11e8-9028-ae38ac1a7d03.png)

## Example usage
```c
#include "rc5.h"
#include "rgb.h"
...
int main(void) {
  struct RC5Struct rc;
  while(1){
    if([some_condition]) {
      printf("Waiting for pressing a button on the IR remote...\r\n");
      while(getRC5Signal(&rc,0)!=1);
      printf("%s\r\n", RC5toString(rc));
    }
    if(getRC5Signal(&rc, COOLDOWN)){
      switch(rc.data_bits){
        case RED:
          setColor(255,0,0);
          break;
        case GREEN:
          setColor(0,255,0);
          break;
        case BLUE:
          setColor(0,0,255);
          break;
        case YELLOW:
          setColor(255,255,0);
          break;
        case AV:
          setColor(255,0,0);
          c.r = getNumber(&rc, 3);
          setColor(0,255,0);
          c.g = getNumber(&rc, 3);
          setColor(0,0,255);
          c.b = getNumber(&rc, 3);
          setColorFromStruct(c);
          break;
      }
    }
  }
}
```
