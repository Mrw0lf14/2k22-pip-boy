#ifndef INC_SNAKE_H_
#define INC_SNAKE_H_

#include "stm32h7xx_hal.h"
#include "st7789.h"
#include "main.h"



struct cell{
	int8_t x;
	int8_t y;
};
#define SNAKESIZE 30
#define FIELDSIZE 10
#define CELLSIZE 20

extern struct cell snake[SNAKESIZE];
extern struct cell apple;
extern int8_t vx,vy;
extern uint8_t snakeCount;
extern uint8_t timFlag;

#define SnakeReset() snakeCount = 1


void SnakeCheckApple(void);
void SnakeDraw(void);
void SnakeMain(void);


#endif /* INC_SNAKE_H_ */
