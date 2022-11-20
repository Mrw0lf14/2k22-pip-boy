#include "snake.h"

struct cell snake[SNAKESIZE];
struct cell apple;
int8_t vx,vy;
uint8_t snakeCount;
uint8_t timFlag;

void SnakeUpdatePos(void)
{
	for (uint16_t i = snakeCount; i >= 1; i--) {
		snake[i].x = snake[i - 1].x;
		snake[i].y = snake[i - 1].y;
	}
	snake[0].x += vx;
	snake[0].y += vy;


	if(snake[0].x > FIELDSIZE){
		snake[0].x = 0;
	}
}

void SnakeCheckApple(void)
{
	if(snake[0].x == apple.x && snake[0].y == apple.y){
		snakeCount += 1;
		snake[snakeCount].x = snake[0].x;
			snake[snakeCount].y = snake[0].y;
			apple.x = rand()%FIELDSIZE;
			apple.y = rand()%FIELDSIZE;
	}

}
void SnakeDraw(void)
{
	for (uint16_t i = 0; i < snakeCount; i++) {
		ST7789_DrawFilledRectangle(snake[i].x * CELLSIZE, snake[i].y * CELLSIZE, CELLSIZE, CELLSIZE, GREEN);
	}
	ST7789_DrawFilledRectangle(apple.x * CELLSIZE, apple.y * CELLSIZE, CELLSIZE, CELLSIZE, RED);
}
void SnakeClear(void){
	ST7789_Fill_Color(WHITE);
}
void SnakeMain(void)
{
	snakeCount = 8;
	apple.x = 8;
	apple.y = 8;
	vx = 1;
	srand(10);
	//SnakeReset();
	while (1) {
		HAL_Delay(500);
		SnakeClear();
		SnakeUpdatePos();
		SnakeDraw();
		SnakeCheckApple();
		timFlag = 0;
	}
}
