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
	if (snake[0].x == 0 && vx < 0)
		snake[0].x = FIELDSIZE;
	else
		snake[0].x += vx;

	if (snake[0].y == 0 && vy < 0)
		snake[0].y = FIELDSIZE;
	else
		snake[0].y += vy;


	if(snake[0].x > FIELDSIZE){
		snake[0].x = 0;
	}
}

void SnakeCheckApple(void)
{
	if(snake[0].x == apple.x && snake[0].y == apple.y){
		snakeCount += 1;
		if (snakeCount >= SNAKESIZE)
			snakeCount = 1;
		snake[snakeCount].x = snake[0].x;
			snake[snakeCount].y = snake[0].y;
			apple.x = rand()%FIELDSIZE;
			apple.y = rand()%FIELDSIZE;
	}

}
void SnakeDraw(void)
{
	myfunc_FillWithColor(WHITE);
	for (uint16_t i = 0; i < snakeCount; i++) {
		//ST7789_DrawFilledRectangle(snake[i].x * CELLSIZE, snake[i].y * CELLSIZE, CELLSIZE, CELLSIZE, GREEN);
		myfunc_DrawFilledRectangle(snake[i].x * CELLSIZE, snake[i].y * CELLSIZE, CELLSIZE, CELLSIZE, GREEN);
	}
	myfunc_DrawFilledRectangle(apple.x * CELLSIZE, apple.y * CELLSIZE, CELLSIZE, CELLSIZE, RED);
}
void SnakeClear(void){
	myfunc_FillWithColor(BLACK);
	myfunc_UpdateFrame();
}
void SnakeGetV(void){
	if(HAL_ADC_Start(&hadc1) != HAL_OK){
		assert(0);
	}
	if(HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
		adc[0] = HAL_ADC_GetValue(&hadc1);
	}
	if(HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK){
		assert(0);
	}
	if(HAL_ADCEx_InjectedPollForConversion(&hadc1, 1) == HAL_OK){
		adc[1] = HAL_ADCEx_InjectedGetValue(&hadc1, 1);
	}
	if (adc[0] > 3000){
		vx = 1;
		vy = 0;
	}
	else if (adc[0] < 1000){
		vx = -1;
		vy = 0;
	}
	else
		vx = 0;
	if (adc[1] > 3000){
		vy = -1;
		vx = 0;
	}
	else if (adc[1] < 1000){
		vy = 1;
		vx = 0;
	}
	else
		vy = 0;
}
void SnakeMain(void)
{
	snakeCount = 1;
	apple.x = 8;
	apple.y = 8;
	HAL_TIM_Base_Start_IT(&htim2);
	srand(10);
	myfunc_SetAddressWindow(0, 0, 239, 319);
	SnakeDraw();
	timFlag = 0;
	//SnakeReset();
	while (1) {

		if(timFlag == 1){
			myfunc_UpdateFrame();
			SnakeGetV();
			SnakeUpdatePos();
			SnakeDraw();
			SnakeCheckApple();
			timFlag = 0;
		}
		myfunc_SetAddressWindow(0, 0, 239, 319);
	}
}
