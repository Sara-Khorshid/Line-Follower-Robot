#include "stm32f1xx_hal.h"
#define RSL HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET)
#define RSH HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET)
#define RWL HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)
#define RWH HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET)
#define EL  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)
#define EH  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET)

void comm(char c)
{
  HAL_GPIO_WritePin(GPIOB,(0xf<<4),GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB,(c&0xf0),GPIO_PIN_SET);
  RSL;
  RWL;
  EH;
  EL;
  HAL_Delay(2);  
	HAL_GPIO_WritePin(GPIOB,(0xf<<4),GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB,((c&0x0f)<<4),GPIO_PIN_SET);
  RSL;
  RWL;
  EH;
  EL;
  HAL_Delay(2);
}

void datw(char c)
{
	
  HAL_GPIO_WritePin(GPIOB,(0xf<<4),GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB,(c&0xf0),GPIO_PIN_SET);
  RSH;
  RWL;
  EH;
  EL;
  HAL_Delay(2);  
	HAL_GPIO_WritePin(GPIOB,(0xf<<4),GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOB,((c&0x0f)<<4),GPIO_PIN_SET);
  RSH;
  RWL;
  EH;
  EL;
  HAL_Delay(2);
}

void lcd_init(void)
{
 HAL_Delay(30);
 comm(0x33);
 comm(0x32); 
 comm(0x28); 
 comm(0x08);
 comm(0x01); 
 comm(0x06);  
 comm(0x0e);
}

void lcd_putchar(char c)
{
 datw(c);
}

void lcd_clear(void)
{
 comm(0x01);
}

void lcd_gotoxy(unsigned char x,unsigned char y)
{
 if(y==0) comm(0x80+x);
 if(y==1) comm(0xC0+x);
}

// write the string str located in SRAM to the LCD
void lcd_puts(char *str)
{
char k;
while (k=*str++) lcd_putchar(k);
}