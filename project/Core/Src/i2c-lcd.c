#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;

#define SLAVE_ADDRESS_LCD 0x4E

void lcd_enviar_cmd (char cmd)
{
  char datos_u, datos_l;
	uint8_t datos_t[4];
	datos_u = (cmd&0xf0);
	datos_l = ((cmd<<4)&0xf0);
	datos_t[0] = datos_u|0x0C;  //en=1, rs=0
	datos_t[1] = datos_u|0x08;  //en=0, rs=0
	datos_t[2] = datos_l|0x0C;  //en=1, rs=0
	datos_t[3] = datos_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) datos_t, 4, 100);
}

void lcd_enviar_datos (char datos)
{
	char datos_u, datos_l;
	uint8_t datos_t[4];
	datos_u = (datos&0xf0);
	datos_l = ((datos<<4)&0xf0);
	datos_t[0] = datos_u|0x0D;  //en=1, rs=0
	datos_t[1] = datos_u|0x09;  //en=0, rs=0
	datos_t[2] = datos_l|0x0D;  //en=1, rs=0
	datos_t[3] = datos_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) datos_t, 4, 100);
}

void lcd_enviar(char *string,int fila,int col)
{
	lcd_cur(fila,col);
	lcd_enviar_string(string);
}

void lcd_clear (void)
{
	lcd_enviar_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_enviar_datos (' ');
	}
}

void lcd_cur(int fila, int col)
{
    switch (fila)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_enviar_cmd (col);
}


void lcd_init (void)
{
	HAL_Delay(50);
	lcd_enviar_cmd (0x30);
	HAL_Delay(5);
	lcd_enviar_cmd (0x30);
	HAL_Delay(1);
	lcd_enviar_cmd (0x30);
	HAL_Delay(10);
	lcd_enviar_cmd (0x20);
	HAL_Delay(10);

	lcd_enviar_cmd (0x28);
	HAL_Delay(1);
	lcd_enviar_cmd (0x08);
	HAL_Delay(1);
	lcd_enviar_cmd (0x01);
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_enviar_cmd (0x06);
	HAL_Delay(1);
	lcd_enviar_cmd (0x0C);
}

void lcd_enviar_string (char *str)
{
	while (*str) lcd_enviar_datos (*str++);
}
