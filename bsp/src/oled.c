/*
 * oled.c
 *
 *  Created on: 7 juin 2016
 *      Author: cyril
 */

#include "oled.h"
#include "font.h"
#include "bsp.h"
#include "delay.h"

#define I2C_TIMEOUT	5000;

void BSP_OLED_Init()
{
	BSP_DELAY_ms(10);

	BSP_OLED_SEND_CMD(SSD1306_DISPLAYOFF);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETDISPLAYCLOCKDIV);
	BSP_OLED_SEND_CMD(0x80);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETMULTIPLEX);
	BSP_OLED_SEND_CMD(SSD1306_LCDHEIGHT - 1);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETDISPLAYOFFSET);
	BSP_OLED_SEND_CMD(0x00);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETSTARTLINE | 0x0);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_CHARGEPUMP);
	BSP_OLED_SEND_CMD(0x14);						// 0x10 -> External Vcc, 0x14 -> Internal Vcc
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_MEMORYMODE);
	BSP_OLED_SEND_CMD(0x00);
	BSP_DELAY_ms(50);

	//OLED_SendCommand(SSD1306_SEGREMAP);
	BSP_OLED_SEND_CMD(0xA1);

	BSP_OLED_SEND_CMD(SSD1306_COMSCANDEC);
	//OLED_SendCommand(SSD1306_COMSCANINC);

	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETCOMPINS);
	BSP_OLED_SEND_CMD(0x12);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETCONTRAST);
	BSP_OLED_SEND_CMD(0xCF);						// 0x9F -> External Vcc, 0xCF -> Internal Vcc
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETPRECHARGE);
	BSP_OLED_SEND_CMD(0xF1);						// 0x22 -> External Vcc, 0xF1 -> Internal Vcc
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_SETVCOMDETECT);
	BSP_OLED_SEND_CMD(0x40);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_DISPLAYALLON_RESUME);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_NORMALDISPLAY );
	//OLED_SendCommand(SSD1306_INVERTDISPLAY);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_DEACTIVATE_SCROLL);
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(0x40);            	//Set Display Start line = 0
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(0xB0);            	//Set Display Start Page = 0
	BSP_DELAY_ms(50);

	BSP_OLED_Clear();					// Clear Display
	BSP_DELAY_ms(50);

	BSP_OLED_SEND_CMD(SSD1306_DISPLAYON);
	BSP_DELAY_ms(50);
}

void BSP_OLED_Off(void)
{
	BSP_OLED_SEND_CMD(0xAE);			   // Display OFF
	BSP_DELAY_ms(50);
}

void BSP_OLED_Clear(void)
{
  unsigned char i,k;

  for(k=0;k<8;k++)
  {
	  BSP_OLED_setXY(k,0);
	  for(i=0;i<128;i++)
	  {
		  BSP_OLED_SEND_CHAR(0);         //clear all COL
	  }
  }
}

void BSP_OLED_setXY(uint8_t row, uint8_t col)
{
	BSP_OLED_SEND_CMD(0xb0+row);         		//set page address
	BSP_OLED_SEND_CMD(0x00+(8*col&0x0f));       	//set low col address
	BSP_OLED_SEND_CMD(0x10+((8*col>>4)&0x0f));  	//set high col address
}


void BSP_OLED_PrintChar(uint8_t byte)
{
	uint8_t	j;

	for (j=0; j<5; j++)
	{
		BSP_OLED_SEND_CHAR(font5x8[(5*byte)+j]);
	}
	BSP_OLED_SEND_CHAR(0x00);						// Space between chars
}

void BSP_OLED_SendStr(uint8_t *string)
{
	uint8_t j;

	while (*string)
	{
		for (j=0; j<5; j++)
		{
			BSP_OLED_SEND_CHAR(font5x8[(*string*5)+j]);
		}
		BSP_OLED_SEND_CHAR(0x00);						// Space between chars
		string++;
	}
}



