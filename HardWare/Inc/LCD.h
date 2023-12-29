#ifndef __LCD_H
#define __LCD_H		

#include "common.h"
#include "font.h"
#include "gpio.h"

#define	LCD_Delay_ms  	HAL_Delay   //延时函数

//液晶控制口置1操作语句宏定义
#define	LCD_SDA_SET  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET) 	   //PB15置1    LCD_SDI： PB15 //数据输入线

#define	LCD_SCL_SET  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET)	     //PB13置1    LCD_SCL： PB13 //时钟线

#define	LCD_CS_SET  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET)	     //PB12置1    LCD_CS：  PB12 //片选	

#define LCD_RST_Set   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)	     //PB14置1      LCD_SDO ：PB14 //数据输出/复位

#define	LCD_RS_SET  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET)		   //PB1置1     LCD_RS： PB1   //命令/数据切换

#define	LCD_BLK_SET  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET) 	     //PB0置1     LCD_BLK ：PB0   //背光控制  

//液晶控制口置0操作语句宏定义

#define	LCD_SDA_CLR  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)  	  //PB15置0 //DIN  LCD_SDI： PB15 //数据输入线

#define	LCD_SCL_CLR  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET)	    //PB13置0 //CLK  LCD_SCL： PB13 //时钟线

#define	LCD_CS_CLR  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET)	   	//PB12置0/CS     LCD_CS：  PB12 //片选	

#define LCD_RST_Clr   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)	    //PB14置0 //RES  LCD_SDO ：PB14 //数据输出/复位

#define	LCD_RS_CLR  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET)		 	//PB1置0 //DC    LCD_RS： PB1   //命令/数据切换

#define	LCD_BLK_CLR  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET)  	  //PB0置0 //DIN   LCD_BLK ：PB0   //背光控制  

#define	LCD_BLK_On          LCD_BLK_SET   		 //开背光  LCD背光控制  
#define	LCD_BLK_Off         LCD_BLK_CLR    		 //关背光  LCD背光控制	

//LCD重要参数集
typedef struct  
{										    
	u16 width;			//LCD 宽度
	u16 height;			//LCD 高度
	u16 id;				  //LCD ID
	u8  dir;			  //横屏还是竖屏控制：竖屏和横屏。	
	u16	wramcmd;		//开始写gram指令
	u16  setxcmd;		//设置x坐标指令
	u16  setycmd;		//设置y坐标指令 
}_lcd_dev; 	  

/////////////////////////////////////用户配置区///////////////////////////////////	

//支持横竖屏快速定义切换


#define LCD_DIR_Mode  	  0	    //4种工作模式，0和2是竖屏模式，1和3是横屏模式

#define USE_HORIZONTAL  	0     //方向设置： 		0,竖屏模式   1,横屏模式.

//////////////////////////////////////////////////////////////////////////////////	

///////////////////////////  颜色值  ///////////////////////////////////////////////////////

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	   0x001F  
#define BRED             0xF81F
#define GRED 			       0xFFE0
#define GBLUE			       0x07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0xBC40 //棕色
#define BRRED 			     0xFC07 //棕红色
#define GRAY  			     0x8430 //灰色


//GUI颜色

#define DARKBLUE      	 0x01CF	//深蓝色
#define LIGHTBLUE      	 0x7D7C	//浅蓝色  
#define GRAYBLUE       	 0x5458 //灰蓝色
//以上三色为PANEL的颜色 
 
 
#define LIGHTGREEN     	 0x841F //浅绿色
#define LGRAY 			     0xC618 //浅灰色(PANNEL),窗体背景色

#define GRAY0   0xEF7D   	    //灰色0 
#define GRAY1   0x8410      	//灰色1   
#define GRAY2   0x4208      	//灰色2  

#define LGRAYBLUE        0xA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0x2B12 //浅棕蓝色(选择条目的反色)



////////////////////////延时函数--宏定义/////////////////////////////////////////////////

void LCD_GPIO_Init(void);
void LCD_HardwareRest(void);
u8 SPI2_ReadWriteByte(u8 TxData);
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height);
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);
void LCD_Clear(u16 color);


void LCD_Init(void);
void LCD_Clear(u16 color);
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 Color);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 Color);
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r, u16 Color);
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u16 color,u8 mode);
u32 LCD_Pow(u8 m,u8 n);
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size,u16 color);
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u16 color,u8 mode);
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u16 color,u8 *p);
void DisplayButtonDown(u16 x1,u16 y1,u16 x2,u16 y2);
void DisplayButtonUp(u16 x1,u16 y1,u16 x2,u16 y2);
void Draw_Test(void);
void Color_Test(void);
void Font_Test(void);

#endif  
	 
	