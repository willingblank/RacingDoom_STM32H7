#include "LCD.h"
#include "Text.h"	
#include "string.h"												    
#include "usart.h"		

extern _lcd_dev lcddev;
extern u16 POINT_COLOR;	//画笔颜色
extern u16 BACK_COLOR;  //背景颜色

//////////////////////////////////////////////////////////////////////////////////	 
 
//汉字显示 驱动代码	    

/******************************************************************************/								  


/**************************************************************************************/

//函数： void Copy_Mem (unsigned char *P1, const unsigned char *P2,unsigned int Count)

//函数功能：内部存储-拷贝函数
/**************************************************************************************/

void Copy_Mem (unsigned char *P1, const unsigned char *P2,unsigned int Count)
{
   
   unsigned int i ;
   for(i=0;i<Count;i++)*P1++=*P2++;

  
}


/**************************************************************************************/
//函数：Show_Str(u16 x,u16 y,u16 width,u16 height,u8*str,u8 size, u16 color, u8 mode)
//函数功能：在指定位置开始显示一个字符串，支持自动换行	
//参数：
//(x,y):起始坐标
//width,height:区域
//str  :字符串
//size :字体大小
//color,字符颜色
//mode:0,非叠加方式;1,叠加方式    	
/**************************************************************************************/
void Show_Str(u16 x,u16 y,u16 width,u16 height, u8*str, u8 size, u16 color, u8 mode)
{					
	u16 x0=x;
	u16 y0=y;							  	  
  u8 bHz=0;     //字符或者中文  
		
  while(*str!=0)//数据未结束
    { 
        if(!bHz)
        {
	        if(*str>0x80)bHz=1;//中文 
					
	        else              //字符
	        {      
            if(x>(x0+width-size/2))//换行
						{				   
							y+=size;
							x=x0;	   
						}	
						
		        if(y>(y0+height-size))break;//越界返回
						
		        if(*str==13)//换行符号
		        {         
		            y+=size;
					      x=x0;
		            str++; 
		        }  
		        else LCD_ShowChar(x,y,*str,size,color,mode);//有效部分写入 
				    str++; 
		        x+=size/2; //字符,为全字的一半 
	        }
        }
				else//中文 
        {     
            bHz=0;//有汉字库 
					
            if(x>(x0+width-size))//换行
						{	    
							y+=size;
							x=x0;		  
						}
						
	        if(y>(y0+height-size))break;//越界返回  
						
	        //Show_Font(x,y,str,size,color,mode); //显示这个汉字,空心显示 
						
	        str+=2; 
	        x+=size;//下一个汉字偏移	    
        }						 
    }   
}

//**************************************************************************************/
//函数：Draw_Font16B(u16 x,u16 y, u16 color, u8*str)
//函数功能：在指定位置开始显示一个16x16点阵的--字符串，支持自动换行(至屏幕一行的终点后，自动换到下一个x起点位置)	
//参数：

//(x,y):起始坐标
//color,字符颜色
//str  :字符串


//非叠加方式;非点阵字符的部分，填充背景颜色

//**************************************************************************************/


void Draw_Font16B(u16 x,u16 y, u16 color, u8*str)
{
	u16 width;
	u16 height;
	
	width=lcddev.width-x;
	height=lcddev.height-y;
	
	Show_Str(x,y,width,height, str, 16, color, 0);
	
	
}

//**************************************************************************************/
//函数：void Draw_Font24B(u16 x,u16 y, u16 color, u8*str)
//函数功能：在指定位置开始显示一个24x24点阵的--字符串，支持自动换行(至屏幕一行的终点后，自动换到下一个x起点位置)	
//参数：

//(x,y):起始坐标
//color,字符颜色
//str  :字符串


//非叠加方式;非点阵字符的部分，填充背景颜色

//**************************************************************************************/

void Draw_Font24B(u16 x,u16 y, u16 color, u8*str)
{
	u16 width;
	u16 height;
	
	width=lcddev.width-x;
	height=lcddev.height-y;
	
	Show_Str(x,y,width,height, str, 24, color, 0);
	
	
}









		  






