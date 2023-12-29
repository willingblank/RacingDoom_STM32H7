#ifndef __Text_H__
#define __Text_H__	 

#include "common.h"
#include "font.h"


//////////////////////////////////////////////////////////////////////////////////	 
 
//汉字显示 驱动代码	

//极客  极客创新
//淘宝店铺：极客.taobao.com
//淘宝店铺：极客.taobao.com	
/******************************************************************************/		 

void Show_Str(u16 x,u16 y,u16 width,u16 height,u8*str,u8 size, u16 color, u8 mode);	//在指定位置显示一个字符串 

void Draw_Font16B(u16 x,u16 y, u16 color, u8*str);//在指定位置--显示 16x16 大小的点阵字符串

void Draw_Font24B(u16 x,u16 y, u16 color, u8*str);//在指定位置--显示 24x24 大小的点阵字符串


#endif



