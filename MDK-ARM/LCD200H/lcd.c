

#include "lcd.h"
#include "stdlib.h"
#include "font.h" 
#include "usart.h"	 
#include "Text.h"
#include "Dis_Picture.h" 
#include "SPI.h" 
//////////////////////////////////////////////////////////////////////////////////	 

////2.0�� 240x320 ������  LCD��ʾ Һ������	  
  
//STM32F4����ģ��-�⺯���汾
//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	

//********************************************************************************

//////////////////////////////////////////////////////////////////////////////////	 
				 
//LCD�Ļ�����ɫ�ͱ���ɫ	  

u16 POINT_COLOR=0x0000;	//������ɫ
u16 BACK_COLOR=0xFFFF;  //����ɫ 



//����LCD��Ҫ����
//Ĭ��Ϊ����
_lcd_dev lcddev;


//����������ʹ�õ���MCUӲ��SPI�ӿ�����


/******************************************************************************

  �ӿڶ��壬����ݽ����޸Ĳ��޸���ӦIO��ʼ��--�������Ӻ�IOҪ��Ӧ

//	#define LCD_SDI        	//PB15  //����������
//	#define LCD_SCL        	//PB13  //ʱ����
//	#define LCD_CS        	//PB12  //Ƭѡ	
//	#define LCD_SDO     		//PB14  //�������/��λ
//	#define LCD_RS         	//PB1   //����/�����л�
//	#define LCD_BLK         //PB0   //������� 

*******************************************************************************/


/******************************************************************************/

//������ void LCD_GPIO_Init(void)

//�������ܣ�����Һ��IO��ʼ������

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	


//	#define LCD_SDI        	//PB15  //����������
//	#define LCD_SCL        	//PB13  //ʱ����
//	#define LCD_CS        	//PB12  //Ƭѡ	
//	#define LCD_SDO     		//PB14  //�������/��λ
//	#define LCD_RS         	//PB1   //����/�����л�
//	#define LCD_BLK         //PB0   //�������  	

/******************************************************************************/
void LCD_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();					          //����PBʱ��
	

    GPIO_Initure.Pin=GPIO_PIN_0 |GPIO_PIN_1 | GPIO_PIN_12;	//PB
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  		    //�������
    GPIO_Initure.Pull=GPIO_PULLUP;         			    //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;  	//����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);     	     	//��ʼ��
	  
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	  //PB0��1 	
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);	  //PB1��1 
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);	//PB12��1 

  
		
		SPI2_Init();           //��ʼ��Ӳ��SPI2
		
		SPI2_SetSpeed(SPI_BAUDRATEPRESCALER_2); //���ø���ģʽ
		
		LCD_HardwareRest();   //Ӳ��λ--���IO���ӣ�Ӳ����λ�ɿ���Ч
		
		LCD_BLK_On;           //��������
	
//	LCD_BLK_Off;   //�رձ���
//	LCD_BLK_On;    //��������
      
}



//********************************************************************/

//������void LCD_WR_REG(u16 regval)
//�������ܣ�
//д�Ĵ�������
//regval:�Ĵ���ֵ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_WR_REG(u16 regval)
{   
	 LCD_CS_CLR;  //LCD_CS=0  //Ƭѡ
   LCD_RS_CLR;  //LCD_RS=0  //��������״̬
	 SPI2_ReadWriteByte(regval&0x00FF);
	 LCD_CS_SET;  //LCD_CS=1	 
}



//*******************************************************************/

//������void LCD_WR_DATA8(u8 data)   //д8λ����
//�������ܣ�
//Һ����--д8λ���ݺ���

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/


void LCD_WR_DATA8(u8 data)   //д8λ����
{
	LCD_CS_CLR;  //LCD_CS=0  //Ƭѡ
	LCD_RS_SET;	 //LCD_RS=1   //��������״̬			    	   
	SPI2_ReadWriteByte(data);	
	LCD_CS_SET;  //LCD_CS=1   			 
}


//*******************************************************************/
//������void LCD_WR_DATA16(u16 data)   //д16λ����
//�������ܣ�дLCD����
//���������
//data:Ҫд���ֵ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/


void LCD_WR_DATA16(u16 data)
{	
	
 	LCD_CS_CLR;  //LCD_CS=0  //Ƭѡ
	LCD_RS_SET;	 //LCD_RS=1   //��������״̬
	SPI2_ReadWriteByte(data>>8);
	SPI2_ReadWriteByte(data);
	LCD_CS_SET;    //LCD_CS=1
	
}


//*******************************************************************/

//������void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)  //дҺ�����Ĵ���

//�������ܣ�д�Ĵ���

//�������: 
//LCD_Reg:�Ĵ�����ַ
//LCD_RegValue:Ҫд�������

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
{	
	LCD_WR_REG(LCD_Reg);		   //д��Ҫд�ļĴ������	 
	LCD_WR_DATA8(LCD_RegValue);//д������	    		 
}


//*******************************************************************/
//������void LCD_WriteRAM_Prepare(void)   
//�������ܣ���ʼдGRAM  ����

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_WriteRAM_Prepare(void)
{
 	LCD_WR_REG(lcddev.wramcmd);	  
}	



//*******************************************************************/
//������LCD_WriteRAM(u16 RGB_Code)   
//�������ܣ�д�������ɫֵ
//�������:
//RGB_Code:��ɫֵ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_WriteRAM(u16 RGB_Code)
{							    
	LCD_WR_DATA16( RGB_Code );//дʮ��λGRAM
}





//********************************************************************************/
//������void LCD_DisplayOn(void)
//�������ܣ�
//LCD������ʾ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/
void LCD_DisplayOn(void)
{					   
	LCD_WR_REG(0X29);	//������ʾ
}	


//*******************************************************************/
//������void LCD_DisplayOff(void)
//�������ܣ�
//LCD�ر���ʾ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_DisplayOff(void)
{	   
	LCD_WR_REG(0X28);	//�ر���ʾ

} 

//********************************************************************************/
//������void LCD_SoftRest(void)
//�������ܣ�����Ļ�����ִ������λ����
//LCD������ʾ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/
void LCD_SoftRest(void)
{					   
	LCD_WR_REG(0x01);	//��������λ����
	LCD_Delay_ms(50);      // delay 50 ms 
}	


//********************************************************************************/
//������void LCD_SoftRest(void)
//�������ܣ�����Ļ�����ִ������λ����
//LCD������ʾ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/
void LCD_HardwareRest(void)
{					   
	LCD_RST_Clr;     //Һ������λ --Ӳ��λ--ʹ��       //PB14��ΪҺ������λ��������
	LCD_Delay_ms(50);      // delay 50 ms 
	LCD_RST_Set;      //Һ������λ --Ӳ��λ--ʧ��       //PB14��ΪҺ������λ��������
	LCD_Delay_ms(30);      // delay 30 ms 
}	



//*******************************************************************/
//������void LCD_SetCursor(u16 Xpos, u16 Ypos)
//�������ܣ����ù��λ��
//���������
//Xpos:������
//Ypos:������

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/
void LCD_SetCursor(u16 Xpos, u16 Ypos)
{	 
//		if(LCD_DIR_Mode==2)Ypos=Ypos;
//	  if(LCD_DIR_Mode==3)Xpos=Xpos;
			
		LCD_WR_REG(lcddev.setxcmd); 
	  LCD_WR_DATA16(Xpos);
	
		LCD_WR_REG(lcddev.setycmd);
    LCD_WR_DATA16(Ypos);	

	
}





//*******************************************************************/
//������void LCD_DrawPoint(u16 x,u16 y)
//�������ܣ�����
//���������
//x,y:����
//POINT_COLOR:�˵����ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_SetCursor(x,y);		//���ù��λ�� 
	LCD_WriteRAM_Prepare();	//��ʼд��GRAM
	LCD_WR_DATA16(POINT_COLOR); 
}


//*******************************************************************/
//������void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
//�������ܣ����ٻ���
//���������
//x,y:����
//color:��ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color)
{	
	
//    if(LCD_DIR_Mode==2)y=y; 
//	  if(LCD_DIR_Mode==3)x=x; 
	
		LCD_WR_REG(lcddev.setxcmd); 
	  LCD_WR_DATA16(x);

  	LCD_WR_REG(lcddev.setycmd); 
	  LCD_WR_DATA16(y);
			 
	  LCD_WR_REG(lcddev.wramcmd); 
	  LCD_WR_DATA16(color); //д��16λ��ɫ
}	 



//*******************************************************************/
//������void LCD_Scan_Dir(u8 dir)
//�������ܣ�����LCD���Զ�ɨ�跽��
//���������

//Ĭ������ΪL2R_U2D,�������Ϊ����ɨ�跽ʽ,���ܵ�����ʾ������.
//dir:0~7,����8������

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*******************************************************************/

void LCD_Scan_Dir(u8 dir)
{
	u8 regval=0;

	//ɨ�跽����--ɨ�跽ʽ�в�ͬ��񣬿��ܶ��岻���Һ����µĲ��շ���ͬ���᷽ܽʽ��ֻ��һ�°���
	
	
	switch(dir)
		{
			case 0:
				regval|=(0<<7)|(0<<6)|(0<<5); 
				break;
			case 1:
				regval|=(0<<7)|(1<<6)|(1<<5); 
				break;
			case 2:
				regval|=(1<<7)|(1<<6)|(0<<5); 
				break;
			case 3:
				regval|=(1<<7)|(0<<6)|(1<<5); 
				break;	 
	 
		}
		
    		
		LCD_WriteReg(0x36,regval);//�ı�ɨ�跽������  ---�˴���Ҫ�鿴�����ֲᣬȷ��RGB��ɫ����λ������


} 




/**************************************************************************/
//������void LCD_Display_Dir(u8 dir)
//�������ܣ�����LCD����ʾ�������ز���

//���������

//����LCD��ʾ����

////dir:   0,����  ��
//         1,����  ��
//         2,����  ��
//         3,����  ��

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*************************************************************************/
void LCD_Display_Dir(u8 dir)
{
	
	u8 SCAN_DIR;
		
	if(dir==0)			     //����  ��
	{
		lcddev.dir=0;	     //����
		lcddev.width=240;
		lcddev.height=320;
    
		lcddev.wramcmd=0X2C;
	 	lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;

    SCAN_DIR=0; //ѡ��ɨ�跽��		
		

	}
	
else if (dir==1)			 //����
	{	  				
		lcddev.dir=0;	     //����
		lcddev.width=320;
		lcddev.height=240;
    
		lcddev.wramcmd=0X2C;
	 	lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;
		
    SCAN_DIR=1; //ѡ��ɨ�跽��		

		
	} 	
	
	
	else if (dir==2)			//����  
	{	  				
		lcddev.dir=1;	     //����  
		lcddev.width=240;
		lcddev.height=320;

		lcddev.wramcmd=0X2C;
	 	lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;  
		
    SCAN_DIR=2; //ѡ��ɨ�跽��		
	
		
	} 
 else if (dir==3)				  //����
	{	  				
		lcddev.dir=1;	        //����
		lcddev.width=320;
		lcddev.height=240;

		lcddev.wramcmd=0X2C;
	 	lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B; 
    
    SCAN_DIR=3; //ѡ��ɨ�跽��

	

	} 	
 else //����Ĭ��Ϊ����--��
 {
	  lcddev.dir=0;	     //����
		lcddev.width=240;
		lcddev.height=320;
    
		lcddev.wramcmd=0X2C;
	 	lcddev.setxcmd=0X2A;
		lcddev.setycmd=0X2B;

    SCAN_DIR=0; //ѡ��ɨ�跽��		
	 

 }	 

 
 //////�������ã�Ϊ���ڲ������ã�������ȫ������ʾ��Χ			

    LCD_Set_Window(0,0,lcddev.width,lcddev.height);//����ȫ������	

 /////������Ļ��ʾ--ɨ�跽��
	
	  LCD_Scan_Dir(SCAN_DIR);	//������Ļ��ʾ--ɨ�跽��
	
     
}	


/**************************************************************************/
//������void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
//�������ܣ�����LCD����ʾ����

//���ô���,���Զ����û������굽�������Ͻ�(sx,sy).
//sx,sy:������ʼ����(���Ͻ�)
//width,height:���ڿ��Ⱥ͸߶�,�������0!!
//�����С:width*height.

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	
//*************************************************************************/

void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{   
	
	
	  width=sx+width-1;
	  height=sy+height-1;

	if(LCD_DIR_Mode==0)
		{	
			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA16(sx);      //���� X�������
			LCD_WR_DATA16(width);   //���� X�����յ�	
		
			LCD_WR_REG(lcddev.setycmd);
			LCD_WR_DATA16(sy);      //���� Y�������
			LCD_WR_DATA16(height);  //���� Y�����յ�
		}
	else if(LCD_DIR_Mode==1)
		{	
			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA16(sx);      //���� X�������
			LCD_WR_DATA16(width);   //���� X�����յ�	
		
			LCD_WR_REG(lcddev.setycmd);
			LCD_WR_DATA16(sy);      //���� Y�������
			LCD_WR_DATA16(height);  //���� Y�����յ�
		}	
		else if(LCD_DIR_Mode==2)
		{	
			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA16(sx);      //���� X�������
			LCD_WR_DATA16(width);   //���� X�����յ�	
		
			LCD_WR_REG(lcddev.setycmd);
			LCD_WR_DATA16(sy);      //���� Y�������
			LCD_WR_DATA16(height);  //���� Y�����յ�
		}		
	 else if(LCD_DIR_Mode==3)
		{	
			LCD_WR_REG(lcddev.setxcmd); 
			LCD_WR_DATA16(sx);      //���� X�������
			LCD_WR_DATA16(width);   //���� X�����յ�	
		
			LCD_WR_REG(lcddev.setycmd);
			LCD_WR_DATA16(sy);      //���� Y�������
			LCD_WR_DATA16(height);  //���� Y�����յ�
		}
} 


/*******************************************************************************/
//������void LCD_Init(void)
//�������ܣ���ʼ��lcd
//�ó�ʼ���������Գ�ʼ��ST7789V

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_Init(void)
{ 	
	 
   	LCD_GPIO_Init();        //��ʼ������ I/O�ӿ�	
	
	  LCD_SoftRest();        //����λ 
	
 	  LCD_Delay_ms(50);      // delay 50 ms 
	
		//************* Start Initial Sequence **********// 

		LCD_WR_REG(0x36);//�ڴ����ݷ��ʿ���
		LCD_WR_DATA8(0x00);

		//if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
		//else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
		//else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
		//else LCD_WR_DATA8(0xA0);

		LCD_WR_REG(0x3A);//�ӿ����ظ�ʽ
		LCD_WR_DATA8(0x05);

		LCD_WR_REG(0xB2);     //��������
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x33);
		LCD_WR_DATA8(0x33); 

		LCD_WR_REG(0xB7);    //�ſ���
		LCD_WR_DATA8(0x35);  

		LCD_WR_REG(0xBB);    //VCOM Setting
		LCD_WR_DATA8(0x19);

		LCD_WR_REG(0xC0);    //LCM Control
		LCD_WR_DATA8(0x2C);

		LCD_WR_REG(0xC2);   //VDV and VRH Command Enable
		LCD_WR_DATA8(0x01);

		LCD_WR_REG(0xC3);  //VRH Set
		LCD_WR_DATA8(0x12);   

		LCD_WR_REG(0xC4);  //VDV Set
		LCD_WR_DATA8(0x20);  

		LCD_WR_REG(0xC6);   //����ģʽ�µ�֡�ʿ���
		LCD_WR_DATA8(0x0F); //֡Ƶ��60֡   

		LCD_WR_REG(0xD0);   //Power Control
		LCD_WR_DATA8(0xA4);
		LCD_WR_DATA8(0xA1);

		LCD_WR_REG(0xE0);    //����ѹ٤�����
		LCD_WR_DATA8(0xD0);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x0D);
		LCD_WR_DATA8(0x11);
		LCD_WR_DATA8(0x13);
		LCD_WR_DATA8(0x2B);
		LCD_WR_DATA8(0x3F);
		LCD_WR_DATA8(0x54);
		LCD_WR_DATA8(0x4C);
		LCD_WR_DATA8(0x18);
		LCD_WR_DATA8(0x0D);
		LCD_WR_DATA8(0x0B);
		LCD_WR_DATA8(0x1F);
		LCD_WR_DATA8(0x23);

		LCD_WR_REG(0xE1);   //����ѹ٤�����
		LCD_WR_DATA8(0xD0);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x11);
		LCD_WR_DATA8(0x13);
		LCD_WR_DATA8(0x2C);
		LCD_WR_DATA8(0x3F);
		LCD_WR_DATA8(0x44);
		LCD_WR_DATA8(0x51);
		LCD_WR_DATA8(0x2F);
		LCD_WR_DATA8(0x1F);
		LCD_WR_DATA8(0x1F);
		LCD_WR_DATA8(0x20);
		LCD_WR_DATA8(0x23);

		LCD_WR_REG(0x21); //��ʾ��ת��

		LCD_WR_REG(0x11); //�˳�����

		delay_ms (120); 

		LCD_WR_REG(0x29); //����ʾ

		delay_ms (200); 

	  LCD_Display_Dir(LCD_DIR_Mode);	//ѡ��--��Ļ��ʾ��ʽ
		
	  LCD_BLK_On;					//��������
		
	  LCD_Clear(WHITE);
	
	
	
}


//////////////////���º�������Ļ��ʾͼ�εļ���������////////////GK  ���ʹ���//////����.taobao.com//////////////////////////////////////



/*******************************************************************************/
//������void LCD_Clear(u16 color)
//�������ܣ�ȫ��������亯��
//���������
//color:Ҫ���������ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_Clear(u16 color)
{
	u32 index=0;
	u32 totalpoint;
	
	LCD_Set_Window(0,0,lcddev.width,lcddev.height);//����ȫ������
	
	totalpoint=lcddev.width * lcddev.height; 			//�õ��ܵ���
 
	LCD_SetCursor(0x00,0x00);	//���ù��λ�� 
	
	LCD_WriteRAM_Prepare();     		//��ʼд��GRAM	
	
	for(index=0;index<totalpoint;index++)
	{
		LCD_WR_DATA16(color);	
	}
} 


/*******************************************************************************/
//������void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)

//�������ܣ�������亯��--��䵥����ɫ
//���������
//��ָ����������䵥����ɫ
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//color:Ҫ������ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color)
{          
		u32 i;
		u32 xlen=0;

 //���ô���
    LCD_Set_Window(sx,sy,ex-sx+1,ey-sy+1);//���ô���
		  	
	  LCD_WR_REG(lcddev.wramcmd);	  
	
 		xlen=(ex-sx+1)*(ey-sy+1);//������ܹ���Ҫд��ĵ���
		
		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM
		
		for(i=0;i<=xlen;i++)
		{
		 	LCD_WR_DATA16(color);	//��ʾ��ɫ 	    
		}


//�ָ�ȫ������
		
//�������ã�Ϊ���ڲ������ã�������ȫ������ʾ��Χ			
			
   LCD_Set_Window(0,0,lcddev.width,lcddev.height);//����ȫ������	
		 
}


/*******************************************************************************/
//������void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)

//�������ܣ�������亯��
//���������
//��ָ�����������ָ����ɫ��			 
//(sx,sy),(ex,ey):�����ζԽ�����,�����СΪ:(ex-sx+1)*(ey-sy+1)   
//color:Ҫ������ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color)
{  

		u32 i;
		u32 xlen=0;

	 //���ô���
	
	  LCD_Set_Window(sx,sy,ex-sx+1,ey-sy+1);//���ô���
  	
	  LCD_WR_REG(lcddev.wramcmd);	  
	
 		xlen=(ex-sx+1)*(ey-sy+1);//������ܹ���Ҫд��ĵ���
		
		LCD_WriteRAM_Prepare();     			//��ʼд��GRAM
		
		for(i=0;i<=xlen;i++)
		{
		 	LCD_WR_DATA16(*color);	//��ʾ��ɫ 	    
		}


//�ָ�ȫ������
		
//�������ã�Ϊ���ڲ������ã�������ȫ������ʾ��Χ			
			
  LCD_Set_Window(0,0,lcddev.width,lcddev.height);//����ȫ������	
		

}



/*******************************************************************************/
//������void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
//�������ܣ�����
//���������
//x1,y1:�������
//x2,y2:�յ�����
//Color;������ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2, u16 Color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	
	delta_x=x2-x1; //������������ 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	
	
	if(delta_x>0)incx=1; //���õ������� 
	else if(delta_x==0)incx=0;//��ֱ�� 
	else
     {
		    incx=-1;
		    delta_x=-delta_x;
		 }

	
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//ˮƽ�� 
	else
     { 
		   incy=-1;
		   delta_y=-delta_y;
		 }

	
	if( delta_x>delta_y)distance=delta_x; //ѡȡ�������������� 
	else distance=delta_y; 
	
	for(t=0;t<=distance+1;t++ )//������� 
	{  
		LCD_Fast_DrawPoint(uRow,uCol,Color);//���� --ʹ��������ɫ���� 
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
} 

/*******************************************************************************/
//������void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
//�������ܣ�������	  
//���������
//(x1,y1),(x2,y2):���εĶԽ�����
//Color;������ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2, u16 Color)
{
	LCD_DrawLine(x1,y1,x2,y1,Color);
	LCD_DrawLine(x1,y1,x1,y2,Color);
	LCD_DrawLine(x1,y2,x2,y2,Color);
	LCD_DrawLine(x2,y1,x2,y2,Color);
}

/*******************************************************************************/
//������void LCD_Draw_Circle(u16 x0,u16 y0,u8 r)
//�������ܣ���ָ��λ�û�һ��ָ����С��Բ
//���������
//(x,y):���ĵ�
//r    :�뾶
//Color;������ɫ

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_Draw_Circle(u16 x0,u16 y0,u8 r, u16 Color)
{
	int a,b;
	int di;
	a=0;b=r;	  
	di=3-(r<<1);             //�ж��¸���λ�õı�־
	while(a<=b)
	{
		LCD_Fast_DrawPoint(x0+a,y0-b,Color);             //5
 		LCD_Fast_DrawPoint(x0+b,y0-a,Color);             //0           
		LCD_Fast_DrawPoint(x0+b,y0+a,Color);             //4               
		LCD_Fast_DrawPoint(x0+a,y0+b,Color);             //6 
		LCD_Fast_DrawPoint(x0-a,y0+b,Color);             //1       
 		LCD_Fast_DrawPoint(x0-b,y0+a,Color);             
		LCD_Fast_DrawPoint(x0-a,y0-b,Color);             //2             
  	LCD_Fast_DrawPoint(x0-b,y0-a,Color);             //7     	         
		a++;
		//ʹ��Bresenham�㷨��Բ     
		if(di<0)di +=4*a+6;	  
		else
		{
			di+=10+4*(a-b);   
			b--;
		} 						    
	}
}


/*******************************************************************************/
//������LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u16 color,u8 mode)
//�������ܣ���ָ��λ����ʾһ���ַ�
//���������
//x,y:��ʼ����
//num:Ҫ��ʾ���ַ�:" "--->"~"
//size:�����С 12/16/24
//color,�ַ���ɫ
//mode:���ӷ�ʽ(1)���Ƿǵ��ӷ�ʽ(0)

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u16 color,u8 mode)
{  							  
  u8 temp,t1,t;
	u16 y0=y;
	u8 csize=(size/8+((size%8)?1:0))*(size/2);		//�õ�����һ���ַ���Ӧ������ռ���ֽ���	
	//���ô���		   
	num=num-' ';//�õ�ƫ�ƺ��ֵ
	for(t=0;t<csize;t++)
	{   
		if(size==12)temp=asc2_1206[num][t]; 	 	//����1206����
		else if(size==16)temp=asc2_1608[num][t];	//����1608����
		else if(size==24)temp=asc2_2412[num][t];	//����2412����
		else return;								//û�е��ֿ�
		for(t1=0;t1<8;t1++)
		{			    
			if(temp&0x80)LCD_Fast_DrawPoint(x,y,color);
			else if(mode==0)LCD_Fast_DrawPoint(x,y,BACK_COLOR);
			
			temp<<=1;
			y++;
			if(y>=lcddev.height)return;		//��������
			if((y-y0)==size)
			{
				y=y0;
				x++;
				if(x>=lcddev.width)return;	//��������
				break;
			}
		}  	 
	}  	    	   	 	  
}



/*******************************************************************************/
//������u32 LCD_Pow(u8 m,u8 n)
//�������ܣ�m^n����
//�������������8λ����
//����ֵ:m^n�η�.

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/
u32 LCD_Pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}	


/*******************************************************************************/
//������void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size,u16 color)
//�������ܣ���ʾ����,��λΪ0,����ʾ
//���������

//x,y :�������	 
//num:��ֵ(0~4294967295);	
//len :���ֵ�λ��
//size:�����С
//color:��ɫ 


//����ֵ:��

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size,u16 color)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+(size/2)*t,y,' ',size,color,0);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,color,0); 
	}
} 



/*******************************************************************************/
//������void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u16 color,u8 mode)
//�������ܣ���ʾ����,��λΪ0,������ʾ
//���������
//��ʾ����,��λΪ0,������ʾ
//x,y:�������
//num:��ֵ(0~999999999);	 
//len:����(��Ҫ��ʾ��λ��)
//size:�����С
//color:��ɫ 
//mode:
//[7]:0,�����;1,���0.
//[6:1]:����
//[0]:0,�ǵ�����ʾ;1,������ʾ.

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/

void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u16 color,u8 mode)
{  
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/LCD_Pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				if(mode&0X80)LCD_ShowChar(x+(size/2)*t,y,'0',size,color,mode&0X01);  
				else LCD_ShowChar(x+(size/2)*t,y,' ',size,color,mode&0X01);  
 				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+(size/2)*t,y,temp+'0',size,color,mode&0X01); 
	}
} 

/*******************************************************************************/
//������void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u16 color,u8 *p)
//�������ܣ���ʾ�ַ���
//���������
//x,y:�������
//width,height:�����С  
//size:�����С
//color:��ɫ 
//*p:�ַ�����ʼ��ַ		  

//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/*******************************************************************************/


void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u16 color,u8 *p)
{         
	u8 x0=x;
	width+=x;
	height+=y;
    while((*p<='~')&&(*p>=' '))//�ж��ǲ��ǷǷ��ַ�!
    {       
        if(x>=width){x=x0;y+=size;}
        if(y>=height)break;//�˳�
        LCD_ShowChar(x,y,*p,size,color,0);
        x+=size/2;
        p++;
    }  
}







//****************************************************************************************/
//������void DisplayButtonDown(u16 x1,u16 y1,u16 x2,u16 y2)
//��������: ����Ļ��ʾһ͹��İ�ť��
//��    ��: u16 x1,y1,x2,y2 ��ť�����ϽǺ����½�����
//��    ��: ��


//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/****************************************************************************************/

void DisplayButtonDown(u16 x1,u16 y1,u16 x2,u16 y2)
{
	
	LCD_DrawLine(x1,y1, x2,y1,GRAY2);  //H
		
	LCD_DrawLine(x1+1,y1+1,x2,y1+1,GRAY1);  //H
		
	LCD_DrawLine(x1,  y1,  x1,y2,GRAY2);    //V
		
	LCD_DrawLine(x1+1,y1+1,x1+1,y2,GRAY1);  //V
	
	LCD_DrawLine(x1, y2, x2, y2,WHITE);     //H
	LCD_DrawLine(x2, y1, x2, y2,WHITE);     //V
}

//****************************************************************************************/
//������void DisplayButtonUp(u16 x1,u16 y1,u16 x2,u16 y2)
//��������: ����Ļ��ʾһ���µİ�ť��
//��    ��: u16 x1,y1,x2,y2 ��ť�����ϽǺ����½�����
//��    ��: ��
//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/****************************************************************************************/
void DisplayButtonUp(u16 x1,u16 y1,u16 x2,u16 y2)
{
	
	LCD_DrawLine(x1,  y1,  x2,y1,WHITE);    //H
	LCD_DrawLine(x1,  y1,  x1,y2,WHITE);    //V
	
	LCD_DrawLine(x1+1,y2-1,x2,y2-1,GRAY1);  //H
		
	LCD_DrawLine(x1,  y2,  x2,y2,GRAY2);    //H
		
	LCD_DrawLine(x2-1,y1+1,x2-1,y2,GRAY1);  //V
		
  LCD_DrawLine(x2  ,y1  ,x2,y2,GRAY2);    //V
	

}


//****************************************************************************************/
//������void Draw_Test(void)
//��������: ����ͼ�κ�������


//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/****************************************************************************************/

void Draw_Test(void)
{
	
		LCD_Clear(WHITE); //����
		
	  LCD_DrawLine(20,64, 220,128,RED);//���ߺ���
	  LCD_DrawLine(20,128, 220,64,RED);//���ߺ���
	
	  LCD_DrawRectangle(20,64,220,128, BLUE);//���Ʒ���״
	  
	  LCD_Draw_Circle(120,96,81, BRED);//����Բ��
	  LCD_Draw_Circle(120,96,80, BRED);//����Բ��
	  LCD_Draw_Circle(120,96,79, BRED);//����Բ��
			
		delay_ms(2000);//��ʱ

}


//****************************************************************************************/
//������void Color_Test(void)
//��������: ��ɫ�����ʾ����


//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/****************************************************************************************/

void Color_Test(void)
{
	
		LCD_Clear(GRAY0); //����
		
		Draw_Font16B(24,16,BLUE,"1: ��ɫ������");
	
	  LCD_Fill(5,5,lcddev.width-5,lcddev.height-5,RED);//����һ�����ڣ�д��ָ��������ɫ
	
	  LCD_Fill(20,20,lcddev.width-20,lcddev.height-20,YELLOW);//����һ�����ڣ�д��ָ��������ɫ
	
	  LCD_Fill(50,50,lcddev.width-50,lcddev.height-50,BLUE);//����һ�����ڣ�д��ָ��������ɫ
	
	  LCD_Fill(80,100,lcddev.width-80,lcddev.height-80,MAGENTA);//����һ�����ڣ�д��ָ��������ɫ
	
		delay_ms(2000);

		LCD_Clear(WHITE);
	
	  Draw_Test();//��ͼ��������
	
		delay_ms(300);//��ʱ
		LCD_Clear(BLACK);
		delay_ms(300);//��ʱ
		LCD_Clear(RED);
		delay_ms(300);//��ʱ
		LCD_Clear(YELLOW);
		delay_ms(300);//��ʱ
		LCD_Clear(GREEN);
		delay_ms(300);//��ʱ
		LCD_Clear(BLUE);
		delay_ms(300);//��ʱ

}


uint16_t D_Color=BLUE; //������ɫ
uint16_t B_Color=WHITE; //������ɫ

//****************************************************************************************/
//������void Font_Test(void)
//��������: ����������ʾ����


//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/****************************************************************************************/

void Font_Test(void)
{
	LCD_Clear(GRAY0);
	Draw_Font16B(24,16,BLUE,"2: ������ʾ����");

	delay_ms(1000);
	LCD_Clear(GRAY0);
	
	Draw_Font24B(16,4,BLACK,"GK-���ʹ���");
	Draw_Font24B(48,32,BLACK,"Һ��������");
	
	Draw_Font16B(24,80, BLACK,"Ƕ��ʽ������");
	Draw_Font16B(24,96, GREEN,"���Ĳ���");
	Draw_Font16B(24,112,BLUE,"רעҺ������");
	Draw_Font16B(24,128,RED, "ȫ�̼���֧��");
	Draw_Font16B(16,144,BLUE,"F: ����.taobao.com");
	Draw_Font16B(16,160,BLUE,"F: ����.taobao.com");
	Draw_Font16B(16,176,RED,"0123456789 ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz");

	delay_ms(3000);	
	
	LCD_Clear(GRAY0);


}


//****************************************************************************************/
//������void Demo_Menu(void)
//��������: ��ʾ����˵�


//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com
/****************************************************************************************/

void Demo_Menu(void)
{

	LCD_Clear(GRAY0); //����
	
	Draw_Font24B(16,4,MAGENTA,"GK-���ʹ���");
	Draw_Font24B(48,32,MAGENTA,"Һ��������");


	DisplayButtonUp(16,64,224,88); //x1,y1,x2,y2
	Draw_Font16B(48,68,BLUE,"1: ��ɫ������");

	DisplayButtonUp(16,104,224,128); //x1,y1,x2,y2
	Draw_Font16B(48,108,BLUE,"2: ������ʾ����");

	DisplayButtonUp(16,144,224,168); //x1,y1,x2,y2
	Draw_Font16B(48,148,BLUE,"3: ͼƬ��ʾ����");
  
	Draw_Font16B(16,180,RED,"F: ����.taobao.com");
	Draw_Font16B(16,196,RED,"F: ����.taobao.com");
	
	delay_ms(2000);//��ʱ
		
	Color_Test();       //��ɫ������
	Font_Test();        //����������ʾ����
  GBK_LibFont_Test(); //GBK�ֿ����
	//Show_Picture();     //��ʾһ��ͼƬ--ͼƬ̫��оƬ�ڲ��洢�����ã��˲�������
	


}




































/************************************************************************************************/

//STM32H7����ģ��-HAL�⺯���汾
//GK  ���ʹ���
//�Ա����̣�����.taobao.com
//�Ա����̣�����.taobao.com	

/************************************************************************************************/









