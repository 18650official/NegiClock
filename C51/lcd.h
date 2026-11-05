#ifndef __LCD_H_
#define __LCD_H_
/**********************************
当使用的是4位数据传输的时候定义，
使用8位取消这个定义
**********************************/
#define LCD1602_4PINS

/**********************************
包含头文件
**********************************/
#include<REGX52.H>
#include "delay.h"

//---重定义关键词---//
#ifndef uchar
#define uchar unsigned char
#endif

#ifndef uint 
#define uint unsigned int
#endif

/**********************************
PIN口定义
**********************************/
#define LCD1602_DATAPINS P0 //连接1602的一字节数据位于P0端口
sbit LCD1602_E=P2^7;
sbit LCD1602_RW=P2^5;
sbit LCD1602_RS=P2^6;

/**********************************
函数声明
**********************************/
/*在51单片机12MHZ时钟下的延时函数*/
void Lcd1602_Delay1ms(uint c);   //误差 0us
/*LCD1602写入8位命令子函数*/
void LcdWriteCom(uchar com);
/*LCD1602写入8位数据子函数*/	
void LcdWriteData(uchar dat)	;
/*LCD1602初始化子程序*/		
void LcdInit();		

void LcdClear(); //清除LED显示
void LcdSetCursor(int x,int y); //设置坐标
void LcdPrintString(char* ch); //字符串输出
void LcdSetModifier(int number,int row); //设置自定义字符集的位号和行数
void LcdCreateCharacter(int number,char* ch); //自定义字符集
void LcdPrintLongString(char* ch,int length); //超标字符串输出
void LcdCursorMode(char i); //光标模式选择
void LcdLeftMove();//左移
void LcdRightMove();//右移
void LcdPrintInt(int i); //显示六位数以下的整数(int)
void LcdPrintLongInt(unsigned long i); //显示整数


#endif
