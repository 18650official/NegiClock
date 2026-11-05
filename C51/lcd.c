#include "lcd.h"
int numBuffer[12];

/*******************************************************************************
* 函 数 名         : Lcd1602_Delay1ms
* 函数功能		   : 延时函数，延时1ms
* 输    入         : c
* 输    出         : 无
* 说    名         : 该函数是在12MHZ晶振下，12分频单片机的延时。
*******************************************************************************/
#ifdef DOMESTIC_TEST
void Lcd1602_Delay1ms(uint c)   //误差 0us
{
    uchar a,b;
	for (; c>0; c--)
	{
		 for (b=199;b>0;b--)
		 {
		  	for(a=1;a>0;a--);
		 }      
	}
    	
}
#else
void Lcd1602_Delay1ms(uint c)   //误差 0us
{
	delay(c);
}
#endif

/*******************************************************************************
* 函 数 名         : LcdWriteCom
* 函数功能		   : 向LCD写入一个字节的命令
* 输    入         : com
* 输    出         : 无
*******************************************************************************/
#ifndef 	LCD1602_4PINS	 //当没有定义这个LCD1602_4PINS时
void LcdWriteCom(uchar com)	  //写入命令
{
	LCD1602_E = 0;     //使能
	LCD1602_RS = 0;	   //选择发送命令
	LCD1602_RW = 0;	   //选择写入
	
	LCD1602_DATAPINS = com;     //放入命令
	Lcd1602_Delay1ms(1);		//等待数据稳定

	LCD1602_E = 1;	          //写入时序
	Lcd1602_Delay1ms(2);	  //保持时间
	LCD1602_E = 0;
}
#else 
void LcdWriteCom(uchar com)	  //写入命令
{
	LCD1602_E = 0;	 //使能清零
	LCD1602_RS = 0;	 //选择写入命令
	LCD1602_RW = 0;	 //选择写入

	LCD1602_DATAPINS = com;	//由于4位的接线是接到P0口的高四位，所以传送高四位不用改
	Lcd1602_Delay1ms(1);

	LCD1602_E = 1;	 //写入时序
	Lcd1602_Delay1ms(2);
	LCD1602_E = 0;

	LCD1602_DATAPINS = com << 4; //发送低四位
	Lcd1602_Delay1ms(1);

	LCD1602_E = 1;	 //写入时序
	Lcd1602_Delay1ms(2);
	LCD1602_E = 0;
}
#endif
/*******************************************************************************
* 函 数 名         : LcdWriteData
* 函数功能		   : 向LCD写入一个字节的数据
* 输    入         : dat
* 输    出         : 无
*******************************************************************************/		   
#ifndef 	LCD1602_4PINS		   
void LcdWriteData(uchar dat)			//写入数据
{
	LCD1602_E = 0;	//使能清零
	LCD1602_RS = 1;	//选择输入数据
	LCD1602_RW = 0;	//选择写入

	LCD1602_DATAPINS = dat; //写入数据
	Lcd1602_Delay1ms(1);

	LCD1602_E = 1;   //写入时序
	Lcd1602_Delay1ms(2);   //保持时间
	LCD1602_E = 0;
}
#else
void LcdWriteData(uchar dat)			//写入数据
{
	LCD1602_E = 0;	  //使能清零
	LCD1602_RS = 1;	  //选择写入数据
	LCD1602_RW = 0;	  //选择写入

	LCD1602_DATAPINS = dat;	//由于4位的接线是接到P0口的高四位，所以传送高四位不用改
	Lcd1602_Delay1ms(1);

	LCD1602_E = 1;	  //写入时序
	Lcd1602_Delay1ms(1);
	LCD1602_E = 0;

	LCD1602_DATAPINS = dat << 4; //写入低四位
	Lcd1602_Delay1ms(1);

	LCD1602_E = 1;	  //写入时序
	Lcd1602_Delay1ms(2);
	LCD1602_E = 0;
}
#endif
/*******************************************************************************
* 函 数 名       : LcdInit()
* 函数功能		 : 初始化LCD屏
* 输    入       : 无
* 输    出       : 无
*******************************************************************************/		   
#ifndef		LCD1602_4PINS
void LcdInit()						  //LCD初始化子程序
{
 	LcdWriteCom(0x38);  //开显示
	LcdWriteCom(0x0c);  //开显示不显示光标
	LcdWriteCom(0x06);  //写一个指针加1
	LcdWriteCom(0x01);  //清屏
	LcdWriteCom(0x80);  //设置数据指针起点
}
#else
void LcdInit()						  //LCD初始化子程序
{
	LcdWriteCom(0x32);	 //将8位总线转为4位总线
	LcdWriteCom(0x28);	 //在四位线下的初始化
	LcdWriteCom(0x0c);  //开显示不显示光标
	LcdWriteCom(0x06);  //写一个指针加1
	LcdWriteCom(0x01);  //清屏
	LcdWriteCom(0x80);  //设置数据指针起点
}
#endif

void LcdClear() //清除LED显示
{
	LcdWriteCom(0x01);
}

void LcdSetCursor(int x,int y) //设置坐标
{
	int addr;
	if(y == 0) addr = 0x80 + x;
	else if(y == 1) addr = 0xC0 + x; //创建光标的起始地址
	LcdWriteCom(addr); //DB7设置为1，其余7位作为指令写入DDRAM分区
}

void LcdPrintString(char* ch) //字符串输出
{
	int i;
	for(i=0;;i++)
	{
		if(!ch[i]) break;
		LcdWriteData(ch[i]);
	}
}

void LcdPrintLongString(char* ch,int length) //字符串输出
{
	int i;
	for(i=0;i<length;i++)
	{
      //if(!ch[i]) break;
		LcdWriteData(ch[i]);
	}
}

void LcdSetModifier(int number,int row) //设置自定义字符集的位号和行数
{
	int command = 0;
	command |= 0x40; //声明写CGRAM
	command += 8 * number + row; //补充序号和行号
	LcdWriteCom(command); //推送数据
}

void LcdCreateCharacter(int number,char* ch) //自定义字符集
{
	int i;
	for(i=0;i<8;i++)
	{
		LcdSetModifier(number,i);
		LcdWriteData(ch[i]); //含有8个二进制数的数组，每次写入1行
	}
}

void LcdCursorMode(char i) //显示控制：0000 11CB
{
	unsigned char command;
	switch(i)
	{
		case 1: //无光标不闪烁
		{
			command = 0x0C;
			break;
		}
		case 2: //有光标不闪烁
		{
			command = 0x0E;
			break;
		}
		case 3: //闪烁光标
		{
			command = 0x0F;
			break;
		}
	}
	LcdWriteCom(command); //写指令
}

void LcdRightMove()
{
	LcdWriteCom(0x1F); //右移屏幕
}

void LcdLeftMove()
{
	LcdWriteCom(0x18); //左移全屏
}

void LcdPrintInt(int i) //显示六位数以下的整数(int)
{
	int j = 0;
	while(i != 0)
	{
		numBuffer[j] = i % 10; //取出末尾
		j++;
		i /= 10;//倒退1位
	}
	if(j != 5)
	for(;j<6;j++)
	{
		numBuffer[j] = 0xA0; //填充特殊字符
	}
	for(j=5;j>=0;j--)
	{
		if(numBuffer[j] != 0xA0)
		{
			LcdWriteData('0' + numBuffer[j]); //倒序输出
		}
	}
}

void LcdPrintLongInt(unsigned long i) //显示整数
{
	int j = 0;
	while(i != 0)
	{
		numBuffer[j] = i % 10; //取出末尾
		j++;
		i /= 10;//倒退1位
	}
	if(j != 5)
	for(;j<12;j++)
	{
		numBuffer[j] = 0xA0; //填充特殊字符
	}
	for(j=11;j>=0;j--)
	{
		if(numBuffer[j] != 0xA0)
		{
			LcdWriteData('0' + numBuffer[j]); //倒序输出
		}
	}
}