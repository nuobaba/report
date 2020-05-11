
#include<reg52.h>  	       //库文件
#define uchar unsigned char//宏定义无符号字符型
#define uint unsigned int  //宏定义无符号整型

sbit LED=P1^0;	 //定义单片机P1口的第1位 （即P1.0）为指示端
sbit DOUT=P2^0;	 //定义单片机P2口的第1位 （即P2.0）为传感器的输入端

void delay()//延时程序
{
uchar m,n,s;
for(m=20;m>0;m--)
for(n=20;n>0;n--)
for(s=248;s>0;s--);
}

void main()
{
	while(1)   //无限循环
	{
	 LED=1;	   //熄灭P1.0口灯
	 if(DOUT==0)//高于设定值时 ，执行条件函数
	 {
    	delay();//延时抗干扰
		if(DOUT==0)//高于设定值时 ，执行条件函数
	    {
		 LED=0;	   //点亮P1.0口灯
		}
	 }
	}
}	
