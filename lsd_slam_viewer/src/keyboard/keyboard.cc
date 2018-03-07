/*
 * Copyright © 2017 Uisee Co., Ltd.
 * File name： keyboard.c
 * Function  ： 按键扫描
 * Author  ： JohnnyWang
 * log   :   
 * =======2017.06.15.10:22(星期四)======= 创建文

*/

#include <stdio.h>
#include <stdlib.h>
#include <termio.h>  

#include "keyboard.h"
/*函数Function：扫描键盘按下的键值
 *参数说明：
  返回值 ：
            成功： 按键对应的ASCII码
            失败：-1
  * */
int scanKeyboard()  
{  
    int in;  
    struct termios new_settings;  
    struct termios stored_settings;  
    tcgetattr(0,&stored_settings);  
    new_settings = stored_settings;  
    new_settings.c_lflag &= (~ICANON);  
    new_settings.c_cc[VTIME] = 0;  
    tcgetattr(0,&stored_settings);  
    new_settings.c_cc[VMIN] = 1;  
    tcsetattr(0,TCSANOW,&new_settings);  
      
    in = getchar(); 

    printf("you press key num is %d",in);
      
//这个方法就可以，返回值是该键的ASCII码值，不需要回车的，  
    tcsetattr(0,TCSANOW,&stored_settings);  
    
    return in;  
}  
void keyboard_init()
{
    system(STTY_US TTY_PATH);
}


int keyboard_input()
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间

    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar(); 
    }

    return ch;
}



