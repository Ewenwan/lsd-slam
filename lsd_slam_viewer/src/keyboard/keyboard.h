
/*
 * Copyright © 2017 Uisee Co., Ltd.
 * File name： keyboard.h
 * Function  ： 头文件
 * Author  ： JohnnyWang
 * log   :   
 * =======2017.06.15.10:19(星期四)=======创建文件
 *
 */
#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

/* 备注   ：
            部分ASCII码表
            97  a
            98  b
            99  c
            100 d
            101 e
            102 f
            103 g
            104 h
            105 i
            106 j
            107 k
            108 l
            109 m
            110 n
            111 o
            112 p
            113 q
            114 r
            115 s
            116 t
            117 u
            118 v
            119 w
            120 x
            121 y
            122 z
*/

int scanKeyboard();  
void keyboard_init();
int keyboard_input();

#endif
