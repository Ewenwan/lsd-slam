/*
 * Copyright © 2017 Uisee Co., Ltd.
 * File name： serial.h
 * Function  ： serial.h
 * Author  ： JohnnyWang
 * log   :   
 * =======2017.06.14(星期三)=======创建文件
 *
 */
#ifndef SERIAL_H_
#define SERIAL_H_
int open_port(int fd,int comport); 
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);
int serial_init(int baurdrate);
#endif
