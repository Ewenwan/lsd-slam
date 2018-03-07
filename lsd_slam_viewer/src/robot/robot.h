/*
 * Copyright © 2017 Uisee Co., Ltd.
 * File name： control.h
 * Function ： robot class
 * Author   ： JohnnyWang
 * log   :   
 * 
 */
#ifndef CONTROL_H_
#define CONTROL_H_

#include<iostream>
#include <stdio.h> 
#include <string.h> 
#include <sys/types.h> 
#include <errno.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include <unistd.h> 
#include <termios.h> 
#include <stdlib.h> 
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../serial/serial.h"
#include "../keyboard/keyboard.h"

using std::string;
using namespace std;

#define SLAM_CAR_RADIUS 0.50d
#define CARB_CAR_RADIUS 1.00d

template <typename wheel_velocity_type>
class wheel_robot
{
/*
    Motion model
    
    robot_pose --   V_X   -- 
               |           |
               |    V_Y    |
               |           |
               --  omega  --

*/
public:
    wheel_robot() = default;
    explicit wheel_robot(Eigen::Vector3d velocity,const string chassis_model)
    :robot_velocity(velocity),chassis_control_model(chassis_model)
    {
        if(chassis_model == "UART")
        {
            serial_fd = serial_init(115200);
            //cout << serial_fd << endl;
        }
        if(chassis_model == "CAN")
        {
            cout << "NOT support CAN INTERFACE now." << endl;
            return;
        }
    };
    virtual wheel_velocity_type wheel_velocity_decode() {};
    virtual int robot_move() {};
    virtual ~wheel_robot() = default;
    
public:
    Eigen::Vector3d      robot_velocity;
    wheel_velocity_type  dewheel_velocity;

//  Chassis control   
public:
    void remote_control_init(){keyboard_init();};
    virtual int remote_control();
    
    
private:
    string chassis_control_model;
    

protected:
    double speed_temp = 0.0;
    int    wantgo = 0;
    int    serial_fd   = 0; 
    unsigned char instruction_message[8]={0x53,0x7f,0xff,0x7f,0xff,0x7f,0xff,0x45};
  //unsigned char instruction_message[8]={0x53,0x7f,0xff,0x7f,0xff,0x7f,0xff,0x45};
  //unsigned char instruction_message[8]={0x53,0x81,0x8f,0x81,0x8f,0x7f,0xff,0x45};
  /*
            53 81 8f 7f ff 7f ff 45 
            53 7f ff 81 8f 7f ff 45 
            53 7f ff 7f ff 81 8f 45 
            一帧数据的结构  八个字节

		帧头   一号轮速度            二号轮速度        三号轮速度       帧尾
		0x53  0xFF    0xFF     0xFF    0xFF      0xFF 0xFF        0x45
			  高位    低位
对应CAN消息 Data[0] Data[1]    Data[2] Data[3]   Data[4] Data[5]
   */
  

};

class three_omni_wheel_robot : public wheel_robot<Eigen::Vector3d>
{
    
/* 
 *                                                    Y
    velocity   --   V_1   --             V1     V2    /|\  
               |           |               -   -       |
               |    V_2    |                - -        |        
               |           |                 |         |_________\ X
               --   V_3   --                 V3       o          / 

*/

public:
    three_omni_wheel_robot() = default;
    explicit three_omni_wheel_robot( Eigen::Vector3d velocity ,const string chassis_model) 
    : wheel_robot(velocity,chassis_model)
    {
       solve_convert <<   -0.5d , -0.866d , SLAM_CAR_RADIUS ,
                          -0.5d ,  0.866d , SLAM_CAR_RADIUS ,
                           1.0d ,  0.0d   , SLAM_CAR_RADIUS ;
    };
    virtual Eigen::Vector3d wheel_velocity_decode() override;//can't be const
    virtual int robot_move()   override ;
    virtual ~three_omni_wheel_robot() = default;

private:
    Eigen::Matrix3d  solve_convert;
  
};

class double_wheel_robot : public wheel_robot<Eigen::Vector2d>
{
/* 
                                                       Y
    velocity   --   V_1   --                          /|\  
               |           |                           |
               |           |                           |        
               |           |                           |_________\ X
               --   V_2   --      V1-------V2         o          / 

*/
public:
    double_wheel_robot() = default;
    explicit double_wheel_robot( Eigen::Vector3d velocity ,const string chassis_model) 
    : wheel_robot(velocity,chassis_model)
    {
       solve_convert <<   0.5d ,  0.0d , -CARB_CAR_RADIUS ,
                          0.5d ,  0.0d ,  CARB_CAR_RADIUS ;
    };
    virtual Eigen::Vector2d wheel_velocity_decode() override; //can't be const
    virtual int robot_move()   override ;
    virtual ~double_wheel_robot() = default;

private:
    Eigen::Matrix<double,2,3>  solve_convert;

};
#endif
