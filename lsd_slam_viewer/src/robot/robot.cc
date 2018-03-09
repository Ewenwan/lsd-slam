/*
 * Copyright © 2017 Uisee Co., Ltd.
 * File name： robot.cc
 * Function ： robot class
 * Author   ： JohnnyWang
 * log   :   
 * 
 *
 */
#include "robot.h"


template <typename wheel_velocity_type>
int wheel_robot<wheel_velocity_type>::remote_control()
{ 
    int ch = keyboard_input();
    
    //std::cout << "ch=" <<ch << std::endl;
    
    if(wantgo == 0)
    {
        robot_velocity << 0, 0, 0;
    }
    
    wantgo--;
    
    if(wantgo < 0)
        wantgo = 0;
    else if(wantgo > 40)
        wantgo = 40;
    
    
    
    if (ch)
    {

        switch (ch)
        {
            case 'b':   
            { 
                printf("break\n\r");
                system(STTY_DEF TTY_PATH);
                return 0;
                
            }
            case 'w':  
            {
                wantgo+=40;
                if( wantgo>=40 )
                {
                    robot_velocity << 0,speed_temp,0;
                }
                //std::cout << "wantgo = "<< wantgo << std::endl;
                //std::cout << "con->linear_velocity = "<< con->linear_velocity << std::endl;
                //std::cout << "con->angular_velocity = "<< con->angular_velocity << std::endl;
                break;
            }
            case 'a':
            {
                wantgo+=40;
                if( wantgo>=40 )
                {
                    robot_velocity << -speed_temp,0,0;
                }
                break;
            }
            case 's':
            {
                wantgo+=40;
                if( wantgo>=40 )
                {
                    robot_velocity << 0,-speed_temp,0;
                }
                break;
            }
            case 'd':
            {
                wantgo+=40;
                if( wantgo>=40 )
                {
                    robot_velocity << speed_temp,0,0;
                }
                break;
            }
            case 'q':
            {
                wantgo+=40;
                if( wantgo>=40 )
                {
                    robot_velocity << 0,0,speed_temp;
                }
                break;
            }
            case 'e':
            {
                wantgo+=40;
                if( wantgo>=40 )
                {
                    robot_velocity << 0,0,-speed_temp;
                }
                break;
            }
            case '0':
            {
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                speed_temp  = 0;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '1':
            {
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                speed_temp  = 175;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '2':
            {
                speed_temp  = 200;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '3':
            {
                speed_temp  = 300;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '4':
            {
                speed_temp  = 400;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '5':
            {
                speed_temp  = 500;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '6':
            {
                speed_temp  = 600;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '7':
            {
                speed_temp  = 700;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '8':
            {
                speed_temp  = 800;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
            case '9':
            {
                speed_temp  = 900;
                //std::cout << "speed_temp = "<< speed_temp << std::endl;
                break;
            }
        }
    } 
    wheel_velocity_decode();
    robot_move();
    
}


Eigen::Vector3d three_omni_wheel_robot::wheel_velocity_decode()
{ 
    dewheel_velocity  = solve_convert * robot_velocity;
    return dewheel_velocity;
}

Eigen::Vector2d double_wheel_robot::wheel_velocity_decode()
{ 
    dewheel_velocity  = solve_convert * robot_velocity;
    return dewheel_velocity;
}

int three_omni_wheel_robot::robot_move()
{
    Eigen::Vector3i dwv= dewheel_velocity.cast<int>();
    
    for(int i = 0; i < 3 ; i++)
    {
        dwv(i) = dwv(i) + 32767;
        instruction_message[i*2+1] = (dwv(i) & 0x0000ff00) >> 8;
        instruction_message[i*2+2] = (dwv(i) & 0x000000ff);
        
        if(instruction_message[i*2+1] == 0x53)
            instruction_message[i*2+1] = 0x54;                                                                                             
        if(instruction_message[i*2+1] == 0x45)
            instruction_message[i*2+1] = 0x46;
      
        if(instruction_message[i*2+2] == 0x53)
            instruction_message[i*2+2] = 0x54;
        if(instruction_message[i*2+2] == 0x45)
            instruction_message[i*2+2] = 0x46;

    }
     if(write(serial_fd,instruction_message,sizeof(instruction_message))<0)
    {
        cout << "Send error" << endl;
        return 0;
    }
    
}

int double_wheel_robot::robot_move()
{
    Eigen::Vector2i dwv= dewheel_velocity.cast<int>();
    
    for(int i = 0; i < 2 ; i++)
    {
        dwv(i) = dwv(i) + 32767;
        instruction_message[i*2+1] = (dwv(i) & 0x0000ff00) >> 8;
        instruction_message[i*2+2] = (dwv(i) & 0x000000ff);
    }
     if(write(serial_fd,instruction_message,sizeof(instruction_message))<0)
    {
        cout << "Send error" << endl;
        return 0;
    }
    
}


















