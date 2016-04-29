/*********************************************************************
 * 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Pisa
 *  Copyright (c) 2010, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alessandro Settimi 2015
 * Author: Mirko Ferrati 2015
 * Author: Gonçalo Cabrita on 07/10/2010
 *********************************************************************/


#include "../include/test_seriale/OpenInterface.h"
#include <ros/init.h>

#define NODE_VERSION 2.01

#include <ros/ros.h>
#include <string>

std::string port;
irobot::OpenInterface * roomba;
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#define KRIGHT 0x43 
#define KLEFT 0x44
#define KFORWARD 0x41
#define KBACKWARD 0x42
#define KSTOP 0x20

class keyboard_handler
{
public:
    keyboard_handler();
    void keyboard_reading();
    ~keyboard_handler();
};

int kfd = 0;
struct termios cooked, raw;

keyboard_handler::keyboard_handler()
{
}

void keyboard_handler::keyboard_reading()
{
    char c;
    
    // get the console in raw mode                                                              
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file                         
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    
    ROS_INFO("Use arrow keys to move the robot.");
    
    while(ros::ok())
    {
        if( roomba->getSensorPackets(100) == -1) ROS_ERROR("Could not retrieve sensor packets.");
        std::cout<<roomba->getLeftEncoderCount()<<" "<<roomba->getRightEncoderCount()<<std::endl;
        if(read(kfd, &c, 1) < 0)
        {
            ROS_ERROR("Something wrong!");
            exit(-1);
        }
        int left_speed;  //WARNING  mm/s
        int right_speed; //WARNING  mm/s
        switch(c)
        {
            case KLEFT:
                left_speed=100;
                right_speed=200;
                break;
            case KRIGHT:
                left_speed=200;
                right_speed=100;
                break;
            case KFORWARD:
                left_speed=right_speed=200;
                break;
            case KBACKWARD:
                left_speed=right_speed=-200;
                break;
            case KSTOP:
                left_speed=right_speed=0;
                break;
        }        
        roomba->driveDirect(left_speed,right_speed);
        usleep(100);
    }    
}

keyboard_handler::~keyboard_handler()
{
    
}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    std::cout<<"quitting"<<std::endl;
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roomba560_light_node");
    
    ROS_INFO("Roomba for ROS %.2f", NODE_VERSION);
    
    double last_x, last_y, last_yaw;
    double vel_x, vel_y, vel_yaw;
    double dt;
    
    ros::NodeHandle n;
    
    n.param<std::string>("port", port, "/dev/ttyUSB0");
    
    roomba = new irobot::OpenInterface(port.c_str());
    irobot::OI_Packet_ID sensor_packets[1] = {irobot::OI_PACKET_GROUP_100};
    roomba->setSensorPackets(sensor_packets, 1, OI_PACKET_GROUP_100_SIZE);
    
    if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
    else
    {
        ROS_FATAL("Could not connect to Roomba.");
        ROS_BREAK();
    }
    while(roomba->getSensorPackets(100) == -1 && ros::ok())
    {
        usleep(100);
        std::cout<<"Waiting for roomba sensors"<<std::endl;
    }
//     sleep(1);
    std::cout<<"Press w to move wheels forward, s to move wheels backward, space to stop"<<std::endl;
    keyboard_handler keyboard_h;
    bool first_loop=true;
//     while(ros::ok())
//     {
// 
//         if( roomba->getSensorPackets(100) == -1) ROS_ERROR("Could not retrieve sensor packets.");
//         std::cout<<roomba->getLeftEncoderCount()<<" "<<roomba->getRightEncoderCount()<<std::endl;
//         
//         if(first_loop)
//         {
            roomba->startOI(true);
            roomba->Full();
//             first_loop=false;
//         }
// //         roomba->driveDirect(100,100);
//         usleep(100);
//     }
    ROS_INFO("Keyboard Handler Started");
    
    signal(SIGINT,quit);
    
    keyboard_h.keyboard_reading();

    roomba->powerDown();
    roomba->closeSerialPort();
    quit(0);
    return 0;
}