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
#define RIGHT 0x43 
#define LEFT 0x44
#define FORWARD 0x41
#define BACKWARD 0x42
#define STOP 0x20

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
            case LEFT:
                left_speed=100;
                right_speed=200;
                break;
            case RIGHT:
                left_speed=200;
                right_speed=100;
                break;
            case FORWARD:
                left_speed=right_speed=200;
                break;
            case BACKWARD:
                left_speed=right_speed=-200;
                break;
            case STOP:
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
    
    n.param<std::string>("roomba/port", port, "/dev/ttyUSB0");
    
    roomba = new irobot::OpenInterface(port.c_str());
        
    if( roomba->openSerialPort(true) == 0) ROS_INFO("Connected to Roomba.");
    else
    {
        ROS_FATAL("Could not connect to Roomba.");
        ROS_BREAK();
    }
    
    ROS_INFO_STREAM("Press w to move wheels forward, s to move wheels backward, space to stop");
    keyboard_handler keyboard_h;
    
    ROS_INFO("Keyboard Handler Started");
    
    signal(SIGINT,quit);
    
    keyboard_h.keyboard_reading();

    roomba->powerDown();
    roomba->closeSerialPort();
    quit();
    return 0;
}