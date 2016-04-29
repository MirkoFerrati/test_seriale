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
* Author: Gonçalo Cabrita on 19/05/2010
*********************************************************************/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include "../include/test_seriale/OpenInterface.h"

// *****************************************************************************
// Constructor
irobot::OpenInterface::OpenInterface(const char * new_serial_port)//:new_odometry_()
{	
	port_name_ = new_serial_port;

	OImode_ = OI_MODE_OFF;
	
	
	encoder_counts_[LEFT] = -1;
	encoder_counts_[RIGHT] = -1;
	
	last_encoder_counts_[LEFT] = 0;
	last_encoder_counts_[RIGHT] = 0;
	
	num_of_packets_ = 0;
	sensor_packets_ = NULL;
	packets_size_ = 0;
    
	// Default packets
	OI_Packet_ID default_packets[2] = {OI_PACKET_RIGHT_ENCODER, OI_PACKET_LEFT_ENCODER};
	this->setSensorPackets(default_packets, 2, OI_PACKET_RIGHT_ENCODER_SIZE + OI_PACKET_LEFT_ENCODER_SIZE);
}


// *****************************************************************************
// Destructor
irobot::OpenInterface::~OpenInterface()
{
	// Clean up!
}


// *****************************************************************************
// Open the serial port
int irobot::OpenInterface::openSerialPort(bool full_control)
{
	try
	{ 
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(115200);
        serial_port_.open();
    }
	catch(std::exception& e){
        std::cerr<<"Error opening serial port: "<<e.what()<<std::endl;
        return(-1); 
    }

// 	int result = this->startOI(full_control);
    
	return(0);
}


// *****************************************************************************
// Set the mode
int irobot::OpenInterface::startOI(bool full_control)
{	
	uint8_t buffer[1];

	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
	try{ 
        Start();}
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	OImode_ = OI_MODE_PASSIVE;

	usleep(OI_DELAY_MODECHANGE_MS * 1e3);
    try{ 
        Safe();}
    catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	OImode_ = OI_MODE_SAFE;
	
	if(full_control)
	{
		usleep(OI_DELAY_MODECHANGE_MS * 1e3);
        try{ 
            Full();}
        catch(std::exception& e){
            std::cerr<<e.what()<<std::endl;
            return(-1);}
		OImode_ = OI_MODE_FULL;
	}
	return(0);
}


// *****************************************************************************
// Close the serial port
int irobot::OpenInterface::closeSerialPort()
{
	this->drive(0.0, 0.0);
	usleep(OI_DELAY_MODECHANGE_MS * 1e3);

    try{ 
        serial_port_.close();}
    catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	return(0);
}


// *****************************************************************************
// Send an OP code to the roomba
int irobot::OpenInterface::sendOpcode(OI_Opcode code)
{
	uint8_t to_send = code;
	std::cout<<"Sending OPCode: "<<(uint8_t)to_send<<std::endl;
	try{ serial_port_.write(&to_send, 1); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	return(0);
}


// *****************************************************************************
// Start the roomba
int irobot::OpenInterface::Start()
{
	return sendOpcode(OI_OPCODE_START);
}

// *****************************************************************************
// Stop the roomba
int irobot::OpenInterface::Stop()
{
	return sendOpcode(OI_OPCODE_STOP);
}

// *****************************************************************************
// Reset the roomba
int irobot::OpenInterface::Reset()
{
	return sendOpcode(OI_OPCODE_RESET);
}

// *****************************************************************************
// Set Safe mode for the roomba
int irobot::OpenInterface::Safe()
{
	int ret = sendOpcode(OI_OPCODE_SAFE);

// 	setDigitLeds(83,65,70,69);
// 	setLeds(0, 0, 0, 1, 1, 1);

	return ret;
}

// *****************************************************************************
// Set Full mode for the roomba
int irobot::OpenInterface::Full()
{
	int ret = sendOpcode(OI_OPCODE_FULL);

// 	setDigitLeds(70,85,76,76);
// 	setLeds(0, 0, 0, 1, 1, 1);

	return ret;
}

// *****************************************************************************
// Power down the roomba
int irobot::OpenInterface::powerDown()
{
	return sendOpcode(OI_OPCODE_POWER);
}


// *****************************************************************************
// Set the speeds/**/
int irobot::OpenInterface::drive(double linear_speed, double angular_speed)
{
	int left_speed_mm_s = (int)((linear_speed-ROOMBA_AXLE_LENGTH*angular_speed/2)*1e3);		// Left wheel velocity in mm/s
	int right_speed_mm_s = (int)((linear_speed+ROOMBA_AXLE_LENGTH*angular_speed/2)*1e3);	// Right wheel velocity in mm/s
	
	return this->driveDirect(left_speed_mm_s, right_speed_mm_s);
}/**/


// *****************************************************************************
// Set the motor speeds
int irobot::OpenInterface::driveDirect(int left_speed, int right_speed)
{
	// Limit velocity
	int16_t left_speed_mm_s = std::max(left_speed, -ROOMBA_MAX_LIN_VEL_MM_S);
	left_speed_mm_s = std::min(left_speed, ROOMBA_MAX_LIN_VEL_MM_S);
	int16_t right_speed_mm_s = std::max(right_speed, -ROOMBA_MAX_LIN_VEL_MM_S);
	right_speed_mm_s = std::min(right_speed, ROOMBA_MAX_LIN_VEL_MM_S);
	
	// Compose comand
	uint8_t cmd_buffer[5];
	cmd_buffer[0] = OI_OPCODE_DRIVE_DIRECT;
	cmd_buffer[1] = (right_speed_mm_s >> 8);
	cmd_buffer[2] = (right_speed_mm_s & 0xFF);
	cmd_buffer[3] = (left_speed_mm_s >> 8);
	cmd_buffer[4] = (left_speed_mm_s & 0xFF);

	try{ serial_port_.write(cmd_buffer, 5); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
	return(0);
}


// *****************************************************************************
// Set the sensors to read
int irobot::OpenInterface::setSensorPackets(OI_Packet_ID * new_sensor_packets, int new_num_of_packets, size_t new_buffer_size)
{
	if(sensor_packets_ == NULL)
	{
		delete [] sensor_packets_;
	}
	
	num_of_packets_ = new_num_of_packets;
	sensor_packets_ = new OI_Packet_ID[num_of_packets_];
	
	for(int i=0 ; i<num_of_packets_ ; i++)
	{
		sensor_packets_[i] = new_sensor_packets[i];
	}

	stream_defined_ = false;
	packets_size_ = new_buffer_size;
	return(0);
}


// *****************************************************************************
// Read the sensors
int irobot::OpenInterface::getSensorPackets(int timeout)
{
	uint8_t cmd_buffer[num_of_packets_+2];
	uint8_t data_buffer[packets_size_];

	// Fill in the command buffer to send to the robot
	cmd_buffer[0] = OI_OPCODE_QUERY;			// Query
	cmd_buffer[1] = num_of_packets_;				// Number of packets
	for(int i=0 ; i<num_of_packets_ ; i++)
	{
		cmd_buffer[i+2] = sensor_packets_[i];		// The packet IDs
	}

	try{ serial_port_.write(cmd_buffer, num_of_packets_+2); }
	catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}	
	try{ 
        serial::Timeout T = serial::Timeout::simpleTimeout(timeout);
        serial_port_.setTimeout(T);
        bool readable=serial_port_.waitReadable();
        if (readable)
            serial_port_.read(data_buffer, packets_size_); 
        else return(-1);
    }
    catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        return(-1);}
        
	return this->parseSensorPackets(/*(unsigned char*)*/data_buffer, packets_size_);
}




// *****************************************************************************
// Parse sensor data
int irobot::OpenInterface::parseSensorPackets(unsigned char * buffer , size_t buffer_lenght)
{	
	if(buffer_lenght != packets_size_)
	{
		// Error wrong packet size
		return(-1);
	}

	int i = 0;
	unsigned int index = 0;
        unsigned int temp_index = 0;
	while(index < packets_size_)
	{
		if(sensor_packets_[i]==OI_PACKET_VOLTAGE)
		{
			index += parseVoltage(buffer, index);
			i++;
		}
		if(sensor_packets_[i]==OI_PACKET_CURRENT)
		{
			index += parseCurrent(buffer, index);
			i++;
		}
		if(sensor_packets_[i]==OI_PACKET_TEMPERATURE)
		{
			index += parseTemperature(buffer, index);
			i++;
		}
		if(sensor_packets_[i]==OI_PACKET_BATTERY_CHARGE)
		{
			index += parseBatteryCharge(buffer, index);
			i++;
		}
		if(sensor_packets_[i]==OI_PACKET_BATTERY_CAPACITY)
		{
			index += parseBatteryCapacity(buffer, index);
			i++;
                }
		if(sensor_packets_[i]==OI_PACKET_OI_MODE)
		{
			index += parseOiMode(buffer, index);
			i++;
                }
		if(sensor_packets_[i]==OI_PACKET_RIGHT_ENCODER)
		{
			index += parseRightEncoderCounts(buffer, index);
			i++;
		}
		if(sensor_packets_[i]==OI_PACKET_LEFT_ENCODER)
		{
			index += parseLeftEncoderCounts(buffer, index);
			i++;
                }
		if(sensor_packets_[i]==OI_PACKET_GROUP_100)	// PACKETS 7-58
		{
                    temp_index=index;
                   
			index +=17;
			index += parseVoltage(buffer, index);2;
			index += parseCurrent(buffer, index);2;
			index += parseTemperature(buffer, index);1;
			index += parseBatteryCharge(buffer, index);2;
                        index += parseBatteryCapacity(buffer, index);2;
			index += 14;
			index += parseOiMode(buffer, index);1;
                        index += 11;
			index += parseRightEncoderCounts(buffer, index);2;
                        index += parseLeftEncoderCounts(buffer, index);2;
			i++;   
                        index=temp_index+80;
                }
	}
	return(0);
}

int irobot::OpenInterface::parseVoltage(unsigned char * buffer, int index)
{
	// Voltage
	this->voltage_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_VOLTAGE_SIZE;
}

int irobot::OpenInterface::parseCurrent(unsigned char * buffer, int index)
{
	// Current
	this->current_ = (float)(buffer2signed_int(buffer, index) / 1000.0);

	return OI_PACKET_CURRENT_SIZE;
}

int irobot::OpenInterface::parseTemperature(unsigned char * buffer, int index)
{
	// Temperature
	this->temperature_ = (char)(buffer[index]);

	return OI_PACKET_TEMPERATURE_SIZE;
}

int irobot::OpenInterface::parseBatteryCharge(unsigned char * buffer, int index)
{
	// Charge
	this->charge_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_BATTERY_CHARGE_SIZE;
}

int irobot::OpenInterface::parseBatteryCapacity(unsigned char * buffer, int index)
{
	// Capacity
	this->capacity_ = (float)(buffer2unsigned_int(buffer, index) / 1000.0);

	return OI_PACKET_BATTERY_CAPACITY_SIZE;
}


int irobot::OpenInterface::parseOiMode(unsigned char * buffer, int index)
{
	this->OImode_ = buffer[index];

	return OI_PACKET_OI_MODE_SIZE;
}


int irobot::OpenInterface::parseRightEncoderCounts(unsigned char * buffer, int index)
{
	// Right encoder counts
	uint16_t right_encoder_counts = buffer2unsigned_int(buffer, index);

        printf("Right Encoder: %d\n", right_encoder_counts);

	if(encoder_counts_[RIGHT] == -1 || right_encoder_counts == last_encoder_counts_[RIGHT])	// First time, we need 2 to make it work!
	{
		encoder_counts_[RIGHT] = 0;
	}
	else
	{
		encoder_counts_[RIGHT] = (int)(right_encoder_counts - last_encoder_counts_[RIGHT]);
		
		if(encoder_counts_[RIGHT] > ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[RIGHT] = encoder_counts_[RIGHT] - ROOMBA_MAX_ENCODER_COUNTS;
		if(encoder_counts_[RIGHT] < -ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[RIGHT] = ROOMBA_MAX_ENCODER_COUNTS + encoder_counts_[RIGHT];
	}
	last_encoder_counts_[RIGHT] = right_encoder_counts;
	
	return OI_PACKET_RIGHT_ENCODER_SIZE;
}

int irobot::OpenInterface::parseLeftEncoderCounts(unsigned char * buffer, int index)
{
	// Left encoder counts
	uint16_t left_encoder_counts = buffer2unsigned_int(buffer, index);

        printf("Left Encoder: %d\n", left_encoder_counts);

	if(encoder_counts_[LEFT] == -1 || left_encoder_counts == last_encoder_counts_[LEFT])	// First time, we need 2 to make it work!
	{
		encoder_counts_[LEFT] = 0;
	}
	else
	{
		encoder_counts_[LEFT] = (int)(left_encoder_counts - last_encoder_counts_[LEFT]);
		
		if(encoder_counts_[LEFT] > ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[LEFT] = encoder_counts_[LEFT] - ROOMBA_MAX_ENCODER_COUNTS;
		if(encoder_counts_[LEFT] < -ROOMBA_MAX_ENCODER_COUNTS/10) encoder_counts_[LEFT] = ROOMBA_MAX_ENCODER_COUNTS + encoder_counts_[LEFT];
	}
	last_encoder_counts_[LEFT] = left_encoder_counts;
	
	return OI_PACKET_LEFT_ENCODER_SIZE;
}

int irobot::OpenInterface::buffer2signed_int(unsigned char * buffer, int index)
{
	int16_t signed_int;
	
	memcpy(&signed_int, buffer+index, 2);
	signed_int = ntohs(signed_int);
	
	return (int)signed_int;
}

int irobot::OpenInterface::buffer2unsigned_int(unsigned char * buffer, int index)
{
	uint16_t unsigned_int;

	memcpy(&unsigned_int, buffer+index, 2);
	unsigned_int = ntohs(unsigned_int);
	
	return (int)unsigned_int;
}

// EOF
