/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andrew Bardsley
 */

/**
 * @file
 *
 *  C++-only configuration and instantiation support.  This allows a
 *  config to be read back from a .ini and instantiated without
 *  Python.  Useful if you want to embed gem5 within a larger system
 *  without carrying the integration cost of the fully-featured
 *  configuration system.
 *
 *  This file contains a demonstration main using CxxConfigManager.
 *  Build with something like:
 *
 *      scons --without-python build/ARM/libgem5_opt.so
 *
 *      g++ -DTRACING_ON -std=c++0x -Ibuild/ARM src/sim/cxx_main.cc \
 *          -o gem5cxx.opt -Lbuild/ARM -lgem5_opt
 */

/* 
use getEventQueue(0)->schedule(Event *event, Tick when, bool global=false) to put items in the event queue
when can be specified as getEventQueue(0)->getCurTick();
*/

#ifndef __PHYSICS_CTRL_NODE_H__
#define __PHYSICS_CTRL_NODE_H__


#include <cstdlib>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <signal.h>


/* ROS Includes */
#include <ros/ros.h>
#include "std_msgs/UInt64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"


// Set the topic name 
#include "ros_gem5_topics.hh"
/*
const std::string PHYSICS_START_TOPIC = "/gem5/start_physics";
const std::string PHYSICS_DONE_TOPIC = "/gem5/physics_done";
const std::string GAZEBO_PHYSICS_PAUSE_TOPIC = "/gazebo/pause_physics";
const std::string GAZEBO_PHYSICS_UNPAUSE_TOPIC = "/gazebo/unpause_physics";
*/


class PhysicsCtrlNode{
  public:
   
    PhysicsCtrlNode();//    { }
    ~PhysicsCtrlNode();//   { }
    static void usage(const std::string &prog_name);
    //void setMyDebugFlag (const std::string &debug_flag);
    //Tick getClockFrequency ();
    void startPhysicsCallback(const std_msgs::Float32::ConstPtr& message);
//    void finishPhysicsCallback(const std_msgs::UInt64::ConstPtr& message);
//    void gem5PubCallback(const std_msgs::StringConstPtr& message);
//    void cameraCallback(const std_msgs::StringConstPtr& message);
/*
	void loadConfigFile (CxxConfigFileBase *conf);
*/
	
  private:
    ros::NodeHandle _nh;
    ros::Subscriber start_sim;
    ros::Publisher finish_physics_cycle;
    ros::ServiceClient pause_physics;
    ros::ServiceClient unpause_physics;
//    int subscriber_buf_size = 1000;
    int subscriber_buf_size = 1;

//  std::string config_file;

}; // end PhysicsCtrlNode 


#endif // __PHYSICS_CTRL_NODE_H__
