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

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <signal.h>
#include <chrono>


/* ROS Includes */
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"
#include "std_msgs/Float32.h"
#include "ros_gem5_topics.hh"

// For execution timing
using namespace std;
using ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;


int physics_pass, gem5_pass;

void stopsim(int sig)
{
  ros::shutdown();
}


// sets synch variable for barrier in loop in main
void cpuListenerCallback(const std_msgs::String::ConstPtr& msg)
{
    gem5_pass = 1;
//    ROS_INFO( "(ROS) CPU setting pass variable to 1.");
}


// sets synch variable for barrier in loop in main
void physicsListenerCallback(const std_msgs::String::ConstPtr& msg)
{
    physics_pass = 1;
//    ROS_INFO ("(ROS) Physics engine setting pass variable to 1.");
}


int
main(int argc, char **argv)
{
    // look into async spinner for non blocking messages
    ros::init(argc, argv, "ros_gem5_sync");
    //ros::AsyncSpinner spinner(1);

    ros::NodeHandle n;
//    std::string prog_name(argv[0]);
    std::chrono::time_point<get_time> cosim_start, cosim_end;
    std::chrono::nanoseconds cosim_diff(0);
/*
    float cosim_cycle_time = 0.020; // 20ms 
    uint64_t cpu_ticks = 20000000000; // 20ms @ 1ps per tick
*/
    float cosim_cycle_time = 0.001; // 1ms 
    uint64_t cpu_ticks = 1000000000; // 1ms @ 1ps per tick

    ros::Publisher  physics_pub, cpu_pub;
    ros::Subscriber psub, csub;
    ros::Rate poss_rate(100);
    int pub_buf_size = 1;
    int subscriber_buf_size = 1;
/*
    ros::Subscriber to_camera;
    std_msgs::String motor_msg;
    std::stringstream ss;
*/
    std_msgs::Float32 physics_cycle_msg;
    std_msgs::UInt64 cpu_ticks_msg;
    
    // register ctrl-c to shutdown node
    signal(SIGINT, stopsim);


    // register ROS subscribers and publishers
    // FIXME: motor topic should be a Twist messgage
//    to_motor = n.advertise<std_msgs::String>("motor_topic", pub_buf_size);
    // drone image has an update rate of 10 and is 320x240 in R8G8B8 format
//    to_camera = n.subscribe("/front_cam/camera/image", subscriber_buf_size , &RosGem5Interface::cameraCallback, gem5 );
    // subscribers
    psub = n.subscribe(PHYSICS_DONE_TOPIC, subscriber_buf_size , physicsListenerCallback);
    csub = n.subscribe(GEM5_CYCLE_COMPLETE_TOPIC, subscriber_buf_size , cpuListenerCallback);

    // publishers
    physics_pub = n.advertise<std_msgs::Float32>(PHYSICS_START_TOPIC, pub_buf_size);

    cpu_pub = n.advertise<std_msgs::UInt64>(GEM5_RUN_CYCLES_TOPIC, pub_buf_size);
    // publisher messages
    physics_cycle_msg.data = cosim_cycle_time; // 20 ms
    cpu_ticks_msg.data = cpu_ticks;  // 20 ms @ 1ps per tick

    // make sure publishers are connected
    while (cpu_pub.getNumSubscribers() == 0 )
    {
        poss_rate.sleep();
    }
    ROS_INFO("(ROS:Sync) CPU publisher registered.");
    while (physics_pub.getNumSubscribers() == 0)
    {
        poss_rate.sleep();
    }
    ROS_INFO("(ROS:Sync) Physics publisher registered.");
  
    // we really ought to wait until both objects
    // (gem5 and physics controller)
    // register themselves by sending a "reset" message

    
    // set physics pass to 0, gem5 pass to 0
    ROS_INFO( "Setting both pass variables to 1.");
    physics_pass = 1;
    gem5_pass = 1;

    cosim_start = get_time::now();
    while (ros::ok()) // && count <= 500)
    {
      if ((physics_pass == 1) && (gem5_pass == 1))
      {
        // do calculations to get performance stats
        cosim_end = get_time::now();
        cosim_diff = cosim_end - cosim_start;

        ROS_INFO("(ROS) Co-simulation real-time factor: %f ", 
                1e9 * cosim_cycle_time / cosim_diff.count()) ;
        std::cout << "(ROS) diff = : " << cosim_diff.count() << "ns\n"; 
        physics_pass = 0;
        gem5_pass = 0;
        cosim_start = get_time::now();
        // send message to turn on gem5 for x time slice
        cpu_pub.publish(cpu_ticks_msg);
        // send message to turn on physics for x time slice
        physics_pub.publish(physics_cycle_msg);
      }
      // spin once on message
      ros::spinOnce();
    
    } // end while

    return EXIT_SUCCESS;
}

