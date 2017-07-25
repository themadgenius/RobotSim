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

#include "physics_ctrl_node.hh"

PhysicsCtrlNode::PhysicsCtrlNode()
{
     std_srvs::Empty empty_packet;

     start_sim = _nh.subscribe(PHYSICS_START_TOPIC, 1, &PhysicsCtrlNode::startPhysicsCallback, this);

     finish_physics_cycle = _nh.advertise<std_msgs::String>(PHYSICS_DONE_TOPIC, 1);

     pause_physics = _nh.serviceClient<std_srvs::Empty>(GAZEBO_PHYSICS_PAUSE_TOPIC,true);

     unpause_physics = _nh.serviceClient<std_srvs::Empty>(GAZEBO_PHYSICS_UNPAUSE_TOPIC,true);

//   pause_physics.call(empty_packet);
}


PhysicsCtrlNode::~PhysicsCtrlNode()
{
}


void PhysicsCtrlNode::startPhysicsCallback(const std_msgs::Float32::ConstPtr& msg)
{
   double cycle, sleep_secs;
   ros::Time cycle_end, cycle_start;
   ros::Duration cycle_diff;
   std_srvs::Empty empty_packet;
   std_msgs::String ss;

   cycle = msg->data;
//   std::cout << "(ROS:Physics) cycle time is " << cycle << " s\n";
   ss.data = "Chris is cool\n";
   sleep_secs = 0.010; // 10 ms

   cycle_start = ros::Time::now();
   cycle_diff = cycle_start - cycle_start;

   // when you get a uint32 then turn on physics engine
   unpause_physics.call(empty_packet);
   while (cycle_diff.toSec() < cycle){
       // loop until time expires
       ros::Duration(sleep_secs).sleep();
       cycle_diff = ros::Time::now() - cycle_start;
   }
   // turn off physics
   pause_physics.call(empty_packet);

   // publish message to physics end topic.
   finish_physics_cycle.publish(ss);
}


void stopsim(int sig)
{
  ros::shutdown();
}


int
main(int argc, char **argv)
{
    PhysicsCtrlNode* p;
    ros::init(argc, argv, "physics_ctrl_node");

    // register ctrl-c to shutdown node
    signal(SIGINT, stopsim);

    p = new PhysicsCtrlNode();

    while (ros::ok()) // && count <= 500)
    {
        ros::spinOnce();
    }

    delete p;
    return EXIT_SUCCESS;
}
