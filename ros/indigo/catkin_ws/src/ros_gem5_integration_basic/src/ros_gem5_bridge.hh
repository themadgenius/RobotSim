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

#ifndef __ROS_GEM5_BRIDGE_H__
#define __ROS_GEM5_BRIDGE_H__


#include <cstdlib>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <signal.h>


/* ROS Includes */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"
#include "sensor_msgs/Image.h"

/* gem5 Includes */
#include "base/inifile.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "sim/cxx_config_ini.hh"
#include "sim/cxx_manager.hh"
#include "sim/init_signals.hh"
#include "sim/serialize.hh"
#include "sim/simulate.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"
#include "sim/process.hh"
#include "stats.hh"
#include "ros/io_address_map.hh"
//#include "ros/hellodevice.hh"
#include "ros/cameradevice.hh"

// Set the node name for the callback function
const std::string CAMERA_PORT_OBJ_NAME = "system.cam_ctrl";
const std::string CPU_EXECUTABLE_OBJ = "system.cpu.workload";

class RosGem5Interface{
  public:
   
    RosGem5Interface(const std::string config_filename = "");//    { }
    ~RosGem5Interface();//   { }
    static void usage(const std::string &prog_name);
    void setClock (Tick ticks_per_sec);
    //void setMyDebugFlag (const std::string &debug_flag);
    void setMyDebugFlag (const char* debug_flag);
    //Tick getClockFrequency ();
    int setParameter(const std::string &object_name, const std::string &param_name, const std::string &param_value);
    int setParamVector(const std::string &object_name, const std::string &param_name, const std::string &param_values);
    void setCheckpointDir(const std::string &cp_dir);
    void setCheckpointRestore(const std::string &cp_dir);
    void setCheckpointSave(const std::string &cp_dir, const std::string &num_ticks);
    void switchCPU(const std::string &old_cpu, const std::string &new_cpu, const std::string &num_ticks);
    int init ();
    Tick getCurTick();
    void dumpCpuEvents();
    void getStats();
    GlobalSimLoopExitEvent* runGem5(Tick io_ticks = 0);
    void IOAddrRemap(const std::string &object_name, Addr vaddr, Addr paddr, int size);
    bool readCpuIOPort(const std::string &object_name, uint32_t *gem5Data);
    void writeCpuIOPort(const std::string &object_name, uint32_t wData);
    int processCmdLineOptions(int argc, char **argv);
    void cameraCallback(const sensor_msgs::Image::ConstPtr& message);
    void runGem5Callback(const std_msgs::UInt64::ConstPtr& message);
//    void gem5PubCallback(const std_msgs::StringConstPtr& message);
//    void cameraCallback(const std_msgs::StringConstPtr& message);
/*
	void loadConfigFile (CxxConfigFileBase *conf);
*/
	
  private:
    Tick clock_ticks_per_second; // clock frequency
    CxxConfigFileBase *conf; // = new CxxIniFile();
    CxxConfigManager *config_manager; // = new CxxConfigManager(*conf);
    bool checkpoint_restore; // = false;
    bool checkpoint_save; // = false;
    bool switch_cpus; // = false;
    std::string checkpoint_dir; // = "";
    std::string from_cpu; // = "";
    std::string to_cpu; // = "";
    Tick pre_run_time; // = 1000000;
    Tick pre_switch_time; // = 1000000;
    GlobalSimLoopExitEvent *exit_event;
    ros::NodeHandle _nh;
    ros::Subscriber to_camera;
    ros::Subscriber start_sim;
    ros::Publisher finish_gem5_cycle;
//    int subscriber_buf_size = 1000;
    int subscriber_buf_size = 1;

//  std::string config_file;

}; // end RosGem5Interface


#endif // __ROS_GEM5_BRIDGE_H__
