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
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include "ros_gem5_bridge.hh"

// For execution timing
using namespace std;
using ns = chrono::nanoseconds;
using get_time = chrono::steady_clock;


void stopsim(int sig)
{
  ros::shutdown();
}

int
main(int argc, char **argv)
{
    // look into async spinner for non blocking messages
    ros::init(argc, argv, "ros_gem5_interface");
    //ros::AsyncSpinner spinner(1);

    ros::NodeHandle n;
    RosGem5Interface* gem5;
    std::string prog_name(argv[0]);
    unsigned int arg_ptr = 2;
    int retval;
    int pub_buf_size = 1000;
    int subscriber_buf_size = 1000;
    int gem5_iter = 1;
    std::chrono::time_point<get_time> gem5_start, gem5_end;
    std::chrono::nanoseconds gem5_diff(0), gem5_sum(0), gem5_avg(0), sim_cycle_time(0);

    ros::Publisher  to_motor;
    ros::Subscriber to_camera;
    ros::ServiceClient pause_physics;
    ros::ServiceClient unpause_physics;
    std_msgs::String motor_msg;
    std_msgs::String camera_msg;
    std_srvs::Empty empty_packet;
    std::stringstream ss;
    
    ros::Time ros_gem5_start, ros_gem5_end;
    ros::Time physics_start, physics_end;
    ros::Duration  ros_gem5_diff;
    ros::Duration  physics_diff;

    


    if (argc == 1){
        RosGem5Interface::usage(prog_name);
        std::exit(EXIT_FAILURE);
    }

    gem5 = new RosGem5Interface(argv[1]);

    // register ROS subscribers and publishers
    // FIXME: motor topic should be a Twist messgage
//    to_motor = n.advertise<std_msgs::String>("motor_topic", pub_buf_size);
    // drone image has an update rate of 10 and is 320x240 in R8G8B8 format
    to_camera = n.subscribe("/front_cam/camera/image", subscriber_buf_size , &RosGem5Interface::cameraCallback, gem5 );

    // create persistent clients to physics pause and upnapuse services
    pause_physics = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics",true);
    unpause_physics = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics",true);

    // Process command line options
    // see usage() for more info.
    while (arg_ptr < argc) {
        std::string option(argv[arg_ptr]);
        arg_ptr++;
        unsigned num_args = argc - arg_ptr;
        if (option == "-p") {
            if (num_args < 3){
                RosGem5Interface::usage(prog_name);
		delete gem5;
		std::exit(EXIT_FAILURE);
            }
            gem5->setParameter(argv[arg_ptr], argv[arg_ptr + 1], argv[arg_ptr + 2]);
            arg_ptr += 3;
        } else if (option == "-v") {
            if (num_args < 3){
                RosGem5Interface::usage(prog_name);
                delete gem5;
                std::exit(EXIT_FAILURE);
            }
            gem5->setParamVector(argv[arg_ptr], argv[arg_ptr + 1], argv[arg_ptr + 2]);
            arg_ptr += 3;
        } else if (option == "-d") {
            if (num_args < 1){
                RosGem5Interface::usage(prog_name);
                delete gem5;
                std::exit(EXIT_FAILURE);
            }
            gem5->setMyDebugFlag(argv[arg_ptr]);
            arg_ptr++;
        } else if (option == "-r") {
            if (num_args < 1){
                RosGem5Interface::usage(prog_name);
                delete gem5;
                std::exit(EXIT_FAILURE);
            }
            gem5-> setCheckpointRestore(argv[arg_ptr]);
            arg_ptr++;
        } else if (option == "-s") {
            if (num_args < 2){
                RosGem5Interface::usage(prog_name);
                delete gem5;
                std::exit(EXIT_FAILURE);
            }
            gem5->setCheckpointSave(argv[arg_ptr], argv[arg_ptr + 1]);
            arg_ptr += 2;
        } else if (option == "-c") {
            if (num_args < 3){
                RosGem5Interface::usage(prog_name);
                delete gem5;
                std::exit(EXIT_FAILURE);
            }
            gem5->switchCPU(argv[arg_ptr], argv[arg_ptr + 1], argv[arg_ptr + 2]);
            arg_ptr += 3;
        } else {
            RosGem5Interface::usage(prog_name);
            delete gem5;
            std::exit(EXIT_FAILURE);
        } // end else-if
    } // end while


    // At this point we have read in the gem5 configuration file
    // Set any additional object parameters as necessary.
//     Ex:  gem5->setParameter( "system.cam_ctrl", "devicename", "Chris_is_Awesome");

    // Initialize the gem5 configuration (instantiates c++ objects)
    retval = gem5->init();

    if (retval == EXIT_FAILURE){
        delete gem5;
        std::exit(EXIT_FAILURE);
    }

    // This is where we do virtual to physical address mapping
    // on the user executable process (usually system.cpu.workload).  
    // It MUST come after the init/instantiate.
    // We remap from virtual to physical address and at a given size
    // We make this remapping non-chacheable by default.  
    gem5->IOAddrRemap(CPU_EXECUTABLE_OBJ, IOAddrMap::vCameraBase, 
                      IOAddrMap::pCameraBase, IOAddrMap::cameraAddrSize);

    
    // corresponsds to 1Mbps I/O rate with a cpu rate of 1GHz and an
    // i/o bus width of 32 bits.
    // 1 tick is 1 ps
    //Tick ticks_per_io_cycle = 32768; // 32.7 ns
    //Tick ticks_per_io_cycle = 5000000; // 5 us
    //Tick ticks_per_io_cycle = 20000000; // 20 us
    //Tick ticks_per_io_cycle = 2000000;  // 2 us
    //Tick ticks_per_io_cycle = 200000; // 200 ns
    //Tick ticks_per_io_cycle = 80000000; // 80 us
    //Tick ticks_per_io_cycle = 230000;  // 230 ns
    //Tick ticks_per_io_cycle = 60000000;  // 60 us
    //Tick ticks_per_io_cycle = 100000000;  // 100 us
    //Tick ticks_per_io_cycle = 1000000000;  // 1 ms
    Tick ticks_per_io_cycle = 5000000000;  // 5 ms
//    Tick ticks_per_io_cycle = 2400000000;  // 24 ms
    //Tick gem5_init_ticks = 14837440512;   // 14.8 ms
    Tick gem5_init_ticks = 14837400000; // 14.8 ms
  
    // Set ros loop frequency in Hz
    int max_io_rate = 30;
    ros::Rate loop_rate(max_io_rate);

    GlobalSimLoopExitEvent* simStatus;

    uint32_t write_value = 0;
    uint32_t cpuvalue = 0;
    
    // register ctrl-c to shutdown node
    signal(SIGINT, stopsim);

     // while ros::ok run for a cycle
//    clock_t begin = clock();
//    double elapsed_secs;
      gem5_start = get_time::now();
    // pause physics simulation
    pause_physics.call(empty_packet);
    ROS_INFO("Stopping physics simulation %f", ros::Time::now().toSec());
    // run gem5 to initialize system before we enter the I/O loop
//    simStatus =  gem5->runGem5(ticks_per_io_cycle);
    simStatus =  gem5->runGem5(gem5_init_ticks);
    while (ros::ok()) // && count <= 500)
    {
      // std::cout << "Count = " << count << "\n";
      // run the gem5 executable for a number of ticks comparable to data rate

// FIXME
//      gem5_start = get_time::now();
//      ros_gem5_start = ros::Time::now();
// realtime is 33 ms per frame
// run simulator for 10 ms
      gem5_end = get_time::now();
      sim_cycle_time = gem5_end - gem5_start;
      gem5_start = gem5_end;
      physics_start = ros::Time::now();
      physics_diff = physics_start - physics_start;
      // turn physics on
      unpause_physics.call(empty_packet);
      ROS_INFO("Starting physics simulation %f", physics_start.toSec());
      // while duration is not met
      while(physics_diff.toSec() < 0.020){ // 20 ms 
        // sleep for 10 ms
        ros::Duration(0.010).sleep();  
        physics_diff = ros::Time::now() - physics_start;
      }
      // turn physics off
      pause_physics.call(empty_packet);
      ROS_INFO("Stopping physics simulation %f", ros::Time::now().toSec());

       
      
// run gem5 for 5 * 2 ms
      simStatus =  gem5->runGem5(ticks_per_io_cycle*4);
//      ros_gem5_end = ros::Time::now();
//      ros_gem5_diff = ros_gem5_end - ros_gem5_start;
//      std::cout << "(ROS)[" << gem5_iter <<"] Gem5 execution time[" << (ticks_per_io_cycle / 1e3) << " ns]: " << ros_gem5_diff.toNSec() << " ns (simulation time).\n";
//      gem5_end = get_time::now();
//      std::cerr << "Paused at tick " << gem5->getCurTick() << ", cause: " << simStatus->getCause() << '\n';
//      gem5_diff = gem5_end - gem5_start;
//      gem5_sum += gem5_diff;
//      gem5_avg = gem5_sum / gem5_iter;
//      std::cout << "(ROS) Gem5 execution time: " << chrono::duration_cast<ns>(gem5_diff).count() ;
//      std::cout << " ns for " << ticks_per_io_cycle << " simulated clock ticks (";
//      std::cout << chrono::duration_cast<ns>(gem5_avg).count() << " avg)\n";
//      simStatus =  gem5->runGem5();


      // write to cpu if anybody published to topics the port cares aobut
      // Note: This is now handled in the cameraCallback method in gem5 object
      /*
        gem5->writeCpuIOPort("system.cam_ctrl", write_value);
        write_value += (uint32_t) 1;
        std::cout << "ROS: Writing " << write_value << " to gem5\n";
      */
        //gem5->runGem5();

        // read from cpu and publish to topic if data is valid
        // check for external i/o;
/*
        bool valid_cpu_data = false;
        cpuvalue = 0;
        valid_cpu_data = gem5->readCpuIOPort("system.cam_ctrl", &cpuvalue);
        if (valid_cpu_data == true){
            ss << "Data from cpu: " << std::to_string(cpuvalue) << "\n";
            motor_msg.data = ss.str();
            //ROS_INFO("%S", motor_msg.data.c_str());
            to_motor.publish(motor_msg);
            // FIXME : this print statement can come out after debug
            std::cout << "ROS: Read data from gem5 is valid.\n";
            std::cout << "ROS: Read 0x" << std::hex << std::uppercase << cpuvalue << " from gem5\n";
        }
*/
    

      // possibly throw in a ros::spin() here. not sure yet.
      // do a ros::spin() so that ros can process messages to/from node
      //std::cout << "(ROS) ROS node elapsed time: " << ros_gem5_end.toSec() - ros_gem5_start.toSec() << " seconds\n";
      gem5_iter += 1;
      //std::cout << "(ROS) Iteration " << gem5_iter << " Checking callback for camera messages.\n";
      ros::spinOnce();

      // then put ros node to sleep
//      std::cout << "putting node to sleep\n";
//      loop_rate.sleep();
//      std::cout << "waking node up from sleep\n";
//      clock_t end = clock();
//      elapsed_secs = double(end-begin) / CLOCKS_PER_SEC;
//      std::cout << "Ros elapsed secs: " << elapsed_secs << "\n";
      std::cout << "Co-simulation real-time factor: " << 1e6 * 20 / sim_cycle_time.count() << "\n";
    } // end while

//    getEventQueue(0)->dump();
    gem5->dumpCpuEvents();
    std::cerr << "Exit at tick " << gem5->getCurTick() << ", cause: " << simStatus->getCause() << '\n';

    std::cout << "Deleting gem5 simulation object\n";
    delete gem5;

    return EXIT_SUCCESS;
}

