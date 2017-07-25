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

 /* Note from Chris: FIXME
 * This code has poor encapsulation 
 *
 *
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


/* ROS Includes */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

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
#include "ros/hellodevice.hh"

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
    GlobalSimLoopExitEvent* runGem5(Tick io_ticks = 0);
    void IOAddrRemap(const std::string &object_name, Addr vaddr, Addr paddr, int size);
    bool readCpuIOPort(const std::string &object_name, uint32_t *gem5Data);
    void writeCpuIOPort(const std::string &object_name, uint32_t wData);
    void cameraCallback(const std_msgs::StringConstPtr& message);
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
//  std::string config_file;

}; // end RosGem5Interface


RosGem5Interface::RosGem5Interface(const std::string config_filename) :
    clock_ticks_per_second(1000000000000),	checkpoint_restore(false),
    checkpoint_save (false), switch_cpus(false), checkpoint_dir (""),
    from_cpu(""), to_cpu (""), pre_run_time (1000000), pre_switch_time(1000000) 
{

    //ros::NodeHandle private_nh("~");
//    private_nh_.param("pub_drawings", p_pub_drawings, false);

    cxxConfigInit();
    initSignals();  
    this->setClock(clock_ticks_per_second);
    curEventQueue(getEventQueue(0));

    Stats::initSimStats();
    Stats::registerHandlers(CxxConfig::statsReset, CxxConfig::statsDump);
	
    Trace::enable();
    setMyDebugFlag("Terminal");
	
    conf = new CxxIniFile();
    if (!conf->load(config_filename.c_str())) {
        std::cerr << "Can't open config file: " << config_filename << '\n';
        return; // EXIT_FAILURE;
		// FIXME: should catch an exception here instead of an if statement
    }
    config_manager = new CxxConfigManager(*conf);
    exit_event = NULL;
} // end constructor


RosGem5Interface::~RosGem5Interface()
{
    #if TRY_CLEAN_DELETE
        config_manager->deleteObjects();
    #endif
    delete config_manager;
} // end destructor

/****************************************************************
IOAddrRemap
  Description:
     We remap from virtual to physical address and at a given size
     We make this remapping non-chacheable by default.  
     This must be done AFTER the gem5 configuration has been
     initialized.
  Inputs:
    &object_name: Pointer to string containing name of gem5 object 
                 from config.ini
    vaddr: virtual address (hex value)
    paddr: physical address (hex value)
    size:  size of address space to remap
  Outputs:
    None
******************************************************************/
void RosGem5Interface::IOAddrRemap(const std::string &object_name, 
                       Addr vaddr, Addr paddr, int size)
{
    // Find the gem5 object system.cpu.workload (the executable) and call 
    // the map() function to remap the virtural address vaddr to the
    // physical address paddr (where the io device lives).
    (config_manager->getObject<LiveProcess>(object_name)).map(vaddr, paddr, size, false);
}


bool RosGem5Interface::readCpuIOPort(const std::string &object_name, uint32_t* gem5Data)
{
    bool data_good = false;

    data_good = (config_manager->getObject<HelloDevice>(object_name)).rosGetData(gem5Data);
    
    return data_good;
}


void RosGem5Interface::cameraCallback(const std_msgs::String::ConstPtr& msg)
{

    std::string::size_type sz;

    uint32_t i = (uint32_t) std::stoi(msg->data.c_str(),&sz);
    this->writeCpuIOPort("system.pio_ctrl", i);
}


void RosGem5Interface::writeCpuIOPort(const std::string &object_name, uint32_t wData){
    (config_manager->getObject<HelloDevice>(object_name)).rosSetData(wData);
}


void RosGem5Interface::setClock(Tick tps)
{
    setClockFrequency(tps);
}


void RosGem5Interface::setMyDebugFlag (const char* debug_flag)
{
    if (debug_flag[0] == '-'){
        clearDebugFlag(debug_flag + 1);
    } else {
        setDebugFlag(debug_flag);
    }
} // end setDebugFlag


void RosGem5Interface::setCheckpointDir(const std::string &cp_dir)
{
    checkpoint_dir = cp_dir;
} // end setCheckpointDir


int RosGem5Interface::setParameter(const std::string &object_name, const std::string &param_name, const std::string &param_value)
{
    try	{
        config_manager->setParam(object_name, param_name, param_value);
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << e.name << ": " << e.message << "\n";
        return EXIT_FAILURE;
    } // end try

	return EXIT_SUCCESS;
} // end SetParameter


int RosGem5Interface::setParamVector(const std::string &object_name, const std::string &param_name, const std::string &param_values)
{
    std::vector<std::string> values_vector;
    tokenize(values_vector, param_values, ',');
    std::cout << "Object = " << object_name << ", Param name = " << param_name << ", Param values = ";
    for (std::vector<std::string>::const_iterator i = values_vector.begin(); i != values_vector.end(); ++i)
        std::cout << *i << ' ';
    std::cout << "\n";
    try{
        config_manager->setParamVector(object_name, param_name, values_vector);
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << e.name << ": " << e.message << "\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
} // end SetParameter


void RosGem5Interface::setCheckpointSave(const std::string &cp_dir, const std::string &num_ticks)
{
    setCheckpointDir(cp_dir);
    std::istringstream(num_ticks) >> pre_run_time;
    checkpoint_save = true;
} // end setCheckpointSave


void RosGem5Interface::setCheckpointRestore(const std::string &cp_dir)
{
    setCheckpointDir(cp_dir);
    checkpoint_restore = true;
}


void RosGem5Interface::switchCPU(const std::string &old_cpu, const std::string &new_cpu, const std::string &num_ticks)
{
    switch_cpus = true;
    from_cpu = old_cpu;
    to_cpu = new_cpu;
    std::istringstream(num_ticks) >> pre_switch_time;
} // end switchCPU


int RosGem5Interface::init()
{
	
    if (checkpoint_save && checkpoint_restore) {
        std::cerr << "Don't try and save and restore a checkpoint in the same run\n";
        return EXIT_FAILURE;
    }
	
    CxxConfig::statsEnable();
    getEventQueue(0)->dump();

    try {
        config_manager->instantiate();
        if (!checkpoint_restore) {
            config_manager->initState();
            config_manager->startup();
        }
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << "Config problem in sim object " << e.name << ": " << e.message << "\n";
        return EXIT_FAILURE;
    }

	return EXIT_SUCCESS;
} // end init


void RosGem5Interface::usage(const std::string &prog_name)
{
    std::cerr << "Usage: " << prog_name << (
        " <config-file.ini> [ <option> ]\n\n"
        "OPTIONS:\n"
        "    -p <object> <param> <value>  -- set a parameter\n"
        "    -v <object> <param> <values> -- set a vector parameter from"
        " a comma\n"
        "                                    separated values string\n"
        "    -d <flag>                    -- set a debug flag (-<flag>\n"
        "                                    clear a flag)\n"
        "    -s <dir> <ticks>             -- save checkpoint to dir after"
        " the given\n"
        "                                    number of ticks\n"
        "    -r <dir>                     -- restore checkpoint from dir\n"
        "    -c <from> <to> <ticks>       -- switch from cpu 'from' to cpu"
        " 'to' after\n"
        "                                    the given number of ticks\n"
        "\n"
        );
}


GlobalSimLoopExitEvent* RosGem5Interface::runGem5(Tick io_ticks)
{
    if (checkpoint_save) {
        exit_event = simulate(pre_run_time);

        unsigned int drain_count = 1;
        do {
            drain_count = config_manager->drain();

            std::cerr << "Draining " << drain_count << '\n';

            if (drain_count > 0) {
                exit_event = simulate();
            }
        } while (drain_count > 0);

        std::cerr << "Simulation stop at tick " << curTick() << ", cause: " << exit_event->getCause() << '\n';
        std::cerr << "Checkpointing\n";

        /* FIXME, this should really be serialising just for
         *  config_manager rather than using serializeAll's ugly
         *  SimObject static object list */
        Serializable::serializeAll(checkpoint_dir);

        std::cerr << "Completed checkpoint\n";

        config_manager->drainResume();
    } // end checkpoint_save

    if (checkpoint_restore) {
        std::cerr << "Restoring checkpoint\n";

        CheckpointIn *checkpoint = new CheckpointIn(checkpoint_dir, config_manager->getSimObjectResolver());

        DrainManager::instance().preCheckpointRestore();
        Serializable::unserializeGlobals(*checkpoint);
        config_manager->loadState(*checkpoint);
        config_manager->startup();

        config_manager->drainResume();

        std::cerr << "Restored from checkpoint\n";
    } // end checkpoint_restore

    if (switch_cpus) {
        exit_event = simulate(pre_switch_time);

        std::cerr << "Switching CPU\n";

        /* Assume the system is called system */
        System &system = config_manager->getObject<System>("system");
        BaseCPU &old_cpu = config_manager->getObject<BaseCPU>(from_cpu);
        BaseCPU &new_cpu = config_manager->getObject<BaseCPU>(to_cpu);

        unsigned int drain_count = 1;
        do {
            drain_count = config_manager->drain();

            std::cerr << "Draining " << drain_count << '\n';

            if (drain_count > 0) {
                exit_event = simulate();
            }
        } while (drain_count > 0);

        old_cpu.switchOut();
        system.setMemoryMode(Enums::timing);
        new_cpu.takeOverFrom(&old_cpu);
        config_manager->drainResume();

        std::cerr << "Switched CPU\n";
    } // end switch_cpus

    if (io_ticks != 0){
        exit_event = simulate(io_ticks);
    } else {
        exit_event = simulate();
    }

//    getEventQueue(0)->dump();
//    std::cerr << "Exit at tick " << curTick() << ", cause: " << exit_event->getCause() << '\n';


    return exit_event;
} // end runGem5

void stopsim(int sig)
{
  ros::shutdown();
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_gem5_interface");

    ros::NodeHandle n;
    RosGem5Interface* gem5;
    std::string prog_name(argv[0]);
    unsigned int arg_ptr = 2;
    int retval;
    int pub_buf_size = 1000;

    ros::Publisher  to_motor;
    ros::Subscriber to_camera;
    std_msgs::String motor_msg;
    std_msgs::String camera_msg;
    std::stringstream ss;
    

    // create a publisher which will be used for processor writes to ros
//    ros::Publisher to_motor = n.advertise<std_msgs::String>("motor_topic", pub_buf_size);


    // to write i need:
    //     a message
    //     a publisher
    //     spin()

    if (argc == 1){
        RosGem5Interface::usage(prog_name);
        std::exit(EXIT_FAILURE);
    }

    gem5 = new RosGem5Interface(argv[1]);

    // register ROS subscribers and publishers
    to_motor = n.advertise<std_msgs::String>("motor_topic", pub_buf_size);
    to_camera = n.subscribe("camera_topic", 1000, &RosGem5Interface::cameraCallback, gem5 );

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
    gem5->setParameter( "system.pio_ctrl", "devicename", "Chris_is_Awesome");

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
    gem5->IOAddrRemap("system.cpu.workload",IOAddrMap::vMotorBase, 
                      IOAddrMap::pMotorBase, IOAddrMap::motorAddrSize);

    
    // corresponsds to 1Mbps I/O rate with a cpu rate of 1GHz and an
    // i/o bus width of 32 bits.
    //Tick ticks_per_io_cycle = 32768; 
    Tick ticks_per_io_cycle = 5000000; 

    // Set ros loop frequency in Hz
    int max_io_rate = 30;
    ros::Rate loop_rate(max_io_rate);

    GlobalSimLoopExitEvent* simStatus;

    uint32_t write_value;
    uint32_t cpuvalue = 0;
    write_value = 0xFEEDFACE;
    
    signal(SIGINT, stopsim);

    // while ros::ok run for a cycle
    int count = 0;
    while (ros::ok()) // && count <= 500)
    {
      count ++;
//      std::cout << "Count = " << count << "\n";
      // run the gem5 executable for a number of ticks comparable to data rate

      simStatus =  gem5->runGem5(ticks_per_io_cycle);


      // write to cpu if anybody published to topics the port cares aobut
//        gem5->writeCpuIOPort("system.pio_ctrl", write_value);
//        write_value += (uint32_t) 1;
//        std::cout << "ROS: Writing " << write_value << " to gem5\n";
        //gem5->runGem5();

      // read from cpu and publish to topic if data is valid
        // check for external i/o;
        bool valid_cpu_data = false;
        cpuvalue = 0;
        valid_cpu_data = gem5->readCpuIOPort("system.pio_ctrl", &cpuvalue);
        if (valid_cpu_data == true){
            ss << "Data from cpu: " << std::to_string(cpuvalue) << "\n";
            motor_msg.data = ss.str();
            //ROS_INFO("%S", motor_msg.data.c_str());
            to_motor.publish(motor_msg);
            std::cout << "ROS: Read data from gem5 is valid.\n";
            std::cout << "ROS: Read 0x" << std::hex << std::uppercase << cpuvalue << " from gem5\n";
        }
    

      // possibly throw in a ros::spin() here. not sure yet.
      // do a ros::spin() so that ros can process messages to/from node
      ros::spinOnce();

      // then put ros node to sleep
      loop_rate.sleep();
    }

    getEventQueue(0)->dump();
    std::cerr << "Exit at tick " << curTick() << ", cause: " << simStatus->getCause() << '\n';

    // we must delete gem5 during node shutdown when we use an event loop
    std::cout << "Deleting gem5 simulation object\n";
    delete gem5;

    return EXIT_SUCCESS;
}

