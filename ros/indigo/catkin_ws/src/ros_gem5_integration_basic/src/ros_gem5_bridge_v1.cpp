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

#include "ros_gem5_bridge.hh"



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

    // FIXME
    //data_good = (config_manager->getObject<CameraDevice>(object_name)).rosGetData(gem5Data);
    
    return data_good;
}


void RosGem5Interface::cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // CHRIS
    uint32_t ctrl_reg, status_reg, numbytes;
    uint32_t write_data;
    int get_image;
    int img_format; 

     std::cout << "Entering callback, frame " << msg->header.seq << "  \n";
//     std::cout << "Image type: " << msg->encoding << "\n";
//     std::cout << "Row stride is : " << msg->step << " bytes \n";
//     std::cout << "(ROS) Callback for camera frame [" << msg->header.seq << "]: \n";
    

//    cam.rosGetData(CAM_REGS::STATUS,&status_reg);
//    cam.rosSetData(CAM_REGS::DATA,ctrl_reg);

    // get gem5 camera controller object
    CameraDevice *cam = &(config_manager->getObject<CameraDevice>(CAMERA_PORT_OBJ_NAME));

    // read control register
    cam->rosGetData(CAM_REGS::CTRL,&ctrl_reg);
//    std::cout << " Control Register = " << ctrl_reg << ", ";

    // if command is to read camera
    get_image = 0;
    get_image = ctrl_reg & CAM_CTRL::CAPTURE_IMAGE;
//    std::cout << "get_image = " << get_image << "\n";
    if (get_image){
        // write camera image to cpu in 32 bit chunks
        // write frame sequence number
        write_data = static_cast<uint32_t>(msg->header.seq);
        cam->rosSetData(CAM_REGS::FRAME_NO,write_data);
//        std::cout << "Frame[" << msg->header.seq << "]: ";
//        std::cout << "writing frame # " << write_data << " to data register \n";

        // write frame height
        write_data = static_cast<uint32_t>(msg->height);
        cam->rosSetData(CAM_REGS::DIM_H,write_data);
//        std::cout << "Frame[" << msg->header.seq << "]: ";
//        std::cout << "writing height " << write_data << " to data register \n";

        // write frame width 
        write_data = static_cast<uint32_t>(msg->width);
        cam->rosSetData(CAM_REGS::DIM_W,write_data);
//        std::cout << "Frame[" << msg->header.seq << "]: ";
//        std::cout << "writing width " << write_data << " to data register \n";

//        std::cout << "Image type: " << msg->encoding << "\n";
//        std::cout << "Row stride is : " << msg->step << " bytes \n";

        // Determine if we want mono or color image
        img_format = 0;
        img_format = ctrl_reg & CAM_CTRL::MONO8;
        
        // write total # of bytes in pixel array
        if (img_format == CAM_CTRL::MONO8){
          numbytes = msg->height * msg->width;
        } else {
          numbytes = msg->height * msg->step;
        }
        write_data = static_cast<uint32_t>(numbytes);
        cam->rosSetData(CAM_REGS::FRAME_SIZE,write_data);
//        std::cout << "Frame[" << msg->header.seq << "]: ";
//        std::cout << "writing total bytes " << numbytes << " to data register \n";

        // write pixel data
        // for last non-full word, data is in msb
        write_data = 0;
        uint64_t tmp;
        unsigned char _frame[numbytes];
        if (img_format == CAM_CTRL::MONO8){
          // send image to gem5 in greyscale
          // assume ROS image is in R8G8B8 format
          unsigned int num_color_img_bytes = numbytes * 3;
          unsigned int j = 0;
          for (unsigned int i=1; i<=num_color_img_bytes; i += 3){
            // use averaging method to convert color pixel to greyscale
            tmp = (uint64_t)msg->data[i-1]; // red pixel
            tmp += (uint64_t)msg->data[i]; // green pixel
            tmp += (uint64_t)msg->data[i+1]; // blue pixel 
            tmp = tmp / 3; 
            // sanity check to make sure we still have 8 bit pixel
            if (tmp & 0xFFFFFFFFFFFFFF00) {
              std::cout << "(ROS) Error converting color image to greyscale\n";
            }
            _frame[j] = (unsigned char) tmp;
//            std::cout << "pixel[" << j << "]: " << std::hex << _frame[j] << "\n";
//            write_data = write_data | tmp << 8*(7-j);
            j++;
//            if (j == 8){
//              j = 0;
//              cam->rosSetData(CAM_REGS::DATA,write_data);
//              write_data = 0;
//            }
          } // end for
          
        } else {
          // send image to gem5 in R8G8B8 format
          for (int i=1; i<=numbytes; i++){
            tmp = (uint64_t)msg->data[i-1];
            _frame[i] = (unsigned char) tmp;
//            write_data = write_data | tmp << 8*(8-(i%8));
//            if (i%8 == 0){
//              cam->rosSetData(CAM_REGS::DATA,write_data);
//              write_data = 0;
//            } // end if
          } // end for
        } // end if MONO8

//        std::cout << "(ROS) Writing gem5 frame buffer \n";
        cam->rosSetFrame((void*)_frame, (unsigned int)numbytes);
//        std::cout << "(ROS) gem5 frame buffer set \n";
//        cam->rosSetData(CAM_REGS::DATA,write_data);

        // clear read request from the command register
        ctrl_reg = ~CAM_CTRL::CAPTURE_IMAGE & ctrl_reg;
        ctrl_reg = ~CAM_CTRL::MONO8 & ctrl_reg;
        cam->rosSetData(CAM_REGS::CTRL,ctrl_reg);

        // set data available bit in status register 
        cam->rosGetData(CAM_REGS::STATUS,&status_reg);
        status_reg = status_reg | CAM_STATUS::DATA_AVAIL;
        cam->rosSetData(CAM_REGS::STATUS,status_reg);
//        std::cout << "(ROS) Leaving callback \n";
    } // end if get_image
//    std::cout << "Leaving callback, frame " << msg->header.seq << "  \n";

//    this->writeCpuIOPort(CAMERA_PORT_OBJ_NAME, i);
}


void RosGem5Interface::writeCpuIOPort(const std::string &object_name, uint32_t wData){
// FIXME
//    (config_manager->getObject<CameraDevice>(object_name)).rosSetData(wData);
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
    this->dumpCpuEvents();
//    getEventQueue(0)->dump();

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


Tick RosGem5Interface::getCurTick()
{
    return curTick();
}


void RosGem5Interface::dumpCpuEvents()
{
    getEventQueue(0)->dump();
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


