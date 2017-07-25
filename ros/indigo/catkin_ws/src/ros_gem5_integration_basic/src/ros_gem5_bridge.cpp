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
#include "ros_gem5_topics.hh"

// Set ros topic names
/*
const std::string CAMERA_IMAGE_TOPIC = "/front_cam/camera/image";
const std::string GEM5_RUN_CYCLES_TOPIC = "/gem5/clock_cycles";
const std::string GEM5_CYCLE_COMPLETE_TOPIC = "/gem5/cycle_end";
*/

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
    
    // create publishers and subscribers
    to_camera = _nh.subscribe(CAMERA_IMAGE_TOPIC, subscriber_buf_size , &RosGem5Interface::cameraCallback, this );

    start_sim = _nh.subscribe(GEM5_RUN_CYCLES_TOPIC, 1 , &RosGem5Interface::runGem5Callback, this );

    finish_gem5_cycle = _nh.advertise<std_msgs::String>(GEM5_CYCLE_COMPLETE_TOPIC, 1);
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


int RosGem5Interface::processCmdLineOptions(int argc, char **argv)
{
    unsigned int arg_ptr =2;
    std::string prog_name(argv[0]);
    int retval = 1;
    
        // Process command line options
    // see usage() for more info.
    while (arg_ptr < argc) {
        std::string option(argv[arg_ptr]);
        arg_ptr++;
        unsigned num_args = argc - arg_ptr;
        if (option == "-p") {
            if (num_args < 3){
                usage(prog_name);
                retval = -1;
                return retval;
            }
            this->setParameter(argv[arg_ptr], argv[arg_ptr + 1], argv[arg_ptr + 2]);

            arg_ptr += 3;
        } else if (option == "-v") {
            if (num_args < 3){
                usage(prog_name);
                retval = -1;
                return retval;
            }
            this->setParamVector(argv[arg_ptr], argv[arg_ptr + 1], argv[arg_ptr + 2]);
            arg_ptr += 3;
        } else if (option == "-d") {
            if (num_args < 1){
                usage(prog_name);
                retval = -1;
                return retval;
            }
            this->setMyDebugFlag(argv[arg_ptr]);
            arg_ptr++;
        } else if (option == "-r") {
            if (num_args < 1){
                usage(prog_name);
                retval = -1;
                return retval;
            }
            this-> setCheckpointRestore(argv[arg_ptr]);
            arg_ptr++;
        } else if (option == "-s") {
            if (num_args < 2){
                usage(prog_name);
                retval = -1;
                return retval;
            }
            this->setCheckpointSave(argv[arg_ptr], argv[arg_ptr + 1]);
            arg_ptr += 2;
        } else if (option == "-c") {
            if (num_args < 3){
                usage(prog_name);
                retval = -1;
                return retval;
            }
            this->switchCPU(argv[arg_ptr], argv[arg_ptr + 1], argv[arg_ptr + 2]);
            arg_ptr += 3;
        } else {
            usage(prog_name);
            retval = -1;
            return retval;
        } // end else-if
    } // end while

    return retval;
} // end processCmdLineOptions


void RosGem5Interface::runGem5Callback(const std_msgs::UInt64::ConstPtr& msg)
{
    // take msg.data, convert it to clock ticks, and run gem5 for that many ticks
    // send blank message when gem5 has finished.
//    std::cout << "(ROS) msg->data = " << msg->data + 5 << "\n";
    // get msg->data, convert to ticks, call runGem5
    Tick t = msg->data;
    runGem5(t);

    std_msgs::String str;
    str.data = "Chris is cool";
    finish_gem5_cycle.publish(str);
}


/*
void RosGem5Interface::gem5PubCallback(const std_msgs::StringConstPtr& msg)
{
    std::cout << "(ROS) msg->data = " << msg->data + 5 << "\n";
}
*/


void RosGem5Interface::cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // CHRIS
//    uint32_t ctrl_reg, status_reg, numbytes, 
//    uint32_t write_data;
//    int get_image;
//    int img_format; 
     uint32_t hw_pixel_len, img_pixel_len; 
     unsigned char * hw_buf, img_data;

//     std::cout << "Entering callback, frame " << msg->header.seq << "  \n";
//     std::cout << "(ROS) Callback for camera frame [" << msg->header.seq << "]: \n";
//     std::cout << "Image type: " << msg->encoding << "\n";
//     std::cout << "Image Resolution is " << msg->width << " x " << msg->height ; 
//     std::cout << " with " << msg->step / msg->width << " byte(s) per pixel \n";

    // get gem5 camera controller object
    CameraDevice *cam = &(config_manager->getObject<CameraDevice>(CAMERA_PORT_OBJ_NAME));


    if (cam->camBufAvail()){
        //std::cout << "(ROS) Callback for camera frame [" << msg->header.seq << "]: \n";
        ROS_INFO("(ROS) Callback for camera frame [%d]", msg->header.seq) ;

        // get pointer to hardware buffer
        hw_buf = cam->getHWBuf();

//        std::cout << "HW buffer pointer read: " << (void *) hw_buf << "\n";

        // copy frame number into hw buffer
        *((uint32_t *) hw_buf) = static_cast<uint32_t>(msg->header.seq);

        // read pixel data length from hw buffer
        hw_pixel_len = *((uint32_t *)(hw_buf + 4));
        // get num of bytes of camra image
        img_pixel_len = msg->height * msg->step;

        // copy pixel data into hw buffer
        if (hw_pixel_len >= img_pixel_len){
            std::memcpy((void*)(hw_buf + 8), (void*)(&(msg->data[0])), img_pixel_len);
            *((uint32_t *) (hw_buf+4)) = img_pixel_len; // new length of pixel data in hw buffer
// CHRIS
            printf("(ROS) Frame %d image buffer ptr 0x%p \n",msg->header.seq, (hw_buf + 8));
        } else {
            // we are not copying the full image here because the hw buffer is not big enough
            std::memcpy((void*)(hw_buf + 8), (void*)((msg->data).data()), hw_pixel_len);
        }

        // put pointer into input queue and set data available flag
        cam->putHWBuf(hw_buf);

//        std::cout << "(ROS) Leaving callback for frame [" << msg->header.seq << "]  \n";
    } // end if camBufAvail 
/*
        else {
          std::cout << "(ROS) Camera buffer not available" << "\n";
     }
*/

} // end callback


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


// FIXME
void RosGem5Interface::getStats()
{
    CxxConfig::statsDump();
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


