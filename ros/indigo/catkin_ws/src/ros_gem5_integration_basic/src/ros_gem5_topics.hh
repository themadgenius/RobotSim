#ifndef __ROS_GEM5_TOPICS_H__
#define __ROS_GEM5_TOPICS_H__
// topics for gem5-ros synchronization
const std::string PHYSICS_START_TOPIC = "/gem5/start_physics";
const std::string PHYSICS_DONE_TOPIC = "/gem5/physics_done";

const std::string GEM5_RUN_CYCLES_TOPIC = "/gem5/clock_cycles";
const std::string GEM5_CYCLE_COMPLETE_TOPIC = "/gem5/cycle_end";

// gazebo sensor/actuator topics
const std::string CAMERA_IMAGE_TOPIC = "/front_cam/camera/image";

// topics for gazebo physics engine
const std::string GAZEBO_PHYSICS_PAUSE_TOPIC = "/gazebo/pause_physics";
const std::string GAZEBO_PHYSICS_UNPAUSE_TOPIC = "/gazebo/unpause_physics";

#endif // __ROS_GEM5_TOPICS_H__
