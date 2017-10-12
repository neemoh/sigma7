//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab
    Refer to Readme.txt for full license text

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   2.0
*/
//==============================================================================

//ROS
#include "ros/ros.h"
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Sigma
#include <dhdc.h>
#include <drdc.h>


//-----------------------------------------------------------------------
// Haptic device class
//-----------------------------------------------------------------------
class HapticDeviceState {
public:
    HapticDeviceState(ros::NodeHandle n, const std::string name_space);

    void WrenchCallback( const geometry_msgs::WrenchStampedConstPtr &msg);

    int CalibrateDevice(const int dev);

    int ReadMeasurementsFromDevice();

    void PublishPoseTwistButtonPedal();

    void HandleWrench();

private:
    static int id;

    double last_orient[3];
    ros::Publisher pub_pose;
    ros::Publisher pub_twist;
    ros::Publisher pub_button;
    ros::Publisher pub_pedal;
    ros::Subscriber force_feedback;

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped twist_msg;

    // Wrench (force, torque)
    geometry_msgs::WrenchStamped wrench;
    bool new_wrench_msg;

    // the button binary state and its previous state
    std_msgs::Int8 sigma_button_state;
    std_msgs::Int8 sigma_button_previous_state;

    // the pedal binary state and its previous state
    std_msgs::Int8 pedal_state;
    std_msgs::Int8 pedal_previous_state;

};