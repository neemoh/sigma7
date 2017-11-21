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
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Sigma
#include <dhdc.h>
#include <drdc.h>
#include <sensor_msgs/Joy.h>


//-----------------------------------------------------------------------
// Haptic device class
//-----------------------------------------------------------------------
class SigmaDevice {
public:

    // Constructor with node handle and a name space for the published topics
    SigmaDevice(ros::NodeHandle n, const std::string name_space);

    // Call back for the wrench subscriber. These are the forces and torques
    // that the sigma device will exert to the operator's hand.
    void WrenchCallback( const geometry_msgs::WrenchStampedConstPtr &msg);

    // Fetches all the measurements (Pose, Twist, button and pedal) from the
    // device
    int ReadMeasurementsFromDevice();

    // Publishing those 4 things in its name! Button and pedal will be
    // published only when they are pressed or released.
    void PublishPoseTwistButtonPedal();

    // Exert wrenches if any.
    void HandleWrench();

private:

    // Calibrates the device using the SDK commands
    int CalibrateDevice();

private:

    // the id of the device. starts from zero and increments by one if
    // another device is connected
    static int id;

    // sigma can simulate a button with the gripper. THat is when yoy close
    // the gripper it resists a bit at the end and and springs back when you
    // release it.
    bool enable_gripper_button=0;

    // we can lock the orientation when the pedal is released. This is
    // useful for teleoperation
    bool lock_orient=0;


    // the orientation matric in the locked state
    double locked_orient[3]={0., 0., 0.};

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped twist_msg;
    geometry_msgs::WrenchStamped wrench;
    bool new_wrench_msg;
    std_msgs::Float32 gripper_angle;

    // the gripper button and pedal state and their previous state
    int buttons_state[2];
    int buttons_previous_state[2];

    sensor_msgs::Joy buttons_msg; // two elements, 0 is gripper button, 1
    // is pedal
    int pedal_previous_state;

    // publishers and subscribers
    ros::Publisher pub_pose;
    ros::Publisher pub_twist;
    ros::Publisher pub_gripper;
    ros::Publisher pub_buttons;
    ros::Subscriber sub_wrench;
};