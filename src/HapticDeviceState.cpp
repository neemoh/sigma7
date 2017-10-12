//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab
    Refer to Readme.txt for full license text

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   -
*/
//==============================================================================

#include "HapticDeviceState.hpp"
int HapticDeviceState::id = -1;

//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
HapticDeviceState::HapticDeviceState(ros::NodeHandle n,
                                     const std::string ns)
        : new_wrench_msg(false)
{
    id++;

    // setup the publishers and the subscriber
    pub_pose = n.advertise<geometry_msgs::PoseStamped>(ns+"/pose",1, 0);
    pub_twist = n.advertise<geometry_msgs::TwistStamped>(ns+"/twist", 1, 0);
    pub_button  = n.advertise<std_msgs::Int8>(ns+"/button", 1, 0);
    pub_pedal = n.advertise <std_msgs::Int8> (ns+"/pedal", 1, 0);

    force_feedback	= n.subscribe("/sigma/"+ns+"/forceFeedback", 1,
                                    &HapticDeviceState::WrenchCallback, this);
}


//------------------------------------------------------------------------------
// WrenchCallback
void HapticDeviceState::WrenchCallback(
        const geometry_msgs::WrenchStampedConstPtr &msg) {
    //newDataDirect = true;
    wrench.wrench = msg->wrench;
    new_wrench_msg = true;
}

//------------------------------------------------------------------------------
// CalibrateDevice
int HapticDeviceState::CalibrateDevice(int const dev){
    // center of workspace
    //	  double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0,  // base  (translations)
    //	                                   0.0, 0.0, 0.0,  // wrist (rotations)
    //	                                   0.0 };          // gripper

    // open device
    if (drdOpenID (dev) < 0) {
        ROS_ERROR(" No device %i found. dhd says: (%s)",dev, dhdErrorGetLastStr());
        dhdSleep (2.0);
        for (int j=0; j<=dev; j++) drdClose (j);
        return -1;
    }

    //Calibrate the device if it is not already calibrated;
    if(drdIsInitialized(dev)){
        ROS_INFO("Device %i is already calibrated.",dev);
    }
    else if(drdAutoInit(dev)<0) {
        ROS_ERROR("Initialization of device %i failed. dhd says: (%s)",dev, dhdErrorGetLastStr ());
        dhdSleep(2.0);
    }

    // move to center
    //	drdMoveTo (nullPose);
    // stop regulation (and leave force enabled)
    drdStop(true,dev);
    // enable force
    dhdEnableForce (DHD_ON, dev);
    //Enable the gripper button
    dhdEmulateButton(DHD_ON, dev);
    return 0;
}

int HapticDeviceState::ReadMeasurementsFromDevice() {

    // -------------------------------
    // Pose
    double p[3];
    double orient_m[3][3];
    //Reading the data from the device
    dhdGetPositionAndOrientationFrame(&p[0], &p[1], &p[2], orient_m, (char)id);

    // convert to pose message
    KDL::Rotation rot;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) { rot(r, c) = orient_m[r][c]; }
    }
    tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(p[0],p[1],p[2])),
                     pose_msg.pose);
    // stamp the msg
    pose_msg.header.stamp = ros::Time::now();

    // -----------------------------
    // Twist
    double v[6];

    dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
    dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
    // convert to twist message
    twist_msg.twist.linear.x = v[0];
    twist_msg.twist.linear.y = v[1];
    twist_msg.twist.linear.z = v[2];
    twist_msg.twist.angular.x = v[3];
    twist_msg.twist.angular.y = v[4];
    twist_msg.twist.angular.z = v[5];
    // stamp the msg
    twist_msg.header.stamp = ros::Time::now();

    // ------------------------------
    // buttons
    // saving the previous states of button and pedal
    sigma_button_previous_state = sigma_button_state;
    sigma_button_state.data = dhdGetButton(0, (char)id);

    pedal_previous_state = pedal_state;
    pedal_state.data = dhdGetButton(1, (char)id);
    return 0;
}

void HapticDeviceState::PublishPoseTwistButtonPedal() {

    pub_pose.publish(pose_msg);
    pub_twist.publish(twist_msg);

    if(sigma_button_state.data != sigma_button_previous_state.data){
        pub_button.publish(sigma_button_state);
    }

    //Publish pedal only when it changes
    if(pedal_state.data != pedal_previous_state.data){
        pub_pedal.publish(pedal_state);
    }

}

void HapticDeviceState::HandleWrench() {

    if(pedal_state.data == 1) {
        if (dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.x,
                                                wrench.wrench.force.y,
                                                wrench.wrench.force.z,
                                                wrench.wrench.torque.x,
                                                wrench.wrench.torque.y,
                                                wrench.wrench.torque.z,
                                                0.0, (char)id) < DHD_NO_ERROR){
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }
        dhdGetOrientationRad(&last_orient[0], &last_orient[1],&last_orient[2]);
    }
    else{
        drdRegulatePos  (false);
        drdRegulateRot  (true);
        drdRegulateGrip (false);
        drdStart();
        drdMoveToRot (last_orient[0], last_orient[1],last_orient[2]);
        drdStop(true);
    }

}

