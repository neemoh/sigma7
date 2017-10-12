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

#include <sstream>
#include "HapticDeviceState.hpp"

int main(int argc, char** argv) {

    // -------------------------------------------------------------------------
    // Initialization: Sigma
    int devs=-1;
    //	bool use_pedal = 0;

    // Looking for connected devices
    for (int i = 0; i<2 ; i++){
        if (drdOpenID ((char)i) > -1) {
            devs = i;
        }
    }

    ROS_INFO("Found %i Device(s)", devs+1);

    ros::init(argc, argv, "master");
    ros::NodeHandle n(ros::this_node::getName());

    HapticDeviceState * sigma[devs+1];

    // calibrating devices
    for (int i=0; i<=devs; i++){
        sigma[i] = new HapticDeviceState(n, "right");
        ROS_INFO("Calibrating device %i ...", i);
        if(sigma[i]->CalibrateDevice(i) >-1) ROS_INFO("Device %i ready.", i);
    }

    double freq_ros;
    n.param<double>("/sigma_frequency", freq_ros, 1000);
    ROS_INFO("Set frequency: %f", freq_ros);
    ros::Rate loop_rate(freq_ros);

    //    n.param<bool>("/sigma_use_pedal", use_pedal, 0);
    //	ROS_INFO("Use pedal: %i", use_pedal);


    ROS_INFO("Initialization done");

    geometry_msgs::WrenchStamped Wrench_temp;

    while (ros::ok()) {

        // Reading Sigma measurements and setting the force
        for (int i=0; i<=devs; i++){

            sigma[i]->ReadMeasurementsFromDevice();

            sigma[i]->PublishPoseTwistButtonPedal();

            //Applying force feedback if the pedal is engaged
            sigma[i]->HandleWrench();
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    ROS_INFO("Ending Session...\n");
    for (int i=0; i<=devs; i++){
        if (dhdClose ((char)i) < 0) {
            ROS_ERROR (" %s\n", dhdErrorGetLastStr ());
        }
        else{
            ROS_INFO("Closed device %i" ,i );
        }
    }
    delete sigma[0];
    delete sigma[1];
    return 0;

}