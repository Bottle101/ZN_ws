#include <iostream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>

class Pos
{
public:
    Pos() = default;
    double x, y, z;
};

class Vel
{
public:
    Vel() = default;
    double vx, vy, vz;
};

class AnomalyDetection
{
public:
    AnomalyDetection(ros::NodeHandle n);
    //~AnomalyDetection();
    ros::NodeHandle nh, nh_;

    ros::Subscriber stateSub;

    std::string StateTopic;

    Pos pos;
    Pos pos_d;
    Vel vel;

    void Onway();
    void Takeoff();
    void Land();
    void Detection(vector<double>& pos_bias_threshold,vector<double>& vel_threshold,int mode);
    void Overspeed(vector<double>& vel_threshold);
    void TrajMatchScore(vector<double>& pos_bias_threshold);

    void callbackState(const std_msgs::String::ConstPtr &msg);

    bool if_onway_init = false;
    bool if_takeoff_init = false;
    bool if_land_init = false;
    bool if_update = false;
    bool resetall = false;

    ros::Time time;

    double height_pre;
    double height;
    double delta_h;

    int mode = 0;
};
