#include <iostream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include 

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

class ErrorCode
{
public:

    ErrorCode(ros::NodeHandle n);
    //~ErrorCode();
    ros::NodeHandle nh, nh_;

    ros::Subscriber stateSub;
    ros::Subscriber modeSub;

    std::string StateTopic;
    std::string ModeTopic;

    Pos pos;
    Pos pos_d;
    Vel vel;

    enum ModuleID {
    AllModule = 0,
    BatteryID = 1,
    RTKID = 2,
    DeviceID = 3,
    CompassID = 4,
    IMUID = 5,
    GPSID = 6,
    RCID = 7,
    NSID = 8,
    RadioID = 9,
    Flight_AnomalID = 10,
    TakeoffID = 11,
    };

    uint8_t getErrorCode(uint8_t mode);
    uint8_t getErrorCode(uint8_t module);
    
    void Onway();
    void Takeoff();
    void Land();
    void Detection(vector<double>& pos_bias_threshold,vector<double>& vel_threshold,int mode);
    void Overspeed(vector<double>& vel_threshold);
    void TrajMatchScore(vector<double>& pos_bias_threshold);

    void callbackState(const nav_msgs::Odometry::ConstPtr &msg);
    void callbackMode(const nav_msgs::Odometry::ConstPtr &msg);

    bool if_onway_init = false;
    bool if_takeoff_init = false;
    bool if_land_init = false;
    bool if_update = false;
    bool if_state_update = false;
    bool if_mode_update = false;
    bool resetall = false;

    double time;

    double height_pre;
    double height;
    double delta_h;

    int mode = 0;
};
