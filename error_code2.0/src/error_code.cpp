/*errorcode 250 means wrong input
  errorcode 0 means successful*/



#include "error_code.h"
#include "error_code/mode.h"

using namespace std;
using namespace ros;

uint8_t ErrorCode::getErrorCode(uint8_t mode)
{
    return ErrorCode::getErrorCode(mode, 0);
}

uint8_t ErrorCode::getErrorCodeMode(uint8_t mode)
{
    return ErrorCode::getErrorCode(mode, 11);
}

uint8_t ErrorCode::getErrorCodeModule(uint8_t module)
{
    return ErrorCode::getErrorCode(0, module);
}

uint8_t ErrorCode::getErrorCode(uint8_t mode, uint8_t module)
{
    // uint8_t ret=0x00;
    // int cnt = 0;
    int if_all = 0;
    if(mode && (!module))
    {
        if_all = 1;
        module++;
    }
    switch((uint32_t)module)
    {
        case ErrorCode::ModuleID::BatteryID: 
            if(DJI::OSDK::Telemetry::Battery::voltage < 20.0)
                    return ErrorCode::ModuleID::BatteryID;
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 0;
                break;
            }
        case ErrorCode::ModuleID::RTKID: 
            if(DJI::OSDK::Telemetry::RTKConnectStatus::rtkConnected)
                return ErrorCode::ModuleID::RTKID;
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 0;
                break;
            }
        case ErrorCode::ModuleID::CompassID: 
            if(DJI::OSDK::Telemetry::FlightAnomaly::compassInstallationError)
                return ErrorCode::ModuleID::CompassID;
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 0;
                break;
            }
        case ErrorCode::ModuleID::IMUID: 
            if(DJI::OSDK::Telemetry::FlightAnomaly::imuInstallationError)
                return ErrorCode::ModuleID::IMUID;
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 0;
                break;
            }
        case ErrorCode::ModuleID::GPSID: 
            if(DJI::OSDK::Telemetry::GPSDetail::NSV == 0)
                return ErrorCode::ModuleID::GPSID;
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 0;
                break;
            }
        case ErrorCode::ModuleID::RadioID: 
            // for (auto iter = ErrorCode::Radio::Radio.cbegin(); iter != ErrorCode::Radio::Radio.cend(); iter++) 
            // {
            //     if(getError(*iter))
            //         return (*iter);
            // }
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 1;
                break;
            }
        case ErrorCode::ModuleID::TakeoffID: 
            if(mode == 1)
            {
                if(pos.z < 90)
                    return ErrorCode::ModuleID::TakeoffID;
            }
            if(if_all)
            {
                module++;
                continue;
            }
            else
            {
                return 0;
                break;
            }
        case ErrorCode::ModuleID::Flight_AnomalID: 
            if(mode)
            {
                for (auto iter = ErrorCode::Flight_Anomal::Flight_Anomal.cbegin(); iter != ErrorCode::Flight_Anomal::Flight_Anomal.cend(); iter++) 
                {
                    if(getError_FA(mode, *iter))
                        return (*iter);
                }
                return 0;                
            }
            else
            {
                ROS_WARN("Illigal input!");
                ROS_WARN("Pls change mode or moduleID input!");
            }
            break;
    }
        default:
            ROS_WARN("Wrong errorcode input!");
            return 250;
}

bool getError(uint8_t ErrorID)
{
    return 0;
}

bool getError_FA(uint8_t ErrorID)
{
    return 0;
}

void ErrorCode::callbackState(const nav_msgs::Odometry::ConstPtr &msg)
{
    if_state_update = true;

    pos.x = msg->pose.pose.position.x;
    pos.y = msg->pose.pose.position.y;
    pos.z = msg->pose.pose.position.z;

    vel.vx = msg->twist.twist.linear.x;
    vel.vy = msg->twist.twist.linear.y;
    vel.vz = msg->twist.twist.linear.z;

    //if_resetall = msg->resetALL;

    //mode = msg->mode;

    if(if_mode_update)
    {
        if_update = true;
    }
}

void ErrorCode::callbackMode(const nav_msgs::Odometry::ConstPtr &msg)
{
    if_mode_update = true;
    mode = msg->mode;
    resetall = msg->resetall;

    if(if_state_update)
    {
        if_update = true;
    }

}

ErrorCode::ErrorCode(ros::NodeHandle n) : nh(n), nh_("~"), if_onway_init(false)
{
    nh_.getParam("stateTopic", StateTopic);
    stateSub = nh.subscribe(StateTopic, 3, &ErrorCode::callbackState, this);

    // nh_.getParam("modeSub", ModeTopic);
    // modeSub = nh.subscribe(ModeTopic, 3, &ErrorCode::callbackMode, this);


}

void ErrorCode::Onway()
{
    NodeHandle nh_anodet("~");
    vector<double> pos_bias_threshold, vel_threshold;//vel_threshold: vel_horizontal_min, vel_horizontal_max, vel_vertical_min, vel_vertical_max
                                                     //pos_bias_threshold: pos_x, pos_y, pos_z;
    nh_anodet.getParam("pos_onway", pos_bias_threshold);
    nh_anodet.getParam("vel_onway", vel_threshold);

    ErrorCode::Detection(pos_bias_threshold, vel_threshold, 1);
}

void ErrorCode::Takeoff()
{
    NodeHandle nh_anodet("~");
    vector<double> pos_bias_threshold, vel_threshold;
    nh_anodet.getParam("pos_onway", pos_bias_threshold);
    nh_anodet.getParam("vel_onway", vel_threshold);

    ErrorCode::Detection(pos_bias_threshold, vel_threshold, 2);
}

void ErrorCode::Land()
{
    NodeHandle nh_anodet("~");
    vector<double> pos_bias_threshold, vel_threshold;
    nh_anodet.getParam("pos_onway", pos_bias_threshold);
    nh_anodet.getParam("vel_onway", vel_threshold);

    ErrorCode::Detection(pos_bias_threshold, vel_threshold, 3);
}

void ErrorCode::Detection(vector<double>& pos_bias_threshold,vector<double>& vel_threshold,int mode)
//pos --> position  vel --> velocity    mode --> 1-巡检过程中; 2-起飞过程中; 3-降落过程中
{
    if(mode == 1)
    //当飞机为巡检模式时，不仅检查是否过速掉高等，还检测飞行轨迹与预期轨迹是否相符
    {
        ErrorCode::TrajMatchScore(pos_bias_threshold);
        ErrorCode::Overspeed(vel_threshold);
    }
    else if(mode == 2 || mode == 3)
    //当飞机在起飞或降落时，只检测是否过速掉高
    {
        ErrorCode::Overspeed(vel_threshold);
    }
}

void ErrorCode::Overspeed(vector<double>& vel_threshold)
{  
    double horizontal_vel_threshold[2], vertical_vel_threshold[2], speed_overshot, height_overshot, pos_bias_overshot;//1:min, 2:max
    
    double vel_horizontal_square = vel.vx*vel.vx + vel.vy*vel.vy;
    double vel_vertical = vel.vz;

    horizontal_vel_threshold[0] = vel_threshold[0]*vel_threshold[0];//squared
    horizontal_vel_threshold[1] = vel_threshold[1]*vel_threshold[1];
    vertical_vel_threshold[0] = vel_threshold[2];
    vertical_vel_threshold[1] = vel_threshold[3];

    //Horizontal水平速度检测
    if(vel_horizontal_square > horizontal_vel_threshold[0] && vel_horizontal_square < horizontal_vel_threshold[1])
    {
        ROS_INFO("Horizontal speed is normal.");
    }
    else if(vel_horizontal_square > horizontal_vel_threshold[1])
    {
        speed_overshot = sqrt(vel_horizontal_square) - sqrt(horizontal_vel_threshold[1]);
        ROS_INFO("Horizontal speed too HIGH!");
    }
    else if(vel_horizontal_square < horizontal_vel_threshold[0])
    {
        speed_overshot = sqrt(vel_horizontal_square) - sqrt(horizontal_vel_threshold[1]);
        ROS_INFO("Horizontal speed too LOW!");
    }

    //Vertical垂直速度检测, 也需要加一个位移检测吧。。。位移待定
    if(vel_vertical > vertical_vel_threshold[0] && vel_vertical < vertical_vel_threshold[1])
    {
        ROS_INFO("Vertical speed is normal.");
    }
    else if(vel_vertical > vertical_vel_threshold[1])
    {
        speed_overshot = vel_vertical - vertical_vel_threshold[0];
        ROS_INFO("Vertical speed too HIGH!");
    }
    else if(vel_vertical < vertical_vel_threshold[0])
    {
        speed_overshot = vel_vertical - vertical_vel_threshold[0];
        ROS_INFO("Vertical speed too LOW!");
    }

    if(mode == 1)
    {
        if(!if_onway_init)//刚进入巡检模式
        {
            if_onway_init = true;
            time = ros::Time::now().toSec();
            height_pre = pos.z;
        }
        else
        {
            if(ros::Time::now().toSec() - time > 10.0)
            {
                height = pos.z;
                delta_h = height - height_pre;
                if(delta_h < -10.0)
                {
                    ROS_INFO("掉高！");
                }
            }
        }

        if(vel.vz < 100.0)
        {
            ROS_INFO("高度过低！");
        }
    }
    else if(mode == 2)
    {
        if(!if_onway_init)//刚进入巡检模式
        {
            if_onway_init = true;
            time = ros::Time::now().toSec();
            height_pre = pos.z;
        }
        else
        {
            if(ros::Time::now().toSec() - time > 10.0)
            {
                height = pos.z;
                delta_h = height - height_pre;
                if(height < 60)
                {
                    if(delta_h < 5.0)
                    {
                        ROS_INFO("起飞慢！");
                    }
                }
                else
                {
                    if(delta_h < 0.0)
                    {
                        ROS_INFO("起飞慢！");
                    }
                }
                    
            }
        }
    }
    else if(mode == 3)
    {
        /***************没啥想法****************/
    }
}

void ErrorCode::TrajMatchScore(vector<double>& pos_bias_threshold)
{
    double pos_bias_square = (pos_d.x-pos.x)*(pos_d.x-pos.x) + (pos_d.y-pos.y)*(pos_d.y-pos.y) + (pos_d.z-pos.z)*(pos_d.z-pos.z);
    double pos_bias_overshot;
    if(pos_bias_square > pos_bias_threshold[0]*pos_bias_threshold[0] + pos_bias_threshold[1]*pos_bias_threshold[1] + pos_bias_threshold[2]*pos_bias_threshold[2])
    {
        pos_bias_overshot = sqrt(pos_bias_square);
        ROS_INFO("Trajectory doesn't match well!");
    }
}