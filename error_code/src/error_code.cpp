/*errorcode 250 means wrong input
  errorcode 0 means successful*/



#include "error_code.h"
#include "error_code/mode.h"

using namespace std;
using namespace ros;


// const uint16_t ErrorCode::ifError::SUCCESS                = 0x0000;
// const uint16_t ErrorCode::CommonACK::KEY_ERROR              = 0xFF00;
// const uint16_t ErrorCode::CommonACK::NO_AUTHORIZATION_ERROR = 0xFF01;
// const uint16_t ErrorCode::CommonACK::NO_RIGHTS_ERROR        = 0xFF02;
// const uint16_t ErrorCode::CommonACK::NO_RESPONSE_ERROR      = 0xFFFF;
const uint8_t  ErrorCode::ifError::MOTOR_FAIL_NONE        = 0;
ErrorCode::ifError::ifError.pushback(ErrorCode::ifError::MOTOR_FAIL_NONE);

const uint8_t ErrorCode::Battery::MOTOR_FAIL_BATTERY_CELL_ERROR = 11;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR = 12;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW = 13;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_BATTERY_USER_LOW_LAND = 14;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_BATTERY_SMART_LOW_LAND = 17;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_M600_BAT_TOO_FEW = 78;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_M600_BAT_AUTH_ERR = 79;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_M600_BAT_COMM_ERR = 80;
const uint8_t ErrorCode::Battery::MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1 = 81;
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_BATTERY_CELL_ERROR);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_BATTERY_USER_LOW_LAND);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_BATTERY_SMART_LOW_LAND);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_M600_BAT_TOO_FEW);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_M600_BAT_AUTH_ERR);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_M600_BAT_COMM_ERR);
ErrorCode::Battery::Battery.pushback(ErrorCode::Battery::MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1);

const uint8_t ErrorCode::RTK::MOTOR_FAIL_RTK_INITING = 102;
const uint8_t ErrorCode::RTK::MOTOR_FAIL_RTK_FAIL_TO_INIT = 103;     
ErrorCode::RTK::RTK.pushback(ErrorCode::RTK::MOTOR_FAIL_RTK_INITING);
ErrorCode::RTK::RTK.pushback(ErrorCode::RTK::MOTOR_FAIL_RTK_FAIL_TO_INIT);

const uint8_t ErrorCode::Takeoff::MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE = 30;        
const uint8_t ErrorCode::Takeoff::MOTOR_FAIL_TAKEOFF_EXCEPTION = 94;
const uint8_t ErrorCode::Takeoff::MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED = 99;        
const uint8_t ErrorCode::Takeoff::TAKEOFF_HEIGHT_EXCEPTION = 116;            
ErrorCode::Takeoff::Takeoff.pushback(ErrorCode::Takeoff::MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE);
ErrorCode::Takeoff::Takeoff.pushback(ErrorCode::Takeoff::MOTOR_FAIL_TAKEOFF_EXCEPTION);
ErrorCode::Takeoff::Takeoff.pushback(ErrorCode::Takeoff::MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED);
ErrorCode::Takeoff::Takeoff.pushback(ErrorCode::Takeoff::TAKEOFF_HEIGHT_EXCEPTION);

const uint8_t ErrorCode::Device::MOTOR_FAIL_TOPOLOGY_ABNORMAL = 75;
ErrorCode::Device::Device.pushback(ErrorCode::Device::MOTOR_FAIL_TOPOLOGY_ABNORMAL);

const uint8_t ErrorCode::Compass::MOTOR_FAIL_COMPASS_ABNORMAL = 1;
const uint8_t ErrorCode::Compass::MOTOR_FAIL_COMPASS_CALIBRATING = 8;
ErrorCode::Compass::Compass.pushback(ErrorCode::Compass::MOTOR_FAIL_COMPASS_ABNORMAL);
ErrorCode::Compass::Compass.pushback(ErrorCode::Compass::MOTOR_FAIL_COMPASS_CALIBRATING);

const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION = 5;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_SN_ERROR = 6;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_PREHEATING = 7;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_NO_ATTITUDE = 9;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_ATTI_LIMIT = 21;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_BIAS_LIMIT = 24;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_DISCONNECTED = 61;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_CALIBRATING = 29;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_CALI_SUCCESS = 93;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_IMU_INITING = 26;
const uint8_t ErrorCode::IMU::MOTOR_FAIL_COMPASS_IMU_MISALIGN = 120;
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_SN_ERROR);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_PREHEATING);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_NO_ATTITUDE);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_ATTI_LIMIT);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_BIAS_LIMIT);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_DISCONNECTED);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_CALIBRATING);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_CALI_SUCCESS);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_IMU_INITING);
ErrorCode::IMU::IMU.pushback(ErrorCode::IMU::MOTOR_FAIL_COMPASS_IMU_MISALIGN);

const uint8_t ErrorCode::GPS::MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE = 10;
const uint8_t ErrorCode::GPS::MOTOR_FAIL_GPS_DISCONNECT = 45;
ErrorCode::GPS::GPS.pushback(ErrorCode::GPS::MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE);
ErrorCode::GPS::GPS.pushback(ErrorCode::GPS::MOTOR_FAIL_GPS_DISCONNECT);

const uint8_t ErrorCode::RC::MOTOR_FAIL_RC_NEED_CALI = 76;
const uint8_t ErrorCode::RC::MOTOR_FAIL_RC_CALIBRATING = 62;
const uint8_t ErrorCode::RC::MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE = 63;
const uint8_t ErrorCode::RC::MOTOR_FAIL_RC_QUIT_CALI = 64;
const uint8_t ErrorCode::RC::MOTOR_FAIL_RC_CENTER_OUT_RANGE = 65;
const uint8_t ErrorCode::RC::MOTOR_FAIL_GPS_SIGNATURE_INVALID = 113;
ErrorCode::RC::RC.pushback(ErrorCode::RC::MOTOR_FAIL_RC_NEED_CALI);
ErrorCode::RC::RC.pushback(ErrorCode::RC::MOTOR_FAIL_RC_CALIBRATING);
ErrorCode::RC::RC.pushback(ErrorCode::RC::MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE);
ErrorCode::RC::RC.pushback(ErrorCode::RC::MOTOR_FAIL_RC_QUIT_CALI);
ErrorCode::RC::RC.pushback(ErrorCode::RC::MOTOR_FAIL_RC_CENTER_OUT_RANGE);
ErrorCode::RC::RC.pushback(ErrorCode::RC::MOTOR_FAIL_GPS_SIGNATURE_INVALID); 


const uint8_t ErrorCode::NS::MOTOR_FAIL_NS_ABNORMAL = 74;
ErrorCode::NS::NS.pushback(ErrorCode::NS::MOTOR_FAIL_NS_ABNORMAL);

const uint8_t ErrorCode::Flight_Anomal::IMPACT_IS_DETECTED = 125;
ErrorCode::Flight_Anomal::Flight_Anomal.pushback(ErrorCode::Flight_Anomal::IMPACT_IS_DETECTED);

//把所有的错误id放在一个容器里
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::ifError::ifError.begin(),ErrorCode::ifError::ifError.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::Battery::Battery.begin(),ErrorCode::Battery::Battery.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::RTK::RTK.begin(),ErrorCode::RTK::RTK.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::Takeoff::Takeoff.begin(),ErrorCode::Takeoff::Takeoff.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::Device::Device.begin(),ErrorCode::Device::Device.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::Compass::Compass.begin(),ErrorCode::Compass::Compass.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::IMU::IMU.begin(),ErrorCode::IMU::IMU.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::GPS::GPS.begin(),ErrorCode::GPS::GPS.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::RC::RC.begin(),ErrorCode::RC::RC.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::NS::NS.begin(),ErrorCode::NS::NS.end());
// ErrorCode::AllModule.insert(AllModule.end(),ErrorCode::Flight_Anomal::Flight_Anomal.begin(),ErrorCode::Flight_Anomal::Flight_Anomal.end());

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
            for (auto iter = ErrorCode::Battery::Battery.cbegin(); iter != ErrorCode::Battery::Battery.cend(); iter++) 
            {
                // ret = ret | (uint8_t(getError(*iter))<<cnt);
                // cnt++;
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::RTKID: 
            for (auto iter = ErrorCode::RTK::RTK.cbegin(); iter != ErrorCode::RTK::RTK.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::TakeoffID: 
            for (auto iter = ErrorCode::Takeoff::Takeoff.cbegin(); iter != ErrorCode::Takeoff::Takeoff.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::DeviceID: 
            for (auto iter = ErrorCode::Device::Device.cbegin(); iter != ErrorCode::Device::Device.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::CompassID: 
            for (auto iter = ErrorCode::Compass::Compass.cbegin(); iter != ErrorCode::Compass::Compass.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::IMUID: 
            for (auto iter = ErrorCode::IMU::IMU.cbegin(); iter != ErrorCode::IMU::IMU.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::GPSID: 
            for (auto iter = ErrorCode::GPS::GPS.cbegin(); iter != ErrorCode::GPS::GPS.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::RCID: 
            for (auto iter = ErrorCode::RC::RC.cbegin(); iter != ErrorCode::RC::RC.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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
        case ErrorCode::ModuleID::NSID: 
            for (auto iter = ErrorCode::NS::NS.cbegin(); iter != ErrorCode::NS::NS.cend(); iter++) 
            {
                if(getError(*iter))
                    return (*iter);
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

void AnomalyDetection::callbackState(const nav_msgs::Odometry::ConstPtr &msg)
{
    if_pose_update = true;

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

void AnomalyDetection::callbackMode(const nav_msgs::Odometry::ConstPtr &msg)
{
    if_mode_update = true;
    mode = msg->mode;
    resetall = msg->resetall;

    if(if_pose_update)
    {
        if_update = true;
    }

}

AnomalyDetection::AnomalyDetection(ros::NodeHandle n) : nh(n), nh_("~"), if_onway_init(false)
{
    nh_.getParam("stateTopic", StateTopic);
    stateSub = nh.subscribe(StateTopic, 3, &AnomalyDetection::callbackState, this);

    // nh_.getParam("modeSub", ModeTopic);
    // modeSub = nh.subscribe(ModeTopic, 3, &AnomalyDetection::callbackMode, this);


}

void AnomalyDetection::Onway()
{
    NodeHandle nh_anodet("~");
    vector<double> pos_bias_threshold, vel_threshold;//vel_threshold: vel_horizontal_min, vel_horizontal_max, vel_vertical_min, vel_vertical_max
                                                     //pos_bias_threshold: pos_x, pos_y, pos_z;
    nh_anodet.getParam("pos_onway", pos_bias_threshold);
    nh_anodet.getParam("vel_onway", vel_threshold);

    AnomalyDetection::Detection(pos_bias_threshold, vel_threshold, 1);
}

void AnomalyDetection::Takeoff()
{
    NodeHandle nh_anodet("~");
    vector<double> pos_bias_threshold, vel_threshold;
    nh_anodet.getParam("pos_onway", pos_bias_threshold);
    nh_anodet.getParam("vel_onway", vel_threshold);

    AnomalyDetection::Detection(pos_bias_threshold, vel_threshold, 2);
}

void AnomalyDetection::Land()
{
    NodeHandle nh_anodet("~");
    vector<double> pos_bias_threshold, vel_threshold;
    nh_anodet.getParam("pos_onway", pos_bias_threshold);
    nh_anodet.getParam("vel_onway", vel_threshold);

    AnomalyDetection::Detection(pos_bias_threshold, vel_threshold, 3);
}

void AnomalyDetection::Detection(vector<double>& pos_bias_threshold,vector<double>& vel_threshold,int mode)
//pos --> position  vel --> velocity    mode --> 1-巡检过程中; 2-起飞过程中; 3-降落过程中
{
    if(mode == 1)
    //当飞机为巡检模式时，不仅检查是否过速掉高等，还检测飞行轨迹与预期轨迹是否相符
    {
        AnomalyDetection::TrajMatchScore(pos_bias_threshold);
        AnomalyDetection::Overspeed(vel_threshold);
    }
    else if(mode == 2 || mode == 3)
    //当飞机在起飞或降落时，只检测是否过速掉高
    {
        AnomalyDetection::Overspeed(vel_threshold);
    }
}

void AnomalyDetection::Overspeed(vector<double>& vel_threshold)
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

void AnomalyDetection::TrajMatchScore(vector<double>& pos_bias_threshold)
{
    double pos_bias_square = (pos_d.x-pos.x)*(pos_d.x-pos.x) + (pos_d.y-pos.y)*(pos_d.y-pos.y) + (pos_d.z-pos.z)*(pos_d.z-pos.z);
    double pos_bias_overshot;
    if(pos_bias_square > pos_bias_threshold[0]*pos_bias_threshold[0] + pos_bias_threshold[1]*pos_bias_threshold[1] + pos_bias_threshold[2]*pos_bias_threshold[2])
    {
        pos_bias_overshot = sqrt(pos_bias_square);
        ROS_INFO("Trajectory doesn't match well!");
    }
}