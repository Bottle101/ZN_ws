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

    enum ModuleID {
    AllModule = 0,
    BatteryID = 1,
    RTKID = 2,
    TakeoffID = 3,
    DeviceID = 4,
    CompassID = 5,
    IMUID = 6,
    GPSID = 7,
    RCID = 8,
    NSID = 9,
    RadioID = 10,
    Flight_AnomalID = 11,
    };

    uint8_t getErrorCode(uint8_t mode);
    uint8_t getErrorCode(uint8_t module);
    
    std::vector<uint8_t> AllModule;

    class ifError
    {
        public:
        std::vector<uint8_t> ifError;
        // These error codes would return either from
        // CMDSet Control::Task or from Missions        
        const static uint8_t MOTOR_FAIL_NONE;            
    }

    class Battery
    {
    public:
        std::vector<uint8_t> Battery;
        // const static uint16_t KEY_ERROR;
        // const static uint16_t NO_AUTHORIZATION_ERROR;
        // const static uint16_t NO_RIGHTS_ERROR;
        // const static uint16_t NO_RESPONSE_ERROR;
        
        /*! Error occured in the battery cell. */
        const static uint8_t MOTOR_FAIL_BATTERY_CELL_ERROR;
        /*! Battery communication is abnormal. */
        const static uint8_t MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR;
        /*! Battery voltage is below the minimum allowable value. */
        const static uint8_t MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW;
        /*! The volume (%) is below the second-level power set by user. */
        const static uint8_t MOTOR_FAIL_BATTERY_USER_LOW_LAND;
        /*! Flight contol calculates that current power is only adequate to land.*/
        const static uint8_t MOTOR_FAIL_BATTERY_SMART_LOW_LAND;
        /*! This error will happen only in M600 if the aircraft 
        * detects the battery number is not engouh. <br>
        * Please insert more battery. */
        const static uint8_t MOTOR_FAIL_M600_BAT_TOO_FEW;
        /*! Battery certification failed. <br>
        * Please ask technical assistance for help if repeats after reset. */
        const static uint8_t MOTOR_FAIL_M600_BAT_AUTH_ERR;
        /*! Battery communication is abnormal. <br>
        * Please check the battery connection. */
        const static uint8_t MOTOR_FAIL_M600_BAT_COMM_ERR;
        /*! Battery voltage difference is too large. Please check the battery
        status.*/
        const static uint8_t MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1;
    };

    class RTK
    {
        public:
        std::vector<uint8_t> RTK;
        /*! RTK initialization error. */
        const static uint8_t MOTOR_FAIL_RTK_INITING;
        /*! rtk yaw and magnetometer yaw misaligned */
        const static uint8_t MOTOR_FAIL_RTK_FAIL_TO_INIT;        
    }

    class Takeoff
    {
        public:
        std::vector<uint8_t> Takeoff;
        // /*! The device is not activated. */
        // const static uint8_t MOTOR_FAIL_NOT_ACTIVATED;
        // /*! The status output by any esc is unhealthy. */
        // const static uint8_t MOTOR_FAIL_ESC_ERROR;
        /*! The aircraft's horizontal attitude angle exceeds the limit 
        * angle when requesting automatic takeoff. */
        const static uint8_t MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE;        
        /*! The aircraft has rollover when taking off. <br>
        * Please check the status of the IMU and motors. */
        const static uint8_t MOTOR_FAIL_TAKEOFF_EXCEPTION;
        /*! During automatic take-off, the status of aircraft doesn't 
        * change from "on the ground" to "in the air" in 5s.*/
        const static uint8_t MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED;        
        /*! The height of the takeoff is abnormal. <br>
        * This error happens when the takeoff height relative to ground is up to
        * 100m.*/
        const static uint8_t TAKEOFF_HEIGHT_EXCEPTION;            

    };

    class Device
    {
        public:
        std::vector<uint8_t> Device;
        /*! Each craft has a set of devices to register. <br>
        * It won't take off if a class of device is missing. Please reset and check
        * the connection.*/
        const static uint8_t MOTOR_FAIL_TOPOLOGY_ABNORMAL;
    };

    class Compass
    {
    
        public:
        std::vector<uint8_t> Compass;
        /*! The compass being used appears as follows: <br>
        * (1) The compass data has too much noise. <br>
        * (2) The compass data is stuck. <br>
        * (3) The compass is disconnected. <br>
        * (4) Compass user compilation error. <br>
        * For the flight control of N3, A3 and M600, there are
        * more situations:  <br>
        * (5) The compass is disturbed.  <br>
        * (6) Multiple compasses point different directions. <br>
        * (7) Compass calibration failed. <br>
        * (8) The compass is not calibrated. */
        const static uint8_t MOTOR_FAIL_COMPASS_ABNORMAL;
        /*! Compass is being calibrated. */
        const static uint8_t MOTOR_FAIL_COMPASS_CALIBRATING;
    };

    class IMU
    {
        public:
        std::vector<uint8_t> IMU;
        /*! The IMU being used appears as follows: <br>
        * (1) The accelerometer output exceeds its range. <br>
        * (2) The accelerometer is stuck. <br>
        * (3) The accelerometer data has too much noise. <br>
        * (4) The accelerometer outputs illegal floating numbers. <br>
        * (5) The factory data of IMU has exception. <br>
        * (6) Multiple accelerometers output differently. <br>
        * (7) The temperature of the IMU is too high. <br>
        * (8) The temperature of the IMU is very high. <br>
        * (9) The gyro output exceeds its range. <br>
        * (10) The gyro is stuck. <br>
        * (11) The gyro data has too much noise. <br>
        * (12) The gyro outputs illegal floating numbers. <br>
        * (13) Multiple accelerometers output differently. <br>
        * (14) The temperature control of gyro is abnormal. <br>
        * For the flight control of Inspire 2, there are more situations: <br>
        * (15)The default IMU exception causes the switch to backup IMU.*/
        const static uint8_t MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION;
        /*! The SN status is wrong. */
        const static uint8_t MOTOR_FAIL_IMU_SN_ERROR;
        /*! The IMU being used is preheated and current temperature is not wihin the
        * calibration range. */
        const static uint8_t MOTOR_FAIL_IMU_PREHEATING;
        /*! The attitude data output by navigation system being used is zero.*/
        const static uint8_t MOTOR_FAIL_IMU_NO_ATTITUDE;
        /*! This error is caused by attitude limit of IMU if horizontal 
        * attitude output by navigation system is over 55 degrees when the
        * system powered up for the first time. */
        const static uint8_t MOTOR_FAIL_IMU_ATTI_LIMIT;
        /*! The IMU is too biased if the gyro's bias is over 0.03rad/s
        * or the accelerometer's bias is over 50 mg when first started up.*/
        const static uint8_t MOTOR_FAIL_IMU_BIAS_LIMIT;
        /*! IMU is disconnected. Please ask technical assistance 
        * for help if repeats after reset. */
        const static uint8_t MOTOR_FAIL_IMU_DISCONNECTED;
        /*! The IMU is in calibration or the aircraft should reset after IMU
        calibration.*/
        const static uint8_t MOTOR_FAIL_IMU_CALIBRATING;
        /*! IMU calibration finished. Please reset aircraft.*/
        const static uint8_t MOTOR_FAIL_IMU_CALI_SUCCESS;
        /*! The IMU is initializing.The attitude data of the current
        * navigation system has not converged yet and the height
        * data of the current navigation system is not ready.*/
        const static uint8_t MOTOR_FAIL_IMU_INITING;
        /*! Compass direction is not the same with IMU. */
        const static uint8_t MOTOR_FAIL_COMPASS_IMU_MISALIGN;
    };

    class GPS
    {
        public:
        std::vector<uint8_t> GPS;
        /*! The aircraft is in Novice Mode without gps. */
        const static uint8_t MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE;
        /* The GPS is disconnected. */
        const static uint8_t MOTOR_FAIL_GPS_DISCONNECT;
    };

    class RC
    {
        public:
        std::vector<uint8_t> RC;
        /*! The RC needs calibration. Please calibrate the RC. */
        const static uint8_t MOTOR_FAIL_RC_NEED_CALI;
        /*! RC is in calibration. Please finish rc calibration. */
        const static uint8_t MOTOR_FAIL_RC_CALIBRATING;
        /*! RC calibration has an exception. Please calibrate the RC. */
        const static uint8_t MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE;
        /* RC calibration is unfinished. Please calibrate the RC. */
        const static uint8_t MOTOR_FAIL_RC_QUIT_CALI;
        /* The center value of RC is out of range. */
        const static uint8_t MOTOR_FAIL_RC_CENTER_OUT_RANGE;
        /* GPS signature is invalid because the GPS module 
        * has not received valid signature information for 2s. */
        const static uint8_t MOTOR_FAIL_GPS_SIGNATURE_INVALID;
    };

    class NS
    {
        public:
        std::vector<uint8_t> NS;
        /*! navigation system abnormal */
        const static uint8_t MOTOR_FAIL_NS_ABNORMAL;
    };

    class Radio
    {
        public:
        std::vector<uint8_t> Radio;
    };

    class Flight_Anomal
    {
        public:
        std::vector<uint8_t> Flight_Anomal;
        /*! The aircraft detects an impact if the measured value of
        * accelerometer exceeds 8g near ground.*/
        const static uint8_t IMPACT_IS_DETECTED;
        // PATH_MISSMATCHING = 0x03,
        // VERTICAL_OVERSPEED = 0x04,
        // HORIZONTAL_OVERSPEED = 0x05,
    };


    enum Errcode{
        SUCCESS = 0x00,//避障没加，我不确定那个要不要单独写成一个包
        FOUR_G_DISCONNECTED = 0x01,
        RADIO_DISCONNECTED = 0x02,
        
    };
};








class AnomalyDetection
{
public:
    AnomalyDetection(ros::NodeHandle n);
    //~AnomalyDetection();
    ros::NodeHandle nh, nh_;

    ros::Subscriber stateSub;
    ros::Subscriber modeSub;

    std::string StateTopic;
    std::string ModeTopic;


    Pos pos;
    Pos pos_d;
    Vel vel;

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
    bool if_pose_update = false;
    bool if_mode_update = false;
    bool resetall = false;

    double time;

    double height_pre;
    double height;
    double delta_h;

    int mode = 0;
};
