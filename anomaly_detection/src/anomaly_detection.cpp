#include "anomaly_detection.h"

using namespace std;
using namespace ros;

void AnomalyDetection::callbackState(const std_msgs::String::ConstPtr &msg)
{
    if_update = true;

    pos.x = msg->pose.pose.position.x;
    pos.y = msg->pose.pose.position.y;
    pos.z = msg->pose.pose.position.z;

    vel.vx = msg->twist.twist.linear.x;
    vel.vy = msg->twist.twist.linear.y;
    vel.vz = msg->twist.twist.linear.z;

    if_resetall = msg->resetALL;

    mode = msg->mode;
}

AnomalyDetection::AnomalyDetection(ros::NodeHandle n) : nh(n), nh_("~"), if_onway_init(false)
{
    nh_.getParam("stateTopic", StateTopic);
    stateSub = nh.subscribe(StateTopic, 3, &AnomalyDetection::callbackState, this);

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
//pos --> position  vel --> velocity    mode --> 1-Ѳ�������; 2-��ɹ�����; 3-���������
{
    if(mode == 1)
    //���ɻ�ΪѲ��ģʽʱ����������Ƿ���ٵ��ߵȣ��������й켣��Ԥ�ڹ켣�Ƿ����
    {
        AnomalyDetection::TrajMatchScore(pos_bias_threshold);
        AnomalyDetection::Overspeed(vel_threshold);
    }
    else if(mode == 2 || mode == 3)
    //���ɻ�����ɻ���ʱ��ֻ����Ƿ���ٵ���
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

    //Horizontalˮƽ�ٶȼ��
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

    //Vertical��ֱ�ٶȼ��, Ҳ��Ҫ��һ��λ�Ƽ��ɡ�����λ�ƴ���
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
        if(!if_onway_init)//�ս���Ѳ��ģʽ
        {
            if_onway_init = true;
            time = ros::Time::now();
            height_pre = pos.z;
        }
        else
        {
            if(ros::Time::now() - time > 10)
            {
                height = pos.z;
                delta_h = height - height_pre;
                if(delta_h < -10.0)
                {
                    ROS_INFO("���ߣ�");
                }
            }
        }

        if(vel.vz < 100.0)
        {
            ROS_INFO("�߶ȹ��ͣ�");
        }
    }
    else if(mode == 2)
    {
        if(!if_onway_init)//�ս���Ѳ��ģʽ
        {
            if_onway_init = true;
            time = ros::Time::now();
            height_pre = pos.z;
        }
        else
        {
            if(ros::Time::now() - time > 10)
            {
                height = pos.z;
                delta_h = height - height_pre;
                if(height < 60)
                {
                    if(delta_h < 5.0)
                    {
                        ROS_INFO("�������");
                    }
                }
                else
                {
                    if(delta_h < 0.0)
                    {
                        ROS_INFO("�������");
                    }
                }
                    
            }
        }
    }
    else if(mode == 3)
    {
        /***************ûɶ�뷨****************/
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