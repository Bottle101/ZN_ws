#include <anomaly_detection.h>
#include <onboard_planner/onboard_planner.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "anomaly_detection_node");
    ros::NodeHandle n;
    ros::Rate LR(3000);
    ros::Duration(5).sleep();
    AnomalyDetection *FliAnmDet = new AnomalyDetection(n);
    while (ros::ok())
    {
        if(FliAnmDet->mode == 1)
        {
            FliAnmDet->if_takeoff_init = 0;
            FliAnmDet->if_land_init = 0;
        }
        else if(FliAnmDet->mode == 2)
        {
            FliAnmDet->if_onway_init = 0;
            FliAnmDet->if_land_init = 0;
        }
        else if(FliAnmDet->mode == 3)
        {
            FliAnmDet->if_onway_init = 0;
            FliAnmDet->if_takeoff_init = 0;
        }

        if(FliAnmDet->if_update)
        {
            if (FliAnmDet->mode == 1 && !FliAnmDet->resetall)//正在巡检
            {
                FliAnmDet->Onway();
            }
            else if((FliAnmDet->mode == 2 || FliAnmDet->mode == 3) && !FliAnmDet->resetall)//正在起飞或者降落的过程中
            {
                if(FliAnmDet->mode == 2)
                {
                    FliAnmDet->Takeoff();
                }
                else if(FliAnmDet->mode == 3)
                {
                    FliAnmDet->Land();
                }
            }
            else if(FliAnmDet->resetall)
            {
                break;
            }
        }
        

        ros::spinOnce();

        if(FliAnmDet->resetall)
        {
            delete FliAnmDet;
            FliAnmDet = new AnomalyDetection(n);
        }
        LR.sleep();
    }

    delete FliAnmDet;

    return 0;
}