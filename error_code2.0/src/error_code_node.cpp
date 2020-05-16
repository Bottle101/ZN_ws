#include <error_code.h>
#include <onboard_planner/onboard_planner.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "error_code_node");
    ros::NodeHandle n;
    ros::Rate LR(50);
    ros::Duration(5).sleep();
    ErrorCode *FliAnmDet = new ErrorCode(n);
    while (ros::ok())
    {
        if(FliAnmDet->mode == 1)
        {
            FliAnmDet->if_onway_init = 0;
            FliAnmDet->if_land_init = 0;
        }
        else if(FliAnmDet->mode == 2)
        {
            FliAnmDet->if_takeoff_init = 0;
            FliAnmDet->if_land_init = 0;
        }
        else if(FliAnmDet->mode == 3)
        {
            FliAnmDet->if_onway_init = 0;
            FliAnmDet->if_takeoff_init = 0;
        }

        if(FliAnmDet->if_update)
        {
            Error.Battery = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::BatteryID);
            Error.RTK = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::RTKID);
            Error.Compass = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::CompassID);
            Error.IMU = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::IMUID);
            Error.GPS = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::GPSID);
            Error.Radio = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::RadioID);

            if (FliAnmDet->mode == 2 && !FliAnmDet->resetall)//正在巡检
            {
                FliAnmDet->Onway();
            }
            else if((FliAnmDet->mode == 1 || FliAnmDet->mode == 3) && !FliAnmDet->resetall)//正在起飞或者降落的过程中
            {
                if(FliAnmDet->mode == 1)
                {
                    Error.Takeoff = ErrorCode::getErrorCodeModule(ErrorCode::ModuleID::TakeoffID);
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
            FliAnmDet = new ErrorCode(n);
        }
        LR.sleep();
    }

    delete FliAnmDet;

    return 0;
}