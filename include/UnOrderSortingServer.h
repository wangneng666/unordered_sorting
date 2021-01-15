#ifndef UNORDERED_SORTING_UNORDERSORTINGSERVER_H
#define UNORDERED_SORTING_UNORDERSORTINGSERVER_H


#include "VisionClient.h"
#include "RobSortingMotion.h"
#include "SolomonCalibrate.h"
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "hirop_msgs/calibrate_set.h"

class UnOrderSortingServer {
public:
    explicit UnOrderSortingServer(ros::NodeHandle* n);
    ~UnOrderSortingServer();

public:
    int start();

private:
    ros::NodeHandle* Node;
    RobSortingMotion robSortingMotion;
    SolomonCalibrate solomonCalibrate;
    VisionClient visionClient;
    ros::ServiceServer server_stepSorting;//单步分拣
    ros::ServiceServer server_continueSorting;//连续分拣
    ros::ServiceServer server_stopSorting;//停止分拣

    ros::ServiceServer server_calibrate_beginCalibrate;//开始标定
    ros::ServiceServer server_calibrate_work;//标定工作
    ros::ServiceServer server_calibrateFinish;//标定结束
    ros::ServiceServer server_getPose;//标定结束
    ros::ServiceServer server_getdata;//标定结束

    bool stepSortingCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool continueSortingCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool stopSortingCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

    bool beginCalibrateCB(hirop_msgs::calibrate_setRequest &req, hirop_msgs::calibrate_setResponse &res);
    bool calibrate_workCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool calibrateFinishCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool getPoseCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);
    bool getdataCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res);

};


#endif //UNORDERED_SORTING_UNORDERSORTINGSERVER_H
