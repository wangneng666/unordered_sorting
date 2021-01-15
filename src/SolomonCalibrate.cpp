#include "SolomonCalibrate.h"

#include <utility>

SolomonCalibrate::SolomonCalibrate() {
}

SolomonCalibrate::~SolomonCalibrate() {

}

int SolomonCalibrate::initSet(string ip, int port) {
    VisionIp=ip;
    VisionPort=port;
    return 0;
}

int SolomonCalibrate::beginCalibrate(std::vector<double > toolTcp,int setTotalPicNum) {
    curindex_CalibPic=1;
    totalCalibPic=setTotalPicNum;
    visionClient.connect_to_server(VisionIp.c_str(),VisionPort);
    visionClient.calibrate_init(std::move(toolTcp));
    return 0;
}

int SolomonCalibrate::calibrateCurPose() {
    if(curindex_CalibPic>=totalCalibPic){
        cout<<"标定数量已经足够，无需再进行"<<endl;
        return 0;
    }
    vector<double> cur_pose;
    if(!Hsc3apiInstance::getInstance()->getRobotLocPose(cur_pose)){
        cout<<"当前笛卡尔点位获取失败"<<endl;
        return -1;
    }
    visionClient.calibrate_work(cur_pose);
    curindex_CalibPic++;
    return 0;
}

int SolomonCalibrate::calibrateWork(vector<double> cur_pose) {
//    visionClient.connect_to_server(VisionIp.c_str(),VisionPort);
    visionClient.calibrate_work(cur_pose);
    return 0;
}

int SolomonCalibrate::calibrateFinish() {
//    visionClient.connect_to_server(VisionIp.c_str(),VisionPort);
    visionClient.calibrate_finish();
    return 0;
}

int SolomonCalibrate::getCurCatlibrateIndex() {
    return curindex_CalibPic;
}


