#ifndef UNORDERED_SORTING_SOLOMONCALIBRATE_H
#define UNORDERED_SORTING_SOLOMONCALIBRATE_H

#include "VisionClient.h"
#include "Hsc3apiInstance.h"

class SolomonCalibrate {
public:
    SolomonCalibrate();
    ~SolomonCalibrate();

private:
    VisionClient visionClient;

    int totalCalibPic;
    int curindex_CalibPic;

    std::string VisionIp;
    int VisionPort;
public:

    int initSet(string ip, int port);

   /***
    * 开始标定
    * @param toolTcp 工具坐标
    * @param setTotalPicNum 设置总共要标定图像数量 20
    * @return
    */
    int beginCalibrate(std::vector<double > toolTcp,int setTotalPicNum);

    /***
     * 标定当前点
     * @return
     */
    int calibrateCurPose();

    /***
     * 标定结束
     * @return
     */
    int calibrateFinish();

    /***
     * 获取标定图像序号
     * @return 返回标定序号
     */
    int getCurCatlibrateIndex();

    int calibrateWork(vector<double> cur_pose);
};


#endif //UNORDERED_SORTING_SOLOMONCALIBRATE_H
