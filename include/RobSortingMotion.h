#ifndef UNORDERED_SORTING_ROBSORTINGMOTION_H
#define UNORDERED_SORTING_ROBSORTINGMOTION_H

#include "Hsc3apiInstance.h"
#include "VisionClient.h"
#include "atomic"

//连续信号R[48] 1非连续 0默认连续
//请求拍照信号 R[50] 1请求
//等待接受点位信号R[60] 1已接收


class RobSortingMotion {
public:
    RobSortingMotion();
    ~RobSortingMotion();
private:
    atomic<bool > isRun_PRG;
    atomic<bool > isLoop_PRG;
    atomic<bool > stop;
    VisionClient visionClient;
public:
    /***
     * 启动机器人程序
     * @param programName
     * @return
     */
    int startRobSortingProg(string programName);

    /***
     * 停止机器人程序
     * @param programName
     * @return
     */
    int stopRobSortingProg(string programName);

    /***
     * 设置循环命令
     * @param isLoop 是否是循环抓取
     * @return
     */
    int setLoopCmd(bool isLoop);

    /***
     * 开始分拣
     * @param photo_mode
     * @return
     */
    int startSorting(std::string photo_mode);

    /***
     * 是否请求去拍照
     * @return
     */
    bool reqSignal_Photo();

    /***
     * 通知拍照完成
     * @return
     */
    int notifyPhotoFinish();


    int waitRSignal(int32_t rValue, double value);

    int print_getRecvData();
};


#endif //UNORDERED_SORTING_ROBSORTINGMOTION_H
