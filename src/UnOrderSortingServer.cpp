#include "UnOrderSortingServer.h"


UnOrderSortingServer::UnOrderSortingServer(ros::NodeHandle* n):Node(n) {

}

UnOrderSortingServer::~UnOrderSortingServer() {

}

int UnOrderSortingServer::start() {
    server_stepSorting = Node->advertiseService("sorting/stepSorting", &UnOrderSortingServer::stepSortingCB,this);
    server_continueSorting = Node->advertiseService("sorting/continueSorting", &UnOrderSortingServer::continueSortingCB,this);
    server_stopSorting = Node->advertiseService("sorting/stopSorting", &UnOrderSortingServer::stopSortingCB,this);
    server_getPose = Node->advertiseService("sorting/getPose", &UnOrderSortingServer::getPoseCB,this);
    server_getdata = Node->advertiseService("sorting/getdata", &UnOrderSortingServer::getdataCB,this);


    server_calibrate_beginCalibrate = Node->advertiseService("calibrate/initSet", &UnOrderSortingServer::beginCalibrateCB,this);
    server_calibrate_work = Node->advertiseService("calibrate/recordOnce", &UnOrderSortingServer::calibrate_workCB,this);
    server_calibrateFinish = Node->advertiseService("calibrate/caliFinish", &UnOrderSortingServer::calibrateFinishCB,this);


    cout<<"unordered_sorting server start"<<endl;
    return 0;
}

bool UnOrderSortingServer::stepSortingCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    robSortingMotion.setLoopCmd(false);
    return true;
}

bool UnOrderSortingServer::continueSortingCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    robSortingMotion.startRobSortingProg("SORT.PRG");
    robSortingMotion.setLoopCmd(false);
    robSortingMotion.startSorting("AA");
    return true;
}

bool UnOrderSortingServer::stopSortingCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    robSortingMotion.stopRobSortingProg("SORT.PRG");
    return true;
}

bool UnOrderSortingServer::beginCalibrateCB(hirop_msgs::calibrate_setRequest &req, hirop_msgs::calibrate_setResponse &res) {

    std::vector<double > toolTcp=req.robTool_tcp;
    int setTotalPicNum=req.calibrate_num;
    solomonCalibrate.initSet(req.vision_ip,req.port);
    solomonCalibrate.beginCalibrate(toolTcp,setTotalPicNum);
    res.is_success=true;
    return true;
}

bool UnOrderSortingServer::calibrate_workCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    //设备连接
    if(!Hsc3apiInstance::getInstance()->is_connect()){
        Hsc3apiInstance::getInstance()->connect("10.10.56.214",23234);
    }
    //获取当前机器人笛卡尔坐标
    vector<double > pose;
    if(!Hsc3apiInstance::getInstance()->getRobotLocPose(pose)){
        cout<<"机器人笛卡尔点位获取失败"<<endl;
        res.success= false;
        return true;
    }
    solomonCalibrate.calibrateWork(pose);
    res.success=true;
    return true;
}

bool UnOrderSortingServer::calibrateFinishCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    solomonCalibrate.calibrateFinish();
    res.success=true;
    return true;
}

bool UnOrderSortingServer::getPoseCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    visionClient.connect_to_server("10.10.56.100",10000);
    std::queue<std::array<double,6>> detectOjbs;
    visionClient.getObjInfo();
    if(visionClient.getObjPose("AAA",detectOjbs)){
        cout<<"视觉点位获取失败"<<endl;
        return -1;
    }
    array<double, 6> arrayPose = detectOjbs.front();
    while (!detectOjbs.empty()){
        std::array<double,6> a=detectOjbs.front();
        cout<<"["<<a[0]<<","<<a[1]<<","<<a[2]<<","<<a[3]<<","<<a[4]<<","<<a[5]<<"]"<<endl;
        detectOjbs.pop();
    }

    //发送点位给机器人LR寄存器
    //工具坐标设置为-1
    if(!Hsc3apiInstance::getInstance()->setLR(50,-1,arrayPose)){
        return -1;
    }

    return true;
}

bool UnOrderSortingServer::getdataCB(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res) {
    vector<double > pose;
    if(!Hsc3apiInstance::getInstance()->getRobotLocPose(pose)){
        cout<<"机器人笛卡尔点位获取失败"<<endl;
        return -1;
    }
    visionClient.connect_to_server("10.10.56.100",10000);
    char* s1;
    visionClient.getRecvData(&s1);
    std::vector<std::string> valid_dataStr1 = visionClient.splitString(s1, std::string("\n"));
    cout<<"接收数据："<<valid_dataStr1[0]<<endl;
    visionClient.setCapturePose(pose);
    sleep(1);
    char* s;
    visionClient.getRecvData(&s);//002 ok ,not002 failed
    std::vector<std::string> valid_dataStr2 = visionClient.splitString(s, std::string("\n"));

    cout<<"接收数据："<<valid_dataStr2[0]<<endl;

    visionClient.getObjInfo();
    sleep(1);
    char* s2;
    visionClient.getRecvData(&s2);//002 ok ,not002 failed
    std::vector<std::string> valid_dataStr3 = visionClient.splitString(s2, std::string("\n"));
    cout<<"接收数据："<<valid_dataStr3[0]<<endl;
    return true;
}

