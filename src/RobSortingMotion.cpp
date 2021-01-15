#include "RobSortingMotion.h"

RobSortingMotion::RobSortingMotion() {
    isRun_PRG= false;
    isLoop_PRG= false;
}

RobSortingMotion::~RobSortingMotion() {

}

int RobSortingMotion::startRobSortingProg(string programName) {
    int ret=0;
    //设备连接
    if(!Hsc3apiInstance::getInstance()->is_connect()){
        ret=Hsc3apiInstance::getInstance()->connect("10.10.56.214",23234);
    }
    if(ret!=0){
        cout<<"机器人连接失败"<<endl;
        return -1;
    } else{
        cout<<"机器人连接成功"<<endl;
    }
    //加载一次机器人程序
    ret = Hsc3apiInstance::getInstance()->setStartUpProject(programName);
    if(ret!=0){
        cout<<"机器人程序启动失败"<<endl;
        return -2;
    }
    isRun_PRG= true;
    this_thread::sleep_for(chrono::seconds(2));
    return 0;
}


int RobSortingMotion::stopRobSortingProg(string programName) {
    stop= true;
    Hsc3apiInstance::getInstance()->setR(48,0);
    Hsc3apiInstance::getInstance()->setR(50,0);
    Hsc3apiInstance::getInstance()->setR(60,0);
    Hsc3apiInstance::getInstance()->setStopProject(programName);
    return 0;
}

int RobSortingMotion::setLoopCmd(bool isLoop) {
    if(isLoop){
        isLoop_PRG=true;
        Hsc3apiInstance::getInstance()->setR(48,0.0);
    } else{
        isLoop_PRG= false;
        Hsc3apiInstance::getInstance()->setR(48,1.0);
    }
    return 0;
}

int RobSortingMotion::startSorting(std::string photo_mode) {
    visionClient.connect_to_server("10.10.56.100",10000);
    print_getRecvData();
    do{
        cout<<"分拣程序执行中"<<endl;
        //请求拍照信号
        if(waitRSignal(50,1)!=0){
            return -1;
        }
//        //视觉客户端设置工具TCP
//        vector<double > rob_tcp{0,0,100,0,0};
//        if(visionClient.calibrate_init(rob_tcp)!=0){
//            cout<<"工具tcp设置失败"<<endl;
//            return -1;
//        }
        //获取当前机器人笛卡尔坐标
        vector<double > pose;
        if(!Hsc3apiInstance::getInstance()->getRobotLocPose(pose)){
            cout<<"机器人笛卡尔点位获取失败"<<endl;
            return -1;
        }
        sleep(1);
        //视觉客户端拍照
        if(visionClient.setCapturePose(pose)!=0){
            cout<<"视觉拍照错误"<<endl;
            return -1;
        }
        print_getRecvData();
        sleep(1);
        //视觉客户端识别
        if(visionClient.getObjInfo()!=0){
            cout<<"视觉识别错误"<<endl;
            return -1;
        }
        //视觉客户端点位获取
//        visionClient.connect_to_server("10.10.56.100",10000);
        std::queue<std::array<double,6>> detectOjbs;
        if(visionClient.getObjPose(photo_mode,detectOjbs)){
            cout<<"视觉点位获取失败"<<endl;
            return -1;
        }
        //发送点位给机器人LR寄存器
        array<double, 6> arrayPose = detectOjbs.front();
        detectOjbs.pop();
        //工具坐标设置为-1
        if(!Hsc3apiInstance::getInstance()->setLR(50,-1,arrayPose)){
            return -1;
        }
        //通知拍照识别结束
        notifyPhotoFinish();

    }while(isLoop_PRG);
    cout<<"分拣程序执行完毕"<<endl;
    return 0;
}

int RobSortingMotion::print_getRecvData(){
    char* s;
    visionClient.getRecvData(&s);
    std::vector<std::string> valid_dataStr = visionClient.splitString(s, std::string("\n"));
    cout<<"接收数据："<<valid_dataStr[0]<<endl;
}

bool RobSortingMotion::reqSignal_Photo() {
    int32_t R=50;
    double value=-1;
    Hsc3apiInstance::getInstance()->getR(R,value);
    return value == 1.0;
}

int RobSortingMotion::notifyPhotoFinish() {
    if(Hsc3apiInstance::getInstance()->setR(60,1.0)!=0){
        return -1;
    }
    return 0;
}

int RobSortingMotion::waitRSignal(int32_t rValue,double value) {
    stop= false;
    double revSignalFinish=0;
    int time_count=0;
    while (!stop)
    {
        //获取寄存器的值，确保动作已经完成
        if(Hsc3apiInstance::getInstance()->getR(rValue, revSignalFinish)!=0){
            cout<<"Hsc3apiInstance::getInstance()->getR 获取失败"<<endl;
            return -1;
        }
//        cout<<"获取R["<<rValue<<"]的值"<<revSignalFinish<<endl;
        if(revSignalFinish==value)
        {
            Hsc3apiInstance::getInstance()->setR(rValue,0.0);
            cout<<"检测到动作完成"<<endl;
            break;
        }
        //超时判断
        if(time_count>100)
        {
            cout<<"机器人运行超时"<<endl;
            return -2;
        }
        time_count++;
        this_thread::sleep_for(chrono::seconds(1));
    }
    if(stop){
        return -3;
    }
    return 0;
}
