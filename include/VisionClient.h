#pragma once

#include <stdio.h>
#include<unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include<stdlib.h>
#include<sys/stat.h>
#include<time.h>
#include <stdarg.h>
#include <vector>
#include <iostream>
#include <queue>
#include <memory>
class VisionClient
{
private:
    int m_clientfd;
public:
    VisionClient();
    ~VisionClient();

public:
  /***
   * socket连接
   * @param serveip
   * @param port
   * @return
   */
   bool connect_to_server(const char *serveip,const int port);

  /***
   * 标定初始化 "resetv2r,0,0,0,0,0,0"
   * @param toolTcp工具TCP设定
   * @return
   */
   int calibrate_init(std::vector<double > toolTcp);

  /***
   * 标定工作，移动一次发送机器人世界坐标并拍照
   * @param robPose [0,0,0,0,0,0] v2r,
   * @return
   */
   int calibrate_work(std::vector<double > robPose);

  /***
   * 标定结束v2rfinished
   * @return
   */
   int calibrate_finish();

  /***
   * 设置机器人拍照位置
   * @param robPhotoPose 机器人拍照位置点 capture,x,y,z,rx,ry,rz
   * @return
   */
   int setCapturePose(std::vector<double > robPhotoPose);

  /***
   *  getObjInfo
   * @return
   */
   int getObjInfo();

  /***
   * 获取识别出目标点集合
   * @param photo_mode 拍照模式，AA,BB,CC
   * @param objPoseList 目标点集合
   * @return
   */
   int getObjPose(std::string photo_mode,std::queue<std::array<double,6>>& detectOjbs);


   int getRecvData(char** buff);

    /***
  * 字符串分割
  * @param s
  * @param c
  * @return
  */
    std::vector<std::string> splitString(const std::string &s, const std::string c);

private:
  /***
   * 接受数据
   * @param buf
   * @param len
   * @return
   */
   int Recv(void *buf, size_t len);
  /***
   * 发送数据
   * @param buf
   * @param len
   * @return
   */
   int Send(const void *buf, size_t len);

   /***
    * 是否连接
    * @return
    */
   bool isConnect();

   /***
    * 关闭socket
    */
   void shutdownSocket();

   /***
    * 解析数据
    * @param data
    * @param detectOjbs
    */
   void parssData(const std::string &data, std::queue<std::array<double,6>>& detectOjbs);



};


