#include <VisionClient.h>
VisionClient::VisionClient()
{
    m_clientfd=0;
}

VisionClient::~VisionClient()
{
    if(m_clientfd>0)  close(m_clientfd);
}

bool VisionClient::connect_to_server(const char *serveip,const int port)
{
    m_clientfd=socket(AF_INET,SOCK_STREAM,0);
    struct hostent* h;
//    memset(h,0,sizeof(hostent));
    h=gethostbyname(serveip);

    if(h==0)
    {
        close(m_clientfd);
        m_clientfd=0;
        return false;
    }

    struct sockaddr_in serveaddr;
    memset(&serveaddr,0,sizeof(serveaddr));
    serveaddr.sin_family=AF_INET;   //ipv4协议族
    serveaddr.sin_port=htons(port);
    memcpy(&serveaddr.sin_addr,h->h_addr,h->h_length);

    if(connect(m_clientfd,(struct sockaddr *)&serveaddr,sizeof(serveaddr))!=0)
    {
        perror("connect failed ");
        close(m_clientfd);
        return false;
    }
//    std::cout<<"m_clientfd: "<<m_clientfd<<std::endl;
    perror("connect sucess ");

    return true;
}

int VisionClient::Recv(void *buf,size_t len)
{
//    memset(buf,0,len);

    return recv(m_clientfd,buf,len,0);
}

int VisionClient::Send(const void *buf,size_t len)
{
    return send(m_clientfd,buf,len,0);
}

bool VisionClient::isConnect()
{
    if(m_clientfd > 0 )
        return true;
    else{
        perror("isConnect error ");
        return  false;
    }
}

void VisionClient::shutdownSocket()
{
     if(m_clientfd > 0 )
     {
        close(m_clientfd);
     }
}

int VisionClient::calibrate_init(std::vector<double> toolTcp) {
    std::string sendString="resetv2r";
    for (int i = 0; i <toolTcp.size(); ++i) {
        sendString+=","+std::to_string(toolTcp[i]);
    }
    char *sendline= const_cast<char*>(sendString.c_str());
    Send(sendline, strlen(sendline));
    return 0;
}

int VisionClient::calibrate_work(std::vector<double> robPose) {
    std::string sendString="v2r";
    for (int i = 0; i <robPose.size(); ++i) {
        sendString+=","+std::to_string(robPose[i]);
    }
    char *sendline= const_cast<char*>(sendString.c_str());
    Send(sendline, strlen(sendline));
    return 0;
}

int VisionClient::calibrate_finish() {
    std::string sendString="v2rfinished";
    char *sendline= const_cast<char*>(sendString.c_str());
    Send(sendline, strlen(sendline));
    return 0;
}

int VisionClient::setCapturePose(std::vector<double> robPhotoPose) {
    std::string sendString="Capture3D";
    for (int i = 0; i <robPhotoPose.size(); ++i) {
        sendString+=","+std::to_string(robPhotoPose[i]);
    }
    char *sendline= const_cast<char*>(sendString.c_str());
    int i1 = Send(sendline, strlen(sendline));
//    std::cout<<"send ret: "<<i1<<std::endl;
    return 0;
}

int VisionClient::getObjInfo() {
    std::string sendString="getobjectinfo";
//    std::string sendString="recognize";
    char *sendline= const_cast<char*>(sendString.c_str());
    Send(sendline, strlen(sendline));
    return 0;
}

int VisionClient::getObjPose(std::string photo_mode, std::queue<std::array<double,6>>& detectOjbs) {
    if(!isConnect()){
        return -1;
    }
    char  buff[1000];
    int nRet;
    std::cout<<"m_clientfd: "<<m_clientfd<<std::endl;
    nRet = recv(m_clientfd, buff,1000,0 );
    if (nRet <= 0){
        std::cout<<"recv failed--  "<<nRet<<std::endl;
        return -1;
    }
    std::string data =  buff;
    parssData(data,detectOjbs);

    return 0;
}

int VisionClient::getRecvData(char **_buff) {
        if(!isConnect()){
            return -1;
        }
        char  buff[1000];
        int nRet;

//        std::cout<<"m_clientfd: "<<m_clientfd<<std::endl;
        nRet = recv(m_clientfd, buff,1000,0 );
        if (nRet <= 0){
            std::cout<<"recv failed--"<<nRet<<std::endl;
            return -1;
        }
//        std::cout<<buff<<std::endl;
        *_buff=buff;

    return 0;
}

//void VisionClient::parssData(const std::string &data,std::queue<std::array<double,6>>& detectOjbs) {
//    std::vector<std::string> dataStr = splitString(data, std::string(","));
//    if (dataStr.size() == 0)
//        return;
//    std::string first = dataStr[0];
//    dataStr[0] = first.substr(1, first.length());
//
//    std::string end = dataStr[dataStr.size() - 1];
//    dataStr[dataStr.size() - 1] = end.substr(0, end.length() - 1);
//
//    //
//    detectOjbs = std::queue<std::array<double, 6>>();
//    for (int index = 0; index < dataStr.size() / 6; index++)
//    {
//        std::array<double, 6> data;
////        for (int j = 0; j < 3; j++) {
//        for (int j = 0; j < 6; j++)
//        {
//
//            try {
//                data[j] = stod(dataStr[index * 6 + j]);
//            } catch (std::exception &e) {
//                std::cout << "stod err " << e.what() << std::endl;
//                return;
//            }
//        }
////        if (data[0] == 0 && data[1] == 0)
////            continue;
//
////        data[2] -= 10;
////        data[3] = -180;
////        data[4] = 0;
////        data[5] = 180;
//        detectOjbs.push(data);
//    }
//    std::cout << "# parssData now detectOjbs size: " << detectOjbs.size() << std::endl;
//}

void VisionClient::parssData(const std::string &data,std::queue<std::array<double,6>>& detectOjbs) {
    std::vector<std::string> valid_dataStr = splitString(data, std::string("\n"));
    std::vector<std::string> split_dataStr = splitString(valid_dataStr[0], std::string(";"));
    if (split_dataStr.size() == 0)
        return;
    //
    detectOjbs = std::queue<std::array<double, 6>>();
    for (int index = 0; index < split_dataStr.size(); index++)
    {
        std::vector<std::string> posedataStr = splitString(split_dataStr[index], std::string(","));
        std::array<double, 6> data;
        for (int j = 0; j < 6; j++)
        {

            try {
                data[j] = stod(posedataStr[j]);
            } catch (std::exception &e) {
                std::cout << "stod err " << e.what() << std::endl;
                return;
            }
        }
//        if (data[0] == 0 && data[1] == 0)
//            continue;

//        data[2] -= 10;
//        data[3] = -180;
//        data[4] = 0;
//        data[5] = 180;
        detectOjbs.push(data);
    }
    std::cout << "# parssData now detectOjbs size: " << detectOjbs.size() << std::endl;
}



std::vector<std::string> VisionClient::splitString(const std::string &s, const std::string c)
{
    std::string::size_type pos1,pos2;
    std::vector<std::string> v;
    pos1 = 0;
    pos2 = s.find(c);
    while(pos2 != std::string::npos){
        v.push_back(s.substr(pos1,pos2 - pos1));
        pos1 = pos2 + c.size();
        pos2 = s.find(c,pos1);
    }
    if(pos1 != s.length()){
        v.push_back(s.substr(pos1));
    }
    return v;
}


