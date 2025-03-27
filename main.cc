#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/applications-module.h>
#include <fstream>
#include <unistd.h>
#include <ns3/flow-monitor-helper.h>  // 必须添加此头文件
#include <string>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Main");

void RecvString(Ptr<Socket> sock)//回调函数
{
    std::cout << "终于接收到了" << std::endl;
}
//----------------------------------------------
// 无人机应用：仅发送图片数据
//----------------------------------------------
class DroneApp : public Application {
public:
    DroneApp() : m_socket(nullptr), m_imageInterval(Seconds(5)) {}
    static TypeId GetTypeId();

    void SetImagePath(const std::string &path) { m_imagePath = path; }
    void SetImageInterval(Time interval) { m_imageInterval = interval; }
    void SetGroundStationIp(Ipv4Address ip) { m_groundStationIp = ip; }

private:
    virtual void StartApplication() override;
    virtual void StopApplication() override;
    void SendImageData();
    
    Ptr<Socket> m_socket;
    Address m_peer;
    std::string m_imagePath;
    Time m_imageInterval;
    EventId m_sendImageEvent;
    Ipv4Address m_groundStationIp; 
};

TypeId DroneApp::GetTypeId() {
    static TypeId tid = TypeId("DroneApp")
        .SetParent<Application>()
        .AddConstructor<DroneApp>()
        .AddAttribute("ImagePath", "Image path", StringValue(""),
                      MakeStringAccessor(&DroneApp::m_imagePath), MakeStringChecker())
        .AddAttribute("ImageInterval", "Transmission interval", TimeValue(Seconds(20)),
                      MakeTimeAccessor(&DroneApp::m_imageInterval), MakeTimeChecker());
    return tid;
}

void DroneApp::StartApplication() {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    //m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), 60000));
    m_peer = InetSocketAddress(m_groundStationIp, 60000);
    m_socket->Connect(m_peer);
    m_sendImageEvent = Simulator::Schedule(Seconds(1), &DroneApp::SendImageData, this); // 1秒后开始发送
}

void DroneApp::StopApplication() {
    if (m_socket) m_socket->Close();
    Simulator::Cancel(m_sendImageEvent);
}

// TODO:发送时候可能有问题,
void DroneApp::SendImageData() {
    if (m_imagePath.empty()) {
        NS_LOG_WARN("Image path not set");
        return;
    }
    
    std::string str = "hello";
    uint8_t buffer[255] ;
    uint32_t len = str.length();
    for(uint32_t i=0;i<len;i++)
    {
        buffer[i]=str[i];//char 与 uint_8逐个赋值
    }
    buffer[len]='\0';
    Ptr<Packet> p = Create<Packet>(buffer,sizeof(buffer));//把buffer写入到包内
    m_socket->Send(p);
    NS_LOG_INFO("发送数据");


    // 8. 调度下一次发送（按需调整）
    m_sendImageEvent = Simulator::Schedule(m_imageInterval, &DroneApp::SendImageData, this);
}

//----------------------------------------------
// 地面站应用：接收原始数据
//----------------------------------------------
class GroundStationApp : public Application {
public:
    Ptr<Socket> m_socket;
    GroundStationApp() : m_socket(nullptr) {}
    static TypeId GetTypeId();

private:
    virtual void StartApplication() override;
    virtual void StopApplication() override;
    void ReceivePacket(Ptr<Socket> socket);
    
    Callback<void, Ptr<Socket> > m_recvCallback; // 新增成员

};

TypeId GroundStationApp::GetTypeId() {
    static TypeId tid = TypeId("GroundStationApp")
        .SetParent<Application>()
        .AddConstructor<GroundStationApp>();
    return tid;
}

void GroundStationApp::StartApplication() {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    
    Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
    Ipv4Address localIp = ipv4->GetAddress(1, 0).GetLocal(); // 接口索引通常从 1 开始
    std::cout << "地面站绑定IP: " << localIp << std::endl;
    m_socket->Bind(InetSocketAddress(localIp, 60000));
    m_socket->SetRecvPktInfo(true);

    // 确保回调生命周期
    m_recvCallback = MakeCallback(&GroundStationApp::ReceivePacket, this);
    m_socket->SetRecvCallback(m_recvCallback);
    NS_LOG_INFO("地面站已启动，监听端口 60000"); // 更新日志
}

void GroundStationApp::StopApplication() {
    if (m_socket) m_socket->Close();
}

void GroundStationApp::ReceivePacket(Ptr<Socket> socket) {
    NS_LOG_INFO("接收到数据");
    Ptr<Packet> packet;
    Address from;
    while ((packet = socket->RecvFrom(from))) {
        // 修复1：取消注释IP解析并添加异常检查
        if (!InetSocketAddress::IsMatchingType(from)) {
            NS_LOG_WARN("Received non-IPv4 packet");
            continue;
        }
        //Ipv4Address ipv4 = InetSocketAddress::ConvertFrom(from).GetIpv4();
        //std::string ipStr = ipv4.ToString();

        // 修复2：生成唯一文件名
        std::ostringstream filenameOSS;
        filenameOSS << "received_from_" 
                   << "_" << static_cast<int>(Simulator::Now().GetSeconds())
                   << ".dat";
        std::string filename = filenameOSS.str();

        // 保存文件
        std::ofstream file(filename, std::ios::binary);
        if (file) {
            uint8_t buffer[packet->GetSize()];
            packet->CopyData(buffer, packet->GetSize());
            file.write(reinterpret_cast<char*>(buffer), packet->GetSize());
            // 修复3：完善日志输出
            NS_LOG_INFO(Simulator::Now() << " Received " << packet->GetSize() 
                       << " bytes from ");
        } else {
            NS_LOG_ERROR("Failed to save: " << filename);
        }
    }
}

//----------------------------------------------
// 主函数
//----------------------------------------------
int main(int argc, char *argv[]) {
    LogComponentEnable("Main", LOG_LEVEL_INFO);

    // 1. 创建节点
    NodeContainer nodes;
    nodes.Create(3); // 节点0-1: 无人机，节点2: 地面站

    // 2. 配置移动模型
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
    // 设置位置
    nodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 10, 0));  // 无人机1
    nodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(10, 0, 0));   // 无人机2
    nodes.Get(2)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 0));    // 地面站

    // 3. 配置无线网络
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    
    YansWifiPhyHelper wifiPhy;
    wifiPhy.Set("TxPowerStart", DoubleValue(20.0));  // 增强信号强度
    wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));
    
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiChannel.AddPropagationLoss("ns3::RangePropagationLossModel", 
                                  "MaxRange", DoubleValue(50.0));
    wifiPhy.SetChannel(wifiChannel.Create());
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

    // 4. 配置网络协议栈
    InternetStackHelper stack;
    stack.Install(nodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // 5. 配置应用层
    // 获取地面站IP
    Ipv4Address groundStationIp = interfaces.GetAddress(2);
    std::cout << "Ground Station IP: " << groundStationIp << std::endl;

    // TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    // Ptr<Socket> Recv_sock = Socket::CreateSocket(nodes.Get(2), tid);
    // // InetSocketAddress addr = InetSocketAddress(Ipv4Address::GetAny(), 10000);
    // InetSocketAddress addr = InetSocketAddress(interfaces.GetAddress(2), 60000);
    // Recv_sock->Bind(addr);
    // Recv_sock->SetRecvCallback(MakeCallback(&RecvString)); //设置回调函数

    // 安装无人机应用
    for (uint32_t i = 0; i < 2; ++i) {
        Ptr<DroneApp> app = CreateObject<DroneApp>();
        app->SetGroundStationIp(groundStationIp);
        app->SetImagePath(i == 0 ? "./scratch/img1.png" : "./scratch/img2.jpg");
        app->SetImageInterval(Seconds(20));
        nodes.Get(i)->AddApplication(app);
    }

    // 安装地面站应用
    Ptr<GroundStationApp> gsApp = CreateObject<GroundStationApp>();
    nodes.Get(2)->AddApplication(gsApp);
   

    // 6. 运行仿真
    FlowMonitorHelper flowMonitor;
    Ptr<FlowMonitor> monitor = flowMonitor.InstallAll(); // 安装监控

    // 6. 运行仿真
    Simulator::Stop(Seconds(100));
    Simulator::Run();

    // 保存流量监控结果
    monitor->SerializeToXmlFile("flow-monitor.xml", true, true);
    Simulator::Destroy();

    return 0;
}
