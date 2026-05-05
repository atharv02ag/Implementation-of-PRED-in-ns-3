#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include <fstream>
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("IncastFixed");

static const uint32_t N_SENDERS      = 8;       // number of transmitters (N in N-to-1)
static const double TRAFFIC_TIME     = 1.00;      
static const double SIM_TIME         = 2 + TRAFFIC_TIME;      
static const double TARGET_LOAD      = 0.6;      // fraction of link capacity [0.1 – 0.9]
static const uint32_t MIN_MOUSE_BYTES     =  3000;  //3 KB
static const uint32_t MAX_MOUSE_BYTES     = 6000;   //6 KB
static const uint32_t MIN_ELEPHANT_BYTES  = 30000;  // 30 KB
static const uint32_t MAX_ELEPHANT_BYTES  = 600000; // 600 KB
static const double MOUSE_FRACTION   = 0.9;    
// For fixed ECN threshold, set both MIN_K and MAX_K equal to each other.
static const uint32_t MIN_K          = 70;       
static const uint32_t MAX_K          = 70;       
                                                  
static const uint32_t QUEUE_LIMIT_PKTS = 1000;   // max switch buffer depth (packets)


// ─────────────────────────────────────────────────────────
//  Per-flow tracking record
// ─────────────────────────────────────────────────────────
struct FlowRecord {
    Time     startTime;
    uint32_t expectedBytes;
    bool     isMouse;
    bool     completed;
    uint32_t bytesReceived;
};

std::map<uint32_t, FlowRecord> flowTracker;   // keyed by dst port
std::ofstream queueLog;
std::ofstream fctLog;

// ─────────────────────────────────────────────────────────
//  Rx callback — FCT written in microseconds so sub-ms
//  mouse flows are not silently rounded to 0.
// ─────────────────────────────────────────────────────────
void PacketReceivedCallback(Ptr<PacketSink> sink,
                            uint32_t       port,
                            Ptr<const Packet> /*packet*/,
                            const Address& /*from*/)
{
    auto& rec = flowTracker[port];
    if (rec.completed) return;

    uint32_t totalRx = sink->GetTotalRx();
    if (totalRx >= rec.expectedBytes) {
        rec.completed = true;
        Time fct = Simulator::Now() - rec.startTime;

        fctLog << Simulator::Now().GetMicroSeconds() << ","
               << (rec.isMouse ? "Mouse" : "Elephant") << ","
               << rec.expectedBytes << ","
               << fct.GetMicroSeconds()
               << "\n";
    }
}

// ─────────────────────────────────────────────────────────
//  Queue monitor at 50 µs granularity
// ─────────────────────────────────────────────────────────
void MonitorQueue(Ptr<QueueDisc> qd, double stopTime)
{
    if (Simulator::Now().GetSeconds() >= stopTime) return;

    queueLog << Simulator::Now().GetMicroSeconds() << ","
             << qd->GetCurrentSize().GetValue()
             << "\n";

    Simulator::Schedule(MicroSeconds(50), &MonitorQueue, qd, stopTime);
}

// ─────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    // Allow command-line overrides of the key parameters defined above.
    // All other parameters must be changed in the "TUNABLE PARAMETERS" block.
    double load      = TARGET_LOAD;
    double traffTime = TRAFFIC_TIME;
    double simTime   = SIM_TIME;

    CommandLine cmd;
    cmd.AddValue("load",        "Target network load [0.1–0.9]",   load);
    cmd.AddValue("trafficTime", "Flow injection window (seconds)",  traffTime);
    cmd.AddValue("simTime",     "Total simulation time (seconds)",  simTime);
    cmd.Parse(argc, argv);

    // ── TCP / DCTCP / ECN ──────────────────────────────────
    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       StringValue("ns3::TcpDctcp"));
    Config::SetDefault("ns3::TcpSocketBase::UseEcn",
                       StringValue("On"));
    Config::SetDefault("ns3::TcpSocket::SegmentSize",   UintegerValue(1460));
    Config::SetDefault("ns3::TcpSocket::SndBufSize",    UintegerValue(16777216));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize",    UintegerValue(16777216));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd",   UintegerValue(10));

    Config::SetDefault("ns3::RedQueueDisc::UseEcn",      BooleanValue(true));
    Config::SetDefault("ns3::RedQueueDisc::UseHardDrop", BooleanValue(false));

    // ── Nodes ──────────────────────────────────────────────
    NodeContainer senders;    senders.Create(N_SENDERS);
    NodeContainer switchNode; switchNode.Create(1);
    NodeContainer receiver;   receiver.Create(1);

    // ── Links ──────────────────────────────────────────────
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",    StringValue("10us"));

    // Important to set small DropTail queue
    p2p.SetQueue("ns3::DropTailQueue", "MaxSize", StringValue("8p"));

    NetDeviceContainer senderDevices, switchDevices;
    for (uint32_t i = 0; i < N_SENDERS; ++i) {
        NetDeviceContainer lnk = p2p.Install(senders.Get(i), switchNode.Get(0));
        senderDevices.Add(lnk.Get(0));
        switchDevices.Add(lnk.Get(1));
    }
    NetDeviceContainer bottleneckLink =
        p2p.Install(switchNode.Get(0), receiver.Get(0));

    // ── Internet stack ─────────────────────────────────────
    InternetStackHelper stack;
    stack.Install(senders);
    stack.Install(switchNode);
    stack.Install(receiver);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    for (uint32_t i = 0; i < N_SENDERS; ++i) {
        NetDeviceContainer pair;
        pair.Add(senderDevices.Get(i));
        pair.Add(switchDevices.Get(i));
        address.Assign(pair);
        address.NewNetwork();
    }
    Ipv4InterfaceContainer bottleneckIfaces = address.Assign(bottleneckLink);
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // Remove pfifo_fast from all sender→switch links (no qdisc needed there)
    TrafficControlHelper tch;
    for (uint32_t i = 0; i < N_SENDERS; ++i) {
        NetDeviceContainer senderPair;
        senderPair.Add(senderDevices.Get(i));
        senderPair.Add(switchDevices.Get(i));
        tch.Uninstall(senderPair);
    }

    // ── Install RED QueueDisc on switch egress toward receiver ─
    tch.Uninstall(bottleneckLink);

    tch.SetRootQueueDisc(
        "ns3::RedQueueDisc",
        "LinkBandwidth", StringValue("10Gbps"),
        "LinkDelay",     StringValue("10us"),
        "MinTh",         DoubleValue(MIN_K),
        "MaxTh",         DoubleValue(MAX_K),
        "QW",            DoubleValue(1.0), //for instantaneous queue size measurement (NO EWMA)
        "MaxSize",       QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, QUEUE_LIMIT_PKTS)),
        "UseEcn",        BooleanValue(true),
        "UseHardDrop",   BooleanValue(false));

    QueueDiscContainer qd = tch.Install(bottleneckLink.Get(0));

    // ── Random variables ───────────────────────────────────
    // Flow inter-arrival: Poisson process (exponential gaps)
    // Flow size: uniform within each class range (randomized)
    double avgSizeBytes = MOUSE_FRACTION * 0.5 * (MIN_MOUSE_BYTES    + MAX_MOUSE_BYTES)
                        + (1.0 - MOUSE_FRACTION) * 0.5 * (MIN_ELEPHANT_BYTES + MAX_ELEPHANT_BYTES);
    double targetBps    = (load * 10e9) / N_SENDERS; // per-sender load in bits/s
    double lambda       = targetBps / (avgSizeBytes * 8.0);

    Ptr<ExponentialRandomVariable> expVar = CreateObject<ExponentialRandomVariable>();
    expVar->SetAttribute("Mean", DoubleValue(1.0 / lambda));

    Ptr<UniformRandomVariable> uniVar = CreateObject<UniformRandomVariable>();

    // Separate uniform RVs for flow size randomization within each class
    Ptr<UniformRandomVariable> mouseSizeVar    = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> elephantSizeVar = CreateObject<UniformRandomVariable>();

    // ── Open log files ─────────────────────────────────────
    std::ostringstream fctName, qName;
    fctName << "fct_ecn_incast_load_" << load << ".csv";
    qName << "queue_ecn_incast_load_" << load << ".csv";
    queueLog.open(qName.str());
    queueLog << "Time_us,Queue_Pkts\n";
    fctLog.open(fctName.str());
    fctLog << "CompletionTime_us,FlowType,SizeBytes,FCT_us\n";

    // ── Schedule flows ─────────────────────────────────────
    uint32_t port = 5000;

    for(uint32_t t = 0; t < N_SENDERS; ++t){
        double   currentTime = 0.1;

        while (currentTime < traffTime) {
            bool isMouse = (uniVar->GetValue() < MOUSE_FRACTION);

            // Randomize flow size uniformly within the class range
            uint32_t flowSize;
            if (isMouse) {
                flowSize = static_cast<uint32_t>(
                    mouseSizeVar->GetValue(MIN_MOUSE_BYTES, MAX_MOUSE_BYTES));
            } else {
                flowSize = static_cast<uint32_t>(
                    elephantSizeVar->GetValue(MIN_ELEPHANT_BYTES, MAX_ELEPHANT_BYTES));
            }

            uint32_t sid = uniVar->GetInteger(0, N_SENDERS - 1);

            // Sink
            Address sinkAddr(InetSocketAddress(Ipv4Address::GetAny(), port));
            PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkAddr);
            ApplicationContainer sinkApp = sinkHelper.Install(receiver.Get(0));
            sinkApp.Start(Seconds(currentTime - 0.0001));
            sinkApp.Stop(Seconds(simTime));

            Ptr<PacketSink> sinkPtr = DynamicCast<PacketSink>(sinkApp.Get(0));
            sinkPtr->TraceConnectWithoutContext(
                "Rx",
                MakeBoundCallback(&PacketReceivedCallback, sinkPtr, port));

            // Source
            Address remoteAddr(
                InetSocketAddress(bottleneckIfaces.GetAddress(1), port));
            BulkSendHelper srcHelper("ns3::TcpSocketFactory", remoteAddr);
            srcHelper.SetAttribute("MaxBytes", UintegerValue(flowSize));
            ApplicationContainer srcApp = srcHelper.Install(senders.Get(sid));
            srcApp.Start(Seconds(currentTime));
            srcApp.Stop(Seconds(simTime));

            flowTracker[port] = {Seconds(currentTime), flowSize, isMouse, false, 0};

            currentTime += expVar->GetValue();
            port++;
        }
    }

    // ── Queue monitor and run ──────────────────────────────
    Simulator::Schedule(Seconds(0.1), &MonitorQueue, qd.Get(0), simTime);

    std::cout << "═══════════════════════════════════════════\n"
              << "  N-to-1 Incast Simulation\n"
              << "═══════════════════════════════════════════\n"
              << "  Senders      : " << N_SENDERS           << "\n"
              << "  Load         : " << load * 100          << " %\n"
              << "  λ per sender(flows/s)  : " << lambda              << "\n"
              << "  Total flows  : " << flowTracker.size()  << "\n"
              << "  Traffic time : " << traffTime           << " s\n"
              << "  Sim time     : " << simTime             << " s\n"
              << "  ECN K: " << MIN_K  << " pkts\n"
              << "  Mouse range  : [" << MIN_MOUSE_BYTES    << ", "
                                      << MAX_MOUSE_BYTES    << "] B\n"
              << "  Elephant range: [" << MIN_ELEPHANT_BYTES << ", "
                                       << MAX_ELEPHANT_BYTES << "] B\n"
              << "═══════════════════════════════════════════\n";

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    queueLog.close();
    fctLog.close();

    // After Simulator::Run(), query the RED disc stats
    Ptr<QueueDisc> redDisc = qd.Get(0);
    QueueDisc::Stats stats = qd.Get(0)->GetStats();

    // Print total drops
    std::cout << "Total enqueued : " << stats.nTotalEnqueuedPackets << "\n"
            << "Total dequeued : " << stats.nTotalDequeuedPackets << "\n"
            << "Total dropped  : " << stats.nTotalDroppedPackets  << "\n";

    // nMarkedPackets is a map<string, uint32_t> keyed by reason
    for (auto& kv : stats.nMarkedPackets) {
        std::cout << "Marked [" << kv.first << "] : " << kv.second << "\n";
    }

    // Similarly for drops by reason
    for (auto& kv : stats.nDroppedPacketsBeforeEnqueue) {
        std::cout << "Dropped before enqueue [" << kv.first << "] : " << kv.second << "\n";
    }
    for (auto& kv : stats.nDroppedPacketsAfterDequeue) {
        std::cout << "Dropped after dequeue  [" << kv.first << "] : " << kv.second << "\n";
    }

    // ── Completion summary ─────────────────────────────────
    uint32_t completedMouse = 0, completedElephant = 0;
    uint32_t totalMouse     = 0, totalElephant     = 0;
    for (auto& kv : flowTracker) {
        if (kv.second.isMouse) {
            totalMouse++;
            if (kv.second.completed) completedMouse++;
        } else {
            totalElephant++;
            if (kv.second.completed) completedElephant++;
        }
    }
    std::cout << "Mouse flows completed    : " << completedMouse
              << " / " << totalMouse    << "\n"
              << "Elephant flows completed : " << completedElephant
              << " / " << totalElephant << "\n"
              << "Outputs: queue_length2.csv  fct_results2.csv\n";

    return 0;
}