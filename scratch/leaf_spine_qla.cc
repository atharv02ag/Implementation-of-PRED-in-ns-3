/*
 * Outputs (CSV)
 * --------
 *   fct_<workload>_load<X>.csv   — per-flow FCT in µs
 *   queue_<workload>_load<X>.csv — queue depth sampled at 50 µs intervals
 *
 * Usage examples
 * --------
 *   ./ns3 run "leaf-spine-ecn --workload=WebSearch --load=0.5"
 *   ./ns3 run "leaf-spine-ecn --workload=DataMining --load=0.8 --simTime=2.0"
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/traffic-control-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeafSpineEcn");

/* =========================================================================
 * Topology constants
 * ========================================================================= */
static constexpr uint32_t N_SPINE          = 8;
static constexpr uint32_t N_LEAF           = 8;
static constexpr uint32_t N_SERVERS        = 128;           // 8 leaves × 16 servers
static constexpr uint32_t SERVERS_PER_LEAF = N_SERVERS / N_LEAF;   // 16

//in MB
static constexpr double MAX_K = 0.5; 
static constexpr double MIN_K = (10.0*1500.0)/1000000.0;
static constexpr uint32_t QUEUE_LIMIT  = 1000;   // switch buffer (pkts)
static const double LAMBDA           = 1.6;  

/* =========================================================================
 * Flow size CDFs
 *   Each entry: {cumulative probability, flow size in bytes}
 * ========================================================================= */
struct CdfEntry { double prob; uint64_t bytes; };

/* WebSearch CDF (approximated from SIGCOMM'10, used in DCTCP paper) */
static const std::vector<CdfEntry> CDF_WEBSEARCH = {
    {0.0,   0},
    {0.15,  10000},
    {0.20,  20000},
    {0.30,  30000},
    {0.40,  50000},
    {0.53,  80000},
    {0.60,  200000},
    {0.70,  1000000},
    {0.80,  2000000},
    {0.90,  5000000},
    {0.97,  10000000},
    {1.00,  30000000},
};

/* DataMining CDF (approximated from SIGCOMM'09 VL2 paper) */
static const std::vector<CdfEntry> CDF_DATAMINING = {
    {0.0,   0},
    {0.10,  1000},
    {0.20,  2000},
    {0.30,  3000},
    {0.40,  7000},
    {0.50,  267000},
    {0.60,  2107000},
    {0.70,  66000000},
    {0.80,  267000000},
    {0.90,  1067000000},
    {1.00,  3000000000ULL},
};

/* =========================================================================
 * Sample a flow size from a CDF table
 * ========================================================================= */
static uint64_t
SampleCdf(const std::vector<CdfEntry>& cdf, double u)
{
    for (std::size_t i = 1; i < cdf.size(); ++i)
    {
        if (u <= cdf[i].prob)
        {
            double range = cdf[i].prob - cdf[i - 1].prob;
            double frac  = (range > 0.0) ? (u - cdf[i - 1].prob) / range : 0.0;
            uint64_t lo  = cdf[i - 1].bytes;
            uint64_t hi  = cdf[i].bytes;
            return lo + static_cast<uint64_t>(frac * static_cast<double>(hi - lo));
        }
    }
    return cdf.back().bytes;
}

/* =========================================================================
 * Per-flow record for FCT measurement
 * ========================================================================= */
struct FlowRecord
{
    Time     startTime;
    uint64_t expectedBytes;
    bool     completed;
};

static std::map<uint32_t, FlowRecord> g_flowTracker;
static std::ofstream g_fctLog;
static std::ofstream g_queueLog;

/* =========================================================================
 * PacketSink Rx callback
 * ========================================================================= */
static void
OnRx(Ptr<PacketSink> sink, uint32_t port,
     Ptr<const Packet> /*pkt*/, const Address& /*from*/)
{
    auto it = g_flowTracker.find(port);
    if (it == g_flowTracker.end() || it->second.completed) { return; }

    if (sink->GetTotalRx() >= it->second.expectedBytes)
    {
        it->second.completed = true;
        Time fct = Simulator::Now() - it->second.startTime;
        g_fctLog << Simulator::Now().GetMicroSeconds() << ","
                 << it->second.expectedBytes << ","
                 << fct.GetMicroSeconds() << "\n";
    }
}

/* =========================================================================
 * Periodic queue monitor
 * ========================================================================= */
static void
MonitorQueue(Ptr<QueueDisc> qd, double stopTime)
{
    if (Simulator::Now().GetSeconds() >= stopTime) { return; }
    g_queueLog << Simulator::Now().GetMicroSeconds() << ","
               << qd->GetCurrentSize().GetValue() << "\n";
    Simulator::Schedule(MicroSeconds(50), &MonitorQueue, qd, stopTime);
}

static TrafficControlHelper
MakeStepPREDHelper()
{
    TrafficControlHelper tch;
    tch.SetRootQueueDisc(
        "ns3::PRedQueueDisc",
        "MinTh",         DoubleValue(MIN_K),
        "MaxTh",         DoubleValue(MAX_K),
        "LinkBandwidth", StringValue("10Gbps"),
        "LinkDelay",     StringValue("10us"),
        "Lambda",        DoubleValue(LAMBDA),
        "MeanPktSize",   UintegerValue(1500),
        "MaxSize",       QueueSizeValue(
                             QueueSize(QueueSizeUnit::PACKETS, QUEUE_LIMIT)),
        "QW",            DoubleValue(1.0),          // instantaneous queue
        "UseEcn",        BooleanValue(true),
        "UseFCS",        BooleanValue(false),
        "UseQLA",        BooleanValue(true),
        "UseHardDrop",   BooleanValue(false));       // true step, no ramp
    return tch;
}

/* =========================================================================
 * main
 * ========================================================================= */
int
main(int argc, char* argv[])
{
    /* ── Command-line arguments ── */
    std::string workload = "WebSearch";
    double      load     = 0.8;
    double      trafficTime = 0.25;
    double      simTime  = trafficTime + 2.0; // allow extra time for flows to drain after traffic generation ends
    double      warmup   = 0.1;

    CommandLine cmd;
    cmd.AddValue("workload", "Traffic workload: WebSearch or DataMining", workload);
    cmd.AddValue("load",     "Target network load [0.1–0.9]",             load);
    cmd.AddValue("simTime",  "Total simulation time (seconds)",            simTime);
    cmd.Parse(argc, argv);

    /* ── Global TCP / DCTCP / ECN ── */
    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       StringValue("ns3::TcpDctcp"));
    Config::SetDefault("ns3::TcpSocketBase::UseEcn",
                       StringValue("On"));
    Config::SetDefault("ns3::TcpSocket::SegmentSize",  UintegerValue(1460));
    Config::SetDefault("ns3::TcpSocket::SndBufSize",   UintegerValue(16777216));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize",   UintegerValue(16777216));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd",  UintegerValue(10));
    Config::SetDefault("ns3::TcpSocket::DelAckCount",  UintegerValue(1));

    /* ── CDF selection ── */
    const std::vector<CdfEntry>* cdf = &CDF_WEBSEARCH;
    if (workload == "DataMining")       { cdf = &CDF_DATAMINING; }
    else if (workload != "WebSearch")
    { NS_LOG_WARN("Unknown workload '" << workload << "'; using WebSearch."); }

    /* =========================================================
     * Node creation
     * ========================================================= */
    NodeContainer spines;  spines.Create(N_SPINE);
    NodeContainer leaves;  leaves.Create(N_LEAF);
    NodeContainer servers; servers.Create(N_SERVERS);

    InternetStackHelper stack;
    stack.Install(spines);
    stack.Install(leaves);
    stack.Install(servers);

    TrafficControlHelper tchNone;

    /* ── Link helper ── */
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute ("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay",    StringValue("10us"));
    // Keep the device queue tiny; all buffering/AQM is in the qdisc layer.
    p2p.SetQueue("ns3::DropTailQueue", "MaxSize", StringValue("8p"));

    Ipv4AddressHelper addrHelper;

    /* =========================================================
     * Wiring: servers → leaves
     * =========================================================
     */
    std::vector<NetDeviceContainer> serverLeafLinks(N_SERVERS);

    for (uint32_t l = 0; l < N_LEAF; ++l)
    {
        for (uint32_t s = 0; s < SERVERS_PER_LEAF; ++s)
        {
            uint32_t sid = l * SERVERS_PER_LEAF + s;
            NetDeviceContainer lnk = p2p.Install(servers.Get(sid), leaves.Get(l));

            serverLeafLinks[sid] = lnk;

            std::ostringstream base;
            base << "10." << (l + 1) << "." << (s * 4) << ".0";
            addrHelper.SetBase(base.str().c_str(), "255.255.255.252");
            addrHelper.Assign(lnk);

            tchNone.Uninstall(lnk);
        }
    }

    /* =========================================================
     * Wiring: leaves → spines
     * ========================================================= */
    std::vector<std::vector<NetDeviceContainer>> leafSpineLinks(
        N_LEAF, std::vector<NetDeviceContainer>(N_SPINE));

    for (uint32_t l = 0; l < N_LEAF; ++l)
    {
        for (uint32_t sp = 0; sp < N_SPINE; ++sp)
        {
            NetDeviceContainer lnk = p2p.Install(leaves.Get(l), spines.Get(sp));

            leafSpineLinks[l][sp] = lnk;

            uint32_t subnet_id = l * N_SPINE + sp;
            std::ostringstream base;
            base << "172.16." << subnet_id << ".0";
            addrHelper.SetBase(base.str().c_str(), "255.255.255.252");
            addrHelper.Assign(lnk);

            tchNone.Uninstall(lnk);
        }
    }

    /* ── ECMP routing ── */
    Config::SetDefault("ns3::Ipv4GlobalRouting::RandomEcmpRouting",
                   BooleanValue(true));
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    /* =========================================================
     * AQM: step-ECN on every switch egress port
     *
     * After stripping defaults above, every device has no qdisc.
     * We now install our step-ECN RedQueueDisc on the switch-facing
     * (egress) side of each link.  Server-facing leaf devices and
     * server-side devices are left without a qdisc (servers are
     * end-hosts; congestion is managed at the switch egress).
     * ========================================================= */
    TrafficControlHelper tchEcn = MakeStepPREDHelper();

    /* leaf egress toward spine (device index 0 = leaf side of leaf-spine link) */
    std::vector<std::vector<QueueDiscContainer>> leafSpineQd(
        N_LEAF, std::vector<QueueDiscContainer>(N_SPINE));
    for (uint32_t l = 0; l < N_LEAF; ++l)
    {
        for (uint32_t sp = 0; sp < N_SPINE; ++sp)
        {
            NetDeviceContainer leafDev;
            leafDev.Add(leafSpineLinks[l][sp].Get(0));   // leaf side
            leafSpineQd[l][sp] = tchEcn.Install(leafDev);
        }
    }

    /* spine egress toward leaf (device index 1 = spine side of leaf-spine link) */
    std::vector<std::vector<QueueDiscContainer>> spineLeafQd(
        N_SPINE, std::vector<QueueDiscContainer>(N_LEAF));
    for (uint32_t sp = 0; sp < N_SPINE; ++sp)
    {
        for (uint32_t l = 0; l < N_LEAF; ++l)
        {
            NetDeviceContainer spineDev;
            spineDev.Add(leafSpineLinks[l][sp].Get(1));  // spine side
            spineLeafQd[sp][l] = tchEcn.Install(spineDev);
        }
    }

    /* leaf egress toward servers (device index 1 = leaf side of server-leaf link) */
    std::vector<QueueDiscContainer> leafServerQd(N_SERVERS);
    std::vector<QueueDiscContainer> serverUplinkQd(N_SERVERS);
    for (uint32_t l = 0; l < N_LEAF; ++l)
    {
        for (uint32_t s = 0; s < SERVERS_PER_LEAF; ++s)
        {
            uint32_t sid = l * SERVERS_PER_LEAF + s;
            NetDeviceContainer leafDev;
            leafDev.Add(serverLeafLinks[sid].Get(1));    // leaf side
            leafServerQd[sid] = tchEcn.Install(leafDev);

            NetDeviceContainer serverDev;
            serverDev.Add(serverLeafLinks[sid].Get(0));  // server side → leaf (uplink)
            serverUplinkQd[sid] = tchEcn.Install(serverDev);
        }
    }

    /* =========================================================
     * Traffic generation
     * ========================================================= */
    double meanBytes = 0.0;
    for (std::size_t i = 1; i < cdf->size(); ++i)
    {
        double dp    = (*cdf)[i].prob - (*cdf)[i - 1].prob;
        double avgSz = 0.5 * (static_cast<double>((*cdf)[i - 1].bytes)
                              + static_cast<double>((*cdf)[i].bytes));
        meanBytes += dp * avgSz;
    }

    double uplinkBps        = static_cast<double>(N_SPINE) * 10.0e9;  // 80 Gbps
    double lambdaPerServer  = (load * uplinkBps) /
                              (static_cast<double>(SERVERS_PER_LEAF) * meanBytes * 8.0);

    NS_LOG_INFO("Workload=" << workload
                << " load=" << load
                << " meanBytes=" << meanBytes
                << " lambdaPerServer=" << lambdaPerServer << " flows/s");

    Ptr<ExponentialRandomVariable> interArrival = CreateObject<ExponentialRandomVariable>();
    interArrival->SetAttribute("Mean", DoubleValue(1.0 / lambdaPerServer));

    Ptr<UniformRandomVariable> uniCdf = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> uniDst = CreateObject<UniformRandomVariable>();

    std::vector<Ipv4Address> serverAddrs(N_SERVERS);
    for (uint32_t i = 0; i < N_SERVERS; ++i)
    {
        Ptr<Ipv4> ipv4 = servers.Get(i)->GetObject<Ipv4>();
        serverAddrs[i] = ipv4->GetAddress(1, 0).GetLocal();
    }

    /* ── Output files ── */
    std::ostringstream fctName, qName;
    fctName << "fct_qla_load_" << load << ".csv";
    qName   << "queue_qla_load_" << load << ".csv";

    g_fctLog.open(fctName.str());
    g_fctLog << "CompletionTime_us,FlowSize_B,FCT_us\n";

    g_queueLog.open(qName.str());
    g_queueLog << "Time_us,QueuePkts\n";

    /* ── Schedule flows ── */
    uint32_t portCounter = 50000;

    for (uint32_t sid = 0; sid < N_SERVERS; ++sid)
    {
        double t = warmup;
        while (t < trafficTime - 0.1)
        {
            double   u        = uniCdf->GetValue(0.0, 1.0);
            uint64_t flowSize = SampleCdf(*cdf, u);
            if (flowSize < 1) { flowSize = 1460; }

            uint32_t dstSid;

            do { dstSid = uniDst->GetInteger(0, N_SERVERS - 1); } while (dstSid == sid);

            uint32_t port = portCounter++;

            Address sinkAddr(InetSocketAddress(serverAddrs[dstSid], port));
            PacketSinkHelper sinkH("ns3::TcpSocketFactory",
                                   InetSocketAddress(Ipv4Address::GetAny(), port));
            ApplicationContainer sinkApp = sinkH.Install(servers.Get(dstSid));
            sinkApp.Start(Seconds(t - 0.0001));
            sinkApp.Stop(Seconds(simTime));

            Ptr<PacketSink> sinkPtr = DynamicCast<PacketSink>(sinkApp.Get(0));
            g_flowTracker[port] = {Seconds(t), flowSize, false};
            sinkPtr->TraceConnectWithoutContext(
                "Rx", MakeBoundCallback(&OnRx, sinkPtr, port));

            BulkSendHelper srcH("ns3::TcpSocketFactory", sinkAddr);
            srcH.SetAttribute("MaxBytes",
                              UintegerValue(static_cast<uint32_t>(
                                  std::min(flowSize, static_cast<uint64_t>(UINT32_MAX)))));
            ApplicationContainer srcApp = srcH.Install(servers.Get(sid));
            srcApp.Start(Seconds(t));
            srcApp.Stop(Seconds(simTime));

            t += interArrival->GetValue();
        }
    }

    /* ── Queue monitor ── */
    Simulator::Schedule(Seconds(warmup),
                        &MonitorQueue,
                        leafSpineQd[0][0].Get(0),
                        simTime);

    /* ── Summary ── */
    std::cout
        << "════════════════════════════════════════════════════════\n"
        << "  Leaf-Spine PRED\n"
        << "════════════════════════════════════════════════════════\n"
        << "  Servers          : " << N_SERVERS << "  ("
                                   << N_LEAF << " leaves × "
                                   << SERVERS_PER_LEAF << " servers)\n"
        << "  Spine switches   : " << N_SPINE << "\n"
        << "  Link BW / delay  : 10 Gbps / 10 µs\n"
        << "  PRED [minTh,maxTh]: " << MIN_K << " MB, " << MAX_K << " MB\n"
        << "  Queue limit      : " << QUEUE_LIMIT  << " pkts  [PACKETS mode]\n"
        << "  Target load      : " << load * 100 << " %\n"
        << "  E[flow size]     : " << meanBytes << " B\n"
        << "  λ per server     : " << lambdaPerServer << " flows/s\n"
        << "  Total flows sched: " << g_flowTracker.size() << "\n"
        << "  Sim time         : " << simTime << " s\n"
        << "  FCT log          : " << fctName.str() << "\n"
        << "  Queue log        : " << qName.str() << "\n"
        << "════════════════════════════════════════════════════════\n";

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    g_fctLog.close();
    g_queueLog.close();

    uint32_t completed = 0;
    uint32_t total     = static_cast<uint32_t>(g_flowTracker.size());
    for (auto& kv : g_flowTracker) { if (kv.second.completed) { ++completed; } }

    std::cout
        << "════════════════════════════════════════════════════════\n"
        << "  Flows completed : " << completed << " / " << total << "\n"
        << "════════════════════════════════════════════════════════\n";

    return 0;
}