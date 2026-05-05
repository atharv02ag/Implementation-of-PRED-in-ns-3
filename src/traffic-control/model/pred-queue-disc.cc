/*
 * Copyright © 2011 Marcos Talau
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Marcos Talau (talau@users.sourceforge.net)
 *
 * Thanks to: Duy Nguyen<duy@soe.ucsc.edu> by RED efforts in NS3
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *
 * Copyright (c) 1990-1997 Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * PORT NOTE: This code was ported from ns-2 (queue/red.cc).  Almost all
 * comments have also been ported from NS-2.
 */

#include "pred-queue-disc.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/drop-tail-queue.h"
#include "ns3/enum.h"
#include "ns3/ipv4-header.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/tcp-header.h"
#include "ns3/udp-header.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("PRedQueueDisc");
NS_OBJECT_ENSURE_REGISTERED(PRedQueueDisc);

/** Convert packet count to MB using the fixed 1500-byte MTU convention. */
static inline double
PktsToMB(double pkts, double avgPktSizeBytes = 1500.0)
{
    return (pkts * avgPktSizeBytes) / 1'000'000.0;
}

TypeId
PRedQueueDisc::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::PRedQueueDisc")
            .SetParent<QueueDisc>()
            .SetGroupName("TrafficControl")
            .AddConstructor<PRedQueueDisc>()
            .AddAttribute("MeanPktSize",
                          "Average of packet size",
                          UintegerValue(1500),
                          MakeUintegerAccessor(&PRedQueueDisc::m_meanPktSize),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("IdlePktSize",
                          "Average packet size used during idle times. Used when m_cautious == 3",
                          UintegerValue(0),
                          MakeUintegerAccessor(&PRedQueueDisc::m_idlePktSize),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("Wait",
                          "True for waiting between dropped packets",
                          BooleanValue(true),
                          MakeBooleanAccessor(&PRedQueueDisc::m_isWait),
                          MakeBooleanChecker())
            .AddAttribute("UseFCS",
                          "True to enable the FCS module (dynamically adjusts lambda)",
                          BooleanValue(true),
                          MakeBooleanAccessor(&PRedQueueDisc::m_isUseFCS),
                          MakeBooleanChecker())
            .AddAttribute("MinTh",
                          "Minimum average length threshold in MB",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&PRedQueueDisc::m_minTh),
                          MakeDoubleChecker<double>())
            .AddAttribute("MaxTh",
                          "Maximum average length threshold in MB",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&PRedQueueDisc::m_maxTh),
                          MakeDoubleChecker<double>())
            .AddAttribute("Lambda",
                          "Base lambda parameter for PRED (FCS will scale this at runtime)",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&PRedQueueDisc::m_lambda),
                          MakeDoubleChecker<double>())
            .AddAttribute("MaxSize",
                          "The maximum number of packets accepted by this queue disc",
                          QueueSizeValue(QueueSize("25p")),
                          MakeQueueSizeAccessor(&QueueDisc::SetMaxSize, &QueueDisc::GetMaxSize),
                          MakeQueueSizeChecker())
            .AddAttribute("QW",
                          "Queue weight for the EWMA",
                          DoubleValue(0.002),
                          MakeDoubleAccessor(&PRedQueueDisc::m_qW),
                          MakeDoubleChecker<double>())
            .AddAttribute("TFCS",
                          "FCS period length (seconds). Default: 1.25*RTT computed in InitializeParams",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PRedQueueDisc::m_TFCS),
                          MakeDoubleChecker<double>())
            .AddAttribute("Interval",
                          "Time interval to update m_curMaxP",
                          TimeValue(Seconds(0.5)),
                          MakeTimeAccessor(&PRedQueueDisc::m_interval),
                          MakeTimeChecker())
            .AddAttribute("Ns1Compat",
                          "NS-1 compatibility",
                          BooleanValue(false),
                          MakeBooleanAccessor(&PRedQueueDisc::m_isNs1Compat),
                          MakeBooleanChecker())
            .AddAttribute("LinkBandwidth",
                          "The RED link bandwidth",
                          DataRateValue(DataRate("1.5Mbps")),
                          MakeDataRateAccessor(&PRedQueueDisc::m_linkBandwidth),
                          MakeDataRateChecker())
            .AddAttribute("LinkDelay",
                          "The RED link delay",
                          TimeValue(MilliSeconds(20)),
                          MakeTimeAccessor(&PRedQueueDisc::m_linkDelay),
                          MakeTimeChecker())
            .AddAttribute("UseEcn",
                          "True to use ECN (packets are marked instead of being dropped)",
                          BooleanValue(false),
                          MakeBooleanAccessor(&PRedQueueDisc::m_useEcn),
                          MakeBooleanChecker())
            .AddAttribute("UseHardDrop",
                          "True to always drop packets above max threshold",
                          BooleanValue(true),
                          MakeBooleanAccessor(&PRedQueueDisc::m_useHardDrop),
                          MakeBooleanChecker())
            .AddAttribute("UseQLA",
                          "True to enable the QLA module",
                          BooleanValue(true),
                          MakeBooleanAccessor(&PRedQueueDisc::m_useQla),
                          MakeBooleanChecker())
            .AddAttribute("QlaBeta",
                          "QLA utility weight beta (throughput vs queue trade-off)",
                          DoubleValue(0.4),
                          MakeDoubleAccessor(&PRedQueueDisc::m_qlaBeta),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("QlaQleft",
                          "QLA left saturation point for Phi (MB). "
                          "Default = 15 pkts * 1500 B / 1e6",
                          DoubleValue(15.0 * 1500.0 / 1'000'000.0),
                          MakeDoubleAccessor(&PRedQueueDisc::m_Qleft),
                          MakeDoubleChecker<double>())
            .AddAttribute("QlaLambdaMin",
                          "QLA minimum lambda floor (triggers minK adjustment below this)",
                          DoubleValue(0.05),
                          MakeDoubleAccessor(&PRedQueueDisc::m_qlaLambdaMin),
                          MakeDoubleChecker<double>())
            .AddAttribute("QlaDeltaLambda",
                          "QLA per-step lambda adjustment Δλ",
                          DoubleValue(0.025),
                          MakeDoubleAccessor(&PRedQueueDisc::m_qlaDeltaLambda),
                          MakeDoubleChecker<double>())
            .AddAttribute("QlaDeltaMinK",
                          "QLA per-step minTh adjustment Δ_minK (MB). "
                          "Default = 5 pkts * 1500 B / 1e6",
                          DoubleValue(5.0 * 1500.0 / 1'000'000.0),
                          MakeDoubleAccessor(&PRedQueueDisc::m_qlaDeltaMinK),
                          MakeDoubleChecker<double>())
            .AddAttribute("TQLA",
                          "QLA trial period length (seconds). 0 = same as T_FCS.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&PRedQueueDisc::m_TQLA),
                          MakeDoubleChecker<double>());

    return tid;
}

PRedQueueDisc::PRedQueueDisc()
    : QueueDisc(QueueDiscSizePolicy::SINGLE_INTERNAL_QUEUE),
      // FCS params
      m_lambdaQla(1.0),
      m_fcsNLast(0),
      m_fcsN(0),
      m_fcsEstimatedN(1),
      // QLA params
      m_qlaTrial(0),
      m_qlaU{0.0, 0.0, 0.0, 0.0},
      m_qlaGoodputBytes(0),
      m_qlaQavgAccum(0.0),
      m_qlaSampleCount(0)
{
    NS_LOG_FUNCTION(this);
    m_uv = CreateObject<UniformRandomVariable>();
}

PRedQueueDisc::~PRedQueueDisc()
{
    NS_LOG_FUNCTION(this);
}

void
PRedQueueDisc::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_fcsTimer.Cancel();
    m_qlaTimer.Cancel();
    m_fcsBitmap.clear();
    m_uv = nullptr;
    QueueDisc::DoDispose();
}


void
PRedQueueDisc::SetPREDParams(double minTh, double maxTh, double lambda)
{
    NS_LOG_FUNCTION(this << minTh << maxTh << lambda);
    m_minTh      = minTh;
    m_maxTh      = maxTh;
    m_lambda     = lambda;
    m_lambdaQla  = lambda;
}

int64_t
PRedQueueDisc::AssignStreams(int64_t stream)
{
    NS_LOG_FUNCTION(this << stream);
    m_uv->SetStream(stream);
    return 1;
}

// Computes 5 tuple <src/dst IP, src/dst port, protocol> hash for the given packet, using FNV-1a.
uint32_t
PRedQueueDisc::ComputeFlowId(Ptr<QueueDiscItem> item) const
{
    NS_LOG_FUNCTION(this << item);

    Ptr<const Packet> pkt    = item->GetPacket();
    Ipv4Header        ipHdr;
    Ptr<Packet>       pktCopy = pkt->Copy();

    if (pktCopy->PeekHeader(ipHdr) == 0)
    {
        return static_cast<uint32_t>(pkt->GetUid());
    }

    uint32_t srcIp   = ipHdr.GetSource().Get();
    uint32_t dstIp   = ipHdr.GetDestination().Get();
    uint8_t  proto   = ipHdr.GetProtocol();
    uint16_t srcPort = 0;
    uint16_t dstPort = 0;

    pktCopy->RemoveHeader(ipHdr);

    if (proto == 6)
    {
        TcpHeader tcpHdr;
        if (pktCopy->PeekHeader(tcpHdr) > 0)
        {
            srcPort = tcpHdr.GetSourcePort();
            dstPort = tcpHdr.GetDestinationPort();
        }
    }
    else if (proto == 17)
    {
        UdpHeader udpHdr;
        if (pktCopy->PeekHeader(udpHdr) > 0)
        {
            srcPort = udpHdr.GetSourcePort();
            dstPort = udpHdr.GetDestinationPort();
        }
    }

    constexpr uint32_t FNV_OFFSET = 2166136261u;
    constexpr uint32_t FNV_PRIME  = 16777619u;

    auto fnv = [](uint32_t hash, const uint8_t* data, std::size_t len) -> uint32_t {
        for (std::size_t i = 0; i < len; ++i)
        {
            hash ^= data[i];
            hash *= FNV_PRIME;
        }
        return hash;
    };

    uint32_t h = FNV_OFFSET;
    h = fnv(h, reinterpret_cast<const uint8_t*>(&srcIp),   sizeof(srcIp));
    h = fnv(h, reinterpret_cast<const uint8_t*>(&dstIp),   sizeof(dstIp));
    h = fnv(h, reinterpret_cast<const uint8_t*>(&srcPort), sizeof(srcPort));
    h = fnv(h, reinterpret_cast<const uint8_t*>(&dstPort), sizeof(dstPort));
    h = fnv(h, &proto, sizeof(proto));

    return h;
}

// Insert a new flow into the FCS bitmap and update the flow count if it's a new flow.
void
PRedQueueDisc::FcsRecordArrival(Ptr<QueueDiscItem> item)
{
    uint32_t flowId = ComputeFlowId(item);
    auto [_, inserted] = m_fcsBitmap.insert(flowId);
    if (inserted)
    {
        ++m_fcsN;
        NS_LOG_DEBUG("FCS: new flow seen, period count n=" << m_fcsN
                     << " flowId=" << flowId);
    }
}

// FCS timer callback: update the estimated flow count, flush the bitmap and update lambda.
void
PRedQueueDisc::FcsTimerExpired()
{
    NS_LOG_FUNCTION(this);

    m_fcsNLast = m_fcsEstimatedN;
    m_fcsEstimatedN = (m_fcsN > 0) ? m_fcsN : 1u;

    NS_LOG_DEBUG("FCS period expired: n_last=" << m_fcsNLast
                 << " N=" << m_fcsEstimatedN);

    m_fcsN = 0;
    m_fcsBitmap.clear();

    if (m_isUseFCS)
    {
        FcsUpdateLambda();
    }

    m_fcsTimer = Simulator::Schedule(Seconds(m_TFCS),
                                     &PRedQueueDisc::FcsTimerExpired,
                                     this);
}

void
PRedQueueDisc::FcsUpdateLambda()
{
    uint32_t N = (m_fcsEstimatedN > 0) ? m_fcsEstimatedN : 1u;
    N = std::max(m_fcsNLast, N);

    // Here, f(N) = N according to the PRED paper.
    m_lambda = m_lambdaQla * static_cast<double>(N);
    

    NS_LOG_DEBUG("FCS: N=" << N
                 << " lambda_QLA=" << m_lambdaQla
                 << " lambda_effective=" << m_lambda);
}


// Computes the QLA utility U for the given average queue size and goodput, using the formula in PRED paper
double
PRedQueueDisc::QlaComputeUtility(double avgQueueMB, double goodputBps) const
{
    /*
     * Throughput term: R/C, normalised to [0,1].
     * C is the link bandwidth in bytes/s.
     */
    double C          = m_linkBandwidth.GetBitRate() / 8.0;   // bytes/s
    double throughput = (C > 0.0) ? std::min(goodputBps / C, 1.0) : 0.0;

    /*
     * Queue term: Φ(q̄).
     *
     *   Φ(q̄) = 1.0                           if q̄ ≤ q_left
     *           1 − (q̄ − q_left)/(maxTh − q_left)  if q_left < q̄ < maxTh
     *           0.0                           if q̄ ≥ q_right (here we simply use maxTh)
     */
    double phi;
    if (avgQueueMB <= m_Qleft)
    {
        phi = 1.0;
    }
    else if (avgQueueMB >= m_maxTh)
    {
        phi = 0.0;
    }
    else
    {
        double range = m_maxTh - m_Qleft;
        phi = (range > 0.0) ? 1.0 - (avgQueueMB - m_Qleft) / range : 0.0;
    }

    double U = m_qlaBeta * throughput + (1.0 - m_qlaBeta) * phi;

    NS_LOG_DEBUG("QLA utility: avgQ_MB=" << avgQueueMB
                 << " goodput_Bps=" << goodputBps
                 << " throughput_norm=" << throughput
                 << " phi=" << phi
                 << " U=" << U);

    return U;
}

// Apply the AIAD QLA decision rule after 4 trials and adjust lambda_QLA accordingly, then call the minK adjuster if needed.
void
PRedQueueDisc::QlaDecide()
{
    /*
     *
     * Trial layout (each trial alternates high/low probe):
     *   trial 0: high probe (λ_QLA + Δλ)  → U[0]
     *   trial 1: low  probe (λ_QLA − Δλ)  → U[1]
     *   trial 2: high probe (λ_QLA - Δλ)  → U[2]
     *   trial 3: low  probe (λ_QLA + Δλ)  → U[3]
     *
     * Decision:
     *   U[0]>U[1] AND U[3]>U[2]  → high is consistently better  → λ_QLA += Δλ
     *   U[1]>U[0] AND U[2]>U[3]  → low  is consistently better  → λ_QLA -= Δλ
     *   otherwise                → inconclusive                 → no change
     */
    double U0 = m_qlaU[0];
    double U1 = m_qlaU[1];
    double U2 = m_qlaU[2];
    double U3 = m_qlaU[3];

    NS_LOG_DEBUG("QLA decide: U[0]=" << U0 << " U[1]=" << U1
                 << " U[2]=" << U2 << " U[3]=" << U3
                 << " lambda_QLA=" << m_lambdaQla);

    if (U0 > U1 && U3 > U2)
    {
        m_lambdaQla += m_qlaDeltaLambda;
        NS_LOG_DEBUG("QLA: higher lambda consistently better → lambda_QLA=" << m_lambdaQla);
    }
    else if (U1 > U0 && U2 > U3)
    {
        m_lambdaQla -= m_qlaDeltaLambda;
        NS_LOG_DEBUG("QLA: lower lambda consistently better → lambda_QLA=" << m_lambdaQla);
    }
    else
    {
        NS_LOG_DEBUG("QLA: inconclusive — lambda_QLA unchanged at " << m_lambdaQla);
    }

    QlaAdjustMinK();

}

// minK adjuster: slowly increase minK if lambda_QLA hits the minimum floor
void
PRedQueueDisc::QlaAdjustMinK()
{
    if (m_lambdaQla < m_qlaLambdaMin)
    {
        NS_LOG_DEBUG("QLA minK mode: lambda_QLA=" << m_lambdaQla
                     << " < lambda_min=" << m_qlaLambdaMin
                     << " → adjusting minTh by -" << m_qlaDeltaMinK);

        m_lambdaQla  = m_qlaLambdaMin;
        m_minTh += m_qlaDeltaMinK;
        if (m_minTh < 0.0)
        {
            m_minTh = 0.0;
        }

        NS_LOG_DEBUG("QLA: new minTh=" << m_minTh);
    }
}

void
PRedQueueDisc::QlaTimerExpired()
{
    NS_LOG_FUNCTION(this);

    /*
     * --- Step 1: compute utility for the trial that just ended --- 
     *
     * Average queue in MB: accumulated sum of m_qAvg (in packets) divided by
     * sample count, then converted to MB.
     */
    double avgQpkts  = (m_qlaSampleCount > 0)
                       ? m_qlaQavgAccum / static_cast<double>(m_qlaSampleCount)
                       : 0.0;
    double avgQmb    = PktsToMB(avgQpkts);
    double goodputBps = static_cast<double>(m_qlaGoodputBytes) / m_TQLA;

    double U = QlaComputeUtility(avgQmb, goodputBps);
    m_qlaU[m_qlaTrial] = U;

    NS_LOG_DEBUG("QLA trial " << m_qlaTrial << " ended: "
                 << "probe=" << m_qlaCurrentProbe
                 << " avgQ_MB=" << avgQmb
                 << " goodput_Bps=" << goodputBps
                 << " U=" << U);

    /* --- Step 2: reset per-trial accumulators --- */
    m_qlaGoodputBytes = 0;
    m_qlaQavgAccum    = 0.0;
    m_qlaSampleCount  = 0;

    /* --- Step 3: advance trial index, decide after 4 trials --- */
    m_qlaTrial++;

    if (m_qlaTrial == 4)
    {
        QlaDecide();
        m_qlaTrial = 0;
        m_qlaCurrentProbe = m_lambdaQla;
        m_qlaTimer = Simulator::Schedule(Seconds(m_TQLA),
                                        &PRedQueueDisc::QlaTimerExpired,
                                        this);
    }
    else{
        /* --- Step 4: set the probe for the next trial ---
        *
        * trials 0 and 3 probe high: λ_QLA + Δλ
        * trials 1 and 2 probe low:  λ_QLA − Δλ
        */
        if (m_qlaTrial == 0 || m_qlaTrial == 3)
        {
            m_lambdaQla = m_qlaCurrentProbe + m_qlaDeltaLambda;
        }
        else
        {
            m_lambdaQla = std::max(m_qlaCurrentProbe - m_qlaDeltaLambda, m_qlaLambdaMin);
        }

        NS_LOG_DEBUG("QLA: next trial=" << m_qlaTrial
                    << " probe=" << m_qlaCurrentProbe);

        /* --- Step 6: re-arm the timer --- */
        m_qlaTimer = Simulator::Schedule(Seconds(m_TQLA),
                                        &PRedQueueDisc::QlaTimerExpired,
                                        this);
    }

}

bool
PRedQueueDisc::DoEnqueue(Ptr<QueueDiscItem> item)
{
    NS_LOG_FUNCTION(this << item);

    NS_LOG_DEBUG("PKT: size=" << item->GetSize() << " bytes  qAvg=" << m_qAvg);

    uint32_t nQueued = GetInternalQueue(0)->GetCurrentSize().GetValue();

    //FCS: record this packet's flow 
    if (m_isUseFCS)
    {
        FcsRecordArrival(item);
    }

    //QLA: accumulate queue sample for utility computation
    if (m_useQla)
    {
        m_qlaQavgAccum += m_qAvg;   // m_qAvg in packets (converted to MB at period end)
        m_qlaSampleCount++;
    }

    uint32_t m = 0;

    if (m_idle == 1)
    {
        NS_LOG_DEBUG("RED Queue Disc is idle.");
        Time now = Simulator::Now();

        if (m_cautious == 3)
        {
            double ptc = m_ptc * m_meanPktSize / m_idlePktSize;
            m = uint32_t(ptc * (now - m_idleTime).GetSeconds());
        }
        else
        {
            m = uint32_t(m_ptc * (now - m_idleTime).GetSeconds());
        }

        m_idle = 0;
    }

    m_qAvg = Estimator(nQueued, m + 1, m_qAvg, m_qW);

    NS_LOG_DEBUG("\t bytesInQueue  " << GetInternalQueue(0)->GetNBytes()
                 << "\tQavg " << m_qAvg);
    NS_LOG_DEBUG("\t packetsInQueue  " << GetInternalQueue(0)->GetNPackets()
                 << "\tQavg " << m_qAvg);

    m_count++;
    m_countBytes += item->GetSize();

    //Convert EWMA queue (packets) → MB for threshold comparison
    double size_qAvg = PktsToMB(m_qAvg);

    uint32_t dropType = DTYPE_NONE;

    if (size_qAvg >= m_minTh && nQueued > 1)
    {
        if (size_qAvg >= m_maxTh)
        {
            NS_LOG_DEBUG("adding DROP FORCED MARK");
            dropType = DTYPE_FORCED;
        }
        else if (m_old == 0)
        {
            m_count      = 1;
            m_countBytes = item->GetSize();
            m_old        = 1;
        }
        else if (DropEarly(item, nQueued))
        {
            NS_LOG_LOGIC("DropEarly returns 1");
            dropType = DTYPE_UNFORCED;
        }
    }
    else
    {
        m_vProb = 0.0;
        m_old   = 0;
    }

    if (dropType == DTYPE_UNFORCED)
    {
        if (!m_useEcn || !Mark(item, UNFORCED_MARK))
        {
            NS_LOG_DEBUG("\t Dropping due to Prob Mark " << m_qAvg);
            DropBeforeEnqueue(item, UNFORCED_DROP);
            return false;
        }
        NS_LOG_DEBUG("\t Marking due to Prob Mark " << m_qAvg);
    }
    else if (dropType == DTYPE_FORCED)
    {
        if (m_useHardDrop || !m_useEcn || !Mark(item, FORCED_MARK))
        {
            NS_LOG_DEBUG("\t Dropping due to Hard Mark " << m_qAvg);
            DropBeforeEnqueue(item, FORCED_DROP);
            if (m_isNs1Compat)
            {
                m_count      = 0;
                m_countBytes = 0;
            }
            return false;
        }
        NS_LOG_DEBUG("\t Marking due to Hard Mark " << m_qAvg);
    }

    bool retval = GetInternalQueue(0)->Enqueue(item);

    NS_LOG_LOGIC("Number packets " << GetInternalQueue(0)->GetNPackets());
    NS_LOG_LOGIC("Number bytes "   << GetInternalQueue(0)->GetNBytes());

    return retval;
}

void
PRedQueueDisc::InitializeParams()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Initializing PRED params.");

    m_cautious = 0;
    m_ptc      = m_linkBandwidth.GetBitRate() / (8.0 * m_meanPktSize);

    if (m_minTh == 0)
    {
        m_minTh = 0.1;
    }

    m_qAvg       = 0.0;
    m_count      = 0;
    m_countBytes = 0;
    m_old        = 0;
    m_idle       = 1;
    m_idleTime   = NanoSeconds(0);

    double rtt = 3.0 * (m_linkDelay.GetSeconds() + 1.0 / m_ptc);
    if(m_TFCS <= 0) m_TFCS     = 1.25 * rtt;
    if(m_TQLA <= 0) m_TQLA     = 5 * rtt;

    if (m_qW == 0.0)
    {
        m_qW = 1.0 - std::exp(-1.0 / m_ptc);
    }
    else if (m_qW == -1.0)
    {
        if (rtt < 0.1) { rtt = 0.1; }
        m_qW = 1.0 - std::exp(-1.0 / (10 * rtt * m_ptc));
    }
    else if (m_qW == -2.0)
    {
        m_qW = 1.0 - std::exp(-10.0 / m_ptc);
    }

    m_lambdaQla     = m_lambda;   // QLA starts at the user-configured λ
    m_qlaCurrentProbe = m_lambdaQla;
    m_fcsN          = 0;
    m_fcsNLast      = 0;
    m_fcsEstimatedN = 1;
    m_fcsBitmap.clear();

    if (m_isUseFCS)
    {
        m_fcsTimer = Simulator::Schedule(Seconds(m_TFCS),
                                         &PRedQueueDisc::FcsTimerExpired,
                                         this);
        NS_LOG_INFO("FCS enabled: T_FCS=" << m_TFCS << "s  lambda_base=" << m_lambdaQla);
    }
    else
    {
        NS_LOG_INFO("FCS disabled: using fixed lambda=" << m_lambda);
    }

    // QLA init
    if (m_useQla)
    {
        m_qlaTrial        = 0;
        m_qlaU            = {0.0, 0.0, 0.0, 0.0};
        m_qlaGoodputBytes = 0;
        m_qlaQavgAccum    = 0.0;
        m_qlaSampleCount  = 0;

        // First trial is always the high probe.
        m_qlaCurrentProbe = m_lambdaQla + m_qlaDeltaLambda;

        m_qlaTimer = Simulator::Schedule(Seconds(m_TQLA),
                                         &PRedQueueDisc::QlaTimerExpired,
                                         this);

        NS_LOG_INFO("QLA enabled: T_QLA=" << m_TQLA
                    << "s  beta=" << m_qlaBeta
                    << "  lambda_min=" << m_qlaLambdaMin
                    << "  delta_lambda=" << m_qlaDeltaLambda
                    << "  delta_minK=" << m_qlaDeltaMinK
                    << "  q_left=" << m_Qleft);
    }
    else
    {
        NS_LOG_INFO("QLA disabled.");
    }

    NS_LOG_DEBUG("\tm_delay "   << m_linkDelay.GetSeconds()
                 << "; m_isWait "    << m_isWait
                 << "; m_qW "        << m_qW
                 << "; m_ptc "       << m_ptc
                 << "; m_minTh "     << m_minTh
                 << "; m_lambda "    << m_lambda
                 << "; m_TFCS "      << m_TFCS);
}

double
PRedQueueDisc::Estimator(uint32_t nQueued, uint32_t m, double qAvg, double qW)
{
    NS_LOG_FUNCTION(this << nQueued << m << qAvg << qW);
    double newAve  = qAvg * std::pow(1.0 - qW, m);
    newAve        += qW * nQueued;
    return newAve;
}

bool
PRedQueueDisc::DropEarly(Ptr<QueueDiscItem> item, uint32_t qSize)
{
    NS_LOG_FUNCTION(this << item << qSize);

    double prob1 = CalculatePNew();
    m_vProb      = prob1;  // PRED does not use the RED-style modification step

    NS_LOG_DEBUG("PROB: qAvg=" << m_qAvg
                 << " raw_p=" << prob1
                 << " modified_p=" << m_vProb
                 << " lambda=" << m_lambda
                 << " N=" << m_fcsEstimatedN);

    if (m_cautious == 1)
    {
        double pkts     = m_ptc * 0.05;
        double fraction = std::pow((1 - m_qW), pkts);
        if ((double)qSize < fraction * m_qAvg)
        {
            return false;
        }
    }

    double u = m_uv->GetValue();

    if (m_cautious == 2)
    {
        double pkts     = m_ptc * 0.05;
        double fraction = std::pow((1 - m_qW), pkts);
        double ratio    = qSize / (fraction * m_qAvg);
        if (ratio < 1.0)
        {
            u *= 1.0 / ratio;
        }
    }

    if (u <= m_vProb)
    {
        NS_LOG_LOGIC("u <= m_vProb; u " << u << "; m_vProb " << m_vProb);
        m_count      = 0;
        m_countBytes = 0;
        return true;
    }

    return false;
}

double
PRedQueueDisc::CalculatePNew()
{
    NS_LOG_FUNCTION(this);

    double p         = 0.0;
    double size_qAvg = PktsToMB(m_qAvg);

    if (size_qAvg >= m_minTh && size_qAvg < m_maxTh)
    {
        p = m_lambda * (size_qAvg - m_minTh);
    }
    else if (size_qAvg >= m_maxTh)
    {
        p = 1.0;
    }

    if (p > 1.0)
    {
        p = 1.0;
    }

    return p;
}

double
PRedQueueDisc::ModifyP(double p, uint32_t size)
{
    NS_LOG_FUNCTION(this << p << size);

    auto count1 = (double)m_count;

    if (GetMaxSize().GetUnit() == QueueSizeUnit::BYTES)
    {
        count1 = (double)(m_countBytes / m_meanPktSize);
    }

    if (m_isWait)
    {
        if (count1 * p < 1.0)       { p = 0.0; }
        else if (count1 * p < 2.0)  { p /= (2.0 - count1 * p); }
        else                         { p = 1.0; }
    }
    else
    {
        if (count1 * p < 1.0)  { p /= (1.0 - count1 * p); }
        else                    { p = 1.0; }
    }

    if ((GetMaxSize().GetUnit() == QueueSizeUnit::BYTES) && (p < 1.0))
    {
        p = (p * size) / m_meanPktSize;
    }

    if (p > 1.0) { p = 1.0; }

    return p;
}


Ptr<QueueDiscItem>
PRedQueueDisc::DoDequeue()
{
    NS_LOG_FUNCTION(this);

    if (GetInternalQueue(0)->IsEmpty())
    {
        NS_LOG_LOGIC("Queue empty");
        m_idle     = 1;
        m_idleTime = Simulator::Now();
        return nullptr;
    }

    m_idle = 0;
    Ptr<QueueDiscItem> item = GetInternalQueue(0)->Dequeue();

    // QLA goodput: count every byte that actually leaves the queue.
    if (m_useQla && item)
    {
        m_qlaGoodputBytes += item->GetSize();
    }

    NS_LOG_LOGIC("Popped " << item);
    NS_LOG_LOGIC("Number packets " << GetInternalQueue(0)->GetNPackets());
    NS_LOG_LOGIC("Number bytes "   << GetInternalQueue(0)->GetNBytes());

    return item;
}

Ptr<const QueueDiscItem>
PRedQueueDisc::DoPeek()
{
    NS_LOG_FUNCTION(this);

    if (GetInternalQueue(0)->IsEmpty())
    {
        NS_LOG_LOGIC("Queue empty");
        return nullptr;
    }

    Ptr<const QueueDiscItem> item = GetInternalQueue(0)->Peek();

    NS_LOG_LOGIC("Number packets " << GetInternalQueue(0)->GetNPackets());
    NS_LOG_LOGIC("Number bytes "   << GetInternalQueue(0)->GetNBytes());

    return item;
}

bool
PRedQueueDisc::CheckConfig()
{
    NS_LOG_FUNCTION(this);

    if (GetNQueueDiscClasses() > 0)
    {
        NS_LOG_ERROR("PRedQueueDisc cannot have classes");
        return false;
    }

    if (GetNPacketFilters() > 0)
    {
        NS_LOG_ERROR("PRedQueueDisc cannot have packet filters");
        return false;
    }

    if (GetNInternalQueues() == 0)
    {
        AddInternalQueue(
            CreateObjectWithAttributes<DropTailQueue<QueueDiscItem>>(
                "MaxSize", QueueSizeValue(GetMaxSize())));
    }

    if (GetNInternalQueues() != 1)
    {
        NS_LOG_ERROR("PRedQueueDisc needs 1 internal queue");
        return false;
    }

    return true;
}

} // namespace ns3