/*
 * Copyright © 2011 Marcos Talau
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Marcos Talau (talau@users.sourceforge.net)
 *
 * Thanks to: Duy Nguyen<duy@soe.ucsc.edu> by RED efforts in NS3
 *
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
 * PORT NOTE: This code was ported from ns-2 (queue/red.h).  Almost all
 * comments also been ported from NS-2.
 */

#ifndef PRED_QUEUE_DISC_H
#define PRED_QUEUE_DISC_H

#include "queue-disc.h"

#include "ns3/boolean.h"
#include "ns3/data-rate.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"

#include <array>
#include <unordered_set>

namespace ns3
{

class TraceContainer;

class PRedQueueDisc : public QueueDisc
{
  public:
    static TypeId GetTypeId();

    PRedQueueDisc();
    ~PRedQueueDisc() override;

    enum
    {
        DTYPE_NONE,
        DTYPE_FORCED,
        DTYPE_UNFORCED,
    };

    void    SetPREDParams(double minTh, double maxTh, double lambda);
    int64_t AssignStreams(int64_t stream);

    static constexpr const char* UNFORCED_DROP = "Unforced drop";
    static constexpr const char* FORCED_DROP   = "Forced drop";
    static constexpr const char* UNFORCED_MARK = "Unforced mark";
    static constexpr const char* FORCED_MARK   = "Forced mark";

  protected:
    void DoDispose() override;

  private:
    
    bool DoEnqueue(Ptr<QueueDiscItem> item) override;
    Ptr<QueueDiscItem> DoDequeue() override;
    Ptr<const QueueDiscItem> DoPeek() override;
    bool CheckConfig() override;
    void InitializeParams() override;

    double Estimator(uint32_t nQueued, uint32_t m, double qAvg, double qW);
    bool   DropEarly(Ptr<QueueDiscItem> item, uint32_t qSize);
    double CalculatePNew();
    double ModifyP(double p, uint32_t size);

    uint32_t ComputeFlowId(Ptr<QueueDiscItem> item) const;
    void     FcsRecordArrival(Ptr<QueueDiscItem> item);
    void     FcsTimerExpired();
    void     FcsUpdateLambda();

    /**
     * @brief Periodic QLA timer — fires every T_QLA seconds.
     *
     * One call = one trial ending + next trial starting.
     * After every 4th trial, QlaDecide() is called and the window resets.
     */
    void QlaTimerExpired();

    /**
     * @brief Compute utility for one T_QLA period.
     *
     * @param avgQueueMB   mean queue length in MB over the period
     * @param goodputBps   bytes delivered to the link per second
     * @return U in [0, 1]
     */
    double QlaComputeUtility(double avgQueueMB, double goodputBps) const;

    /**
     * @brief Apply the TCT decision rule after 4 trials and update λ_QLA.
     *
     * Compares U[0]..U[3].  Calls QlaClampAndAdjustMinK() afterward.
     */
    void QlaDecide();

    /**
     * @brief Activate minK mode if needed.
     *
     * If λ_QLA falls below λ_min:
     *   λ_QLA ← λ_min
     *   minTh += Δ_minK   (widen the no-drop zone).
     */
    void QlaAdjustMinK();

    /* ================================================================== */
    /*  Member variables                                                   */
    /* ================================================================== */

    /* RED / PRED attributes */
    uint32_t m_meanPktSize;
    uint32_t m_idlePktSize;
    bool     m_isWait;
    bool     m_isUseFCS;
    double   m_minTh;           //!< Min threshold (MB) — may be adjusted by QLA
    double   m_maxTh;           //!< Max threshold (MB)
    double   m_qW;
    Time     m_interval;
    bool     m_isNs1Compat;
    DataRate m_linkBandwidth;
    Time     m_linkDelay;
    bool     m_useEcn;
    bool     m_useHardDrop;

    /* PRED λ */
    double m_lambda;            //!< Effective λ read by CalculatePNew
    double m_lambdaBase;        //!< Immutable user-configured base (FCS scales from this)
    double m_lambdaQla;         //!< λ_QLA: current QLA operating point
    double m_TFCS;              //!< FCS period (seconds)

    /* RED state */
    double   m_qAvg;
    double   m_ptc;
    double   m_vProb;
    uint32_t m_count;
    uint32_t m_countBytes;
    uint32_t m_old;
    uint32_t m_idle;
    int      m_cautious;
    Time     m_idleTime;

    /* FCS state */
    std::unordered_set<uint32_t> m_fcsBitmap;
    uint32_t m_fcsNLast;
    uint32_t m_fcsN;
    uint32_t m_fcsEstimatedN;
    EventId  m_fcsTimer;

    /* QLA hyperparameter attributes */
    bool   m_useQla;            //!< Enable QLA (default true)
    double m_qlaBeta;           //!< β weight (default 0.4)
    double m_Qleft;          //!< q_left in MB (default 15*1500/1e6)
    double m_qlaLambdaMin;      //!< λ_min (default 0.05)
    double m_qlaDeltaLambda;    //!< Δλ (default 0.025)
    double m_qlaDeltaMinK;      //!< Δ_minK in MB (default 5*1500/1e6)
    double m_TQLA;           //!< T_QLA in seconds

    /* QLA runtime state */
    uint32_t              m_qlaTrial;         //!< Current trial index 0..3
    std::array<double, 4> m_qlaU;             //!< Utility values for 4 trials
    uint64_t              m_qlaGoodputBytes;  //!< Bytes dequeued this trial period
    double                m_qlaQavgAccum;     //!< Accumulated qAvg (pkts) this trial
    uint64_t              m_qlaSampleCount;   //!< Number of qAvg samples this trial
    double                m_qlaCurrentProbe;  //!< λ probe active this trial
    EventId               m_qlaTimer;         //!< Recurring T_QLA timer

    /* Misc */
    Ptr<UniformRandomVariable> m_uv;
};

} // namespace ns3

#endif // PRED_QUEUE_DISC_H