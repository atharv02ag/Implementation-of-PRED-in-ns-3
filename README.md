# PRED: Performance-oriented Random Early Detection for Consistently Stable Performance in Datacenters

## Authors - Atharv Agarwal, Daksh Agarwal

NS-3 implementation of **PRED**, a dynamic AQM algorithm for datacenter network.

The implementation is evaluated on two topologies: a simple N-to-1 incast scenario and a full 128-server leaf-spine datacenter fabric.

---

## Relevant files in the repository

```
.
├── scratch/
│   ├── incast_ecn.cc          # N-to-1 incast, standard step-ECN baseline
│   ├── incast_red.cc          # N-to-1 incast, standard RED
│   ├── incast_pred.cc         # N-to-1 incast, full PRED (FCS + QLA)
│   ├── incast_fcs.cc          # N-to-1 incast, FCS only (no QLA)
│   ├── incast_qla.cc          # N-to-1 incast, QLA only (no FCS)
│   ├── leaf_spine_ecn.cc      # Leaf-spine topology, step-ECN baseline
│   ├── leaf_spine_red.cc      # Leaf-spine topology, standard RED
│   ├── leaf_spine_pred.cc     # Leaf-spine topology, full PRED
│   ├── leaf_spine_fcs.cc      # Leaf-spine topology, FCS only
│   └── leaf_spine_qla.cc      # Leaf-spine topology, QLA only
│
├── src/traffic-control/model/
│   ├── pred-queue-disc.cc     # PRED queue disc implementation
│   └── pred-queue-disc.h      # PRED queue disc header
│
├── incast_results/            # Raw CSV logs from incast simulations
│  
│
└── leaf_spine_results/        # Raw CSV logs from leaf-spine simulations
   
```

## Installation

### 1. Clone this repository in place of (or as) your ns-3 tree

This repository **is** the ns-3 source tree with PRED added. Clone it directly:

```bash
git clone https://github.com/<your-username>/<your-repo>.git ns-3-pred
cd ns-3-pred
```

### 2. Configure the build

```bash
./ns3 configure --enable-examples --enable-tests
```

### 3. Build

```bash
./ns3 build
```

Build time is typically 5–15 minutes depending on your machine. Only the traffic-control module and scratch files need to recompile after changes to PRED.

To rebuild only the PRED module after editing `pred-queue-disc.cc` or `pred-queue-disc.h`:

```bash
./ns3 build scratch/incast_pred   # or whichever target you are working on
```

---

## Running simulations

### Incast simulations

```bash
# Standard ECN baseline
./ns3 run incast_ecn

# Full PRED
./ns3 run incast_pred

```

### Leaf-spine simulations

```bash
# Standard ECN baseline
./ns3 run leaf_spine_ecn

# Full PRED
./ns3 run leaf_spine_pred

```

### Output files

Each simulation writes two CSV files to the working directory:

| File | Contents |
|---|---|
| `fct_<label>.csv` | completion time and size of flow |
| `queue_<label>.csv` | queue depth sampled every 50 µs |

Pre-generated results from our tests are in `incast_results/` and `leaf_spine_results/`.

---

## Key parameters

These can be set as NS-3 attributes on `ns3::PRedQueueDisc`:

| Attribute | Default | Description |
|---|---|---|
| `Lambda` | `1.0` | Base λ — peak drop probability at maxTh for a single flow |
| `MinTh` | `0.1` MB | Queue threshold below which no marking occurs |
| `MaxTh` | `0.5` MB | Queue threshold above which forced marking occurs |
| `UseFCS` | `true` | Enable Flow Concurrent Stabilizer |
| `UseQLA` | `true` | Enable Queue-Length-Aware Learning |
| `QlaBeta` | `0.4` | Utility weight β between throughput and queue length |
| `QlaLambdaMin` | `0.05` | λ floor below which QLA adjusts minTh instead |
| `QlaDeltaLambda` | `0.025` | Per-step λ adjustment Δλ |
| `QlaDeltaMinK` | `0.0075` MB | Per-step minTh adjustment Δ_minK |
| `TFCS` | auto | FCS period length (default: 1.25 × RTT) |
| `TQLA` | auto | QLA period length (default: 5 × RTT) |

Set attributes from a simulation script:

```cpp
tch.SetRootQueueDisc(
    "ns3::PRedQueueDisc",
    "Lambda",  DoubleValue(1.6),
    "MinTh",   DoubleValue(10.0 * 1500.0 / 1e6),
    "MaxTh",   DoubleValue(0.5),
    "UseFCS",  BooleanValue(true),
    "UseQLA",  BooleanValue(true));
```

---

## References

**PRED paper**

> *PRED: Performance-oriented Random Early Detection for Consistently Stable Performance in Datacenters*
> [https://dl.acm.org/doi/10.5555/3767955.3767956](https://dl.acm.org/doi/10.5555/3767955.3767956)

**NS-3 simulator**

> *ns-3: A Discrete-Event Network Simulator for Internet Systems*.
> [https://www.nsnam.org](https://www.nsnam.org)

**DCTCP**

> *Data Center TCP (DCTCP)*.
> [https://dl.acm.org/doi/10.1145/1851182.1851192](https://dl.acm.org/doi/10.1145/1851182.1851192)

**RED (Random Early Detection)**

> *Random Early Detection Gateways for Congestion Avoidance*.
> IEEE/ACM Transactions on Networking, 1(4):397–413, August 1993.