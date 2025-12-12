#pragma once

#include <vector>
#include <set>
#include <memory>
#include <deque>
#include <algorithm>
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "Profiler.h"

// A two-state time-dependent Dijkstra that applies per-stop min transfer time (buffer)
// only when boarding a scheduled trip from an "at-stop" state. Continuing in-vehicle
// does not incur the buffer. Walking never incurs a buffer.
//
// OPTIMIZATIONS: 
// 1. Hot/Cold data splitting.
// 2. Hybrid Trip-Scanning.
// 3. Single-Stream Vehicle Updates.
// 4. Inner-loop Pruning.
// 5. Flattened Graph Data & Direct Pointers.
// 6. Optimized HotLabel (Packed reachedByWalking).

template<typename GRAPH, typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStateful {
public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

    enum class State : uint8_t { AtStop = 0, OnVehicle = 1 };

    // --- HOT LABEL (AtStop) ---
    // Packed to fit 16 bytes: 4 (arr) + 4 (ts) + 4 (vtx) + 1 (bool) + 3 (pad)
    struct HotLabel : public ExternalKHeapElement {
        HotLabel() : ExternalKHeapElement(), arrivalTime(intMax), timeStamp(-1), vertex(noVertex), reachedByWalking(false) {}

        inline void reset(int ts, Vertex v) {
            arrivalTime = intMax;
            timeStamp = ts;
            vertex = v;
            reachedByWalking = false;
        }

        inline bool hasSmallerKey(const HotLabel* other) const noexcept { return arrivalTime < other->arrivalTime; }

        int arrivalTime;
        int timeStamp;
        Vertex vertex;
        bool reachedByWalking; // Moved here for Hot access
    };

    // --- VEHICLE LABEL ---
    struct VehicleLabel {
        int arrivalTime = intMax; // 4
        int timeStamp = -1;       // 4
        int tripId = -1;          // 4
        Vertex parent = noVertex; // 4
    };

    // --- COLD LABEL (AtStop Only) ---
    struct ColdLabel {
        Vertex parent = noVertex;
        State parentState = State::AtStop;
        int tripId = -1;
        // reachedByWalking moved to HotLabel
    };

public:
    TimeDependentDijkstraStateful(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices())
        , atStopHot(g.numVertices())
        , atStopCold(g.numVertices())
        , globalVehicleLabels(g.getNumStopEvents())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex) {
            if (chData) {
                initialTransfers = std::make_unique<CoreCHInitialTransfers>(*chData, FORWARD, numberOfStops);
            }
        }

    inline void clear() noexcept {
        profiler.startPhase(TDD::PHASE_CLEAR);
        Q.clear();
        timeStamp++;
        settleCount = 0;
        relaxCount = 0;
        timer.restart();
        targetVertex = noVertex;
        profiler.donePhase(TDD::PHASE_CLEAR);
    }

    inline void addSource(const Vertex s, const int time) noexcept {
        HotLabel& L = getAtStopHot(s);
        if (time < L.arrivalTime) {
            L.arrivalTime = time;
            L.reachedByWalking = true; // Set directly
            Q.update(&L);
            
            ColdLabel& C = atStopCold[s];
            C.parent = noVertex;
            
            profiler.countMetric(TDD::METRIC_ENQUEUES);
        }
    }

    template<typename STOP = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const STOP& stop = NoOperation) noexcept {
        profiler.start();
        clear();
        targetVertex = target;  
        
        profiler.startPhase(TDD::PHASE_INITIALIZATION);
        if (initialTransfers) {
            initialTransfers->run(source, target);
            for (const Vertex stop : initialTransfers->getForwardPOIs()) {
                const int arrivalTime = departureTime + initialTransfers->getForwardDistance(stop);
                addSource(stop, arrivalTime);
            }
            if (target != noVertex) {
                const int dist = initialTransfers->getDistance();
                if (dist != INFTY) {
                    addSource(target, departureTime + dist);
                }
            }
        } else {
            addSource(source, departureTime);
        }
        profiler.donePhase(TDD::PHASE_INITIALIZATION);
        
        runRelaxation(target, stop);
        profiler.done();
    }

    inline bool reachable(const Vertex v) const noexcept {
        if (v >= atStopHot.size()) return false;
        const HotLabel& a = atStopHot[v];
        return (a.timeStamp == timeStamp && a.arrivalTime != never);
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        if (v >= atStopHot.size()) return never;
        const HotLabel& a = atStopHot[v];
        if (a.timeStamp == timeStamp) return a.arrivalTime;
        return never;
    }

    inline int getSettleCount() const noexcept { return settleCount; }
    inline int getRelaxCount() const noexcept { return relaxCount; }
    inline double getElapsedMilliseconds() const noexcept { return timer.elapsedMilliseconds(); }

    struct PathEntry {
        Vertex vertex;
        State state;
        int arrivalTime;
    };

    inline std::vector<PathEntry> getPath(const Vertex target) const noexcept {
        std::vector<PathEntry> path;
        if (!reachable(target)) return path;

        const HotLabel* bestLabel = nullptr;
        int bestTime = never;

        const HotLabel& atStop = atStopHot[target];
        if (atStop.timeStamp == timeStamp && atStop.arrivalTime < bestTime) {
            bestTime = atStop.arrivalTime;
            bestLabel = &atStop;
        }

        if (!bestLabel) return path;

        Vertex curVertex = bestLabel->vertex;
        State curState = State::AtStop;
        int curTime = bestLabel->arrivalTime;

        while (true) {
            path.push_back({curVertex, curState, curTime});
            
            if (curState == State::AtStop) {
                const ColdLabel& curCold = atStopCold[curVertex];
                const HotLabel& curHot = atStopHot[curVertex]; // Access reachedByWalking
                
                if (curCold.parent == noVertex) break;

                if (curHot.reachedByWalking) {
                    curVertex = curCold.parent;
                    curTime = atStopHot[curVertex].arrivalTime; 
                    curState = State::AtStop;
                } else {
                    int tripId = curCold.tripId;
                    Vertex prevStop = curCold.parent;
                    
                    curVertex = prevStop;
                    curState = State::OnVehicle;
                    
                    uint32_t startOffset = graph.getTripOffset(tripId);
                    uint32_t endOffset = graph.getTripOffset(tripId + 1);
                    for (uint32_t idx = startOffset; idx < endOffset; ++idx) {
                        const auto& leg = graph.getTripLeg(idx);
                        if (leg.stopId == prevStop) {
                             curTime = globalVehicleLabels[idx].arrivalTime;
                             break;
                        }
                    }
                }
            } else {
                break; 
            }
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

private:
    inline HotLabel& getAtStopHot(const Vertex v) noexcept {
        HotLabel& L = atStopHot[v];
        if (L.timeStamp != timeStamp) L.reset(timeStamp, v);
        return L;
    }

    template<typename STOP>
    inline void runRelaxation(const Vertex target, const STOP& stop) noexcept {
        profiler.startPhase(TDD::PHASE_MAIN_LOOP);
        
        while (!Q.empty()) {
            const HotLabel* cur = Q.extractFront();
            const Vertex u = cur->vertex;
            const int t = cur->arrivalTime;

            settleCount++;
            profiler.countMetric(TDD::METRIC_SETTLES);
            
            int targetUpperBound = never;
            if constexpr (TARGET_PRUNING) {
                if (target != noVertex) {
                    const HotLabel& targetLabel = atStopHot[target];
                    if (targetLabel.timeStamp == timeStamp) {
                        targetUpperBound = targetLabel.arrivalTime;
                        if (t >= targetUpperBound) {
                            profiler.countMetric(TDD::METRIC_PRUNED_LABELS);
                            continue; 
                        }
                    }
                }
            }
            
            if (stop()) break;

            const bool curReachedByWalking = cur->reachedByWalking;
            
            if (targetVertex != noVertex && initialTransfers && u < numberOfStops) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxWalking(targetVertex, u, State::AtStop, arrivalAtTarget, curReachedByWalking, -1);
                }
            }
            
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);

                if (u < numberOfStops) {
                    const auto& atf = graph.get(Function, e);
                    
                    // Direct pointer access to flattened trips
                    const DiscreteTrip* begin = graph.getTripsBegin(atf);
                    const DiscreteTrip* end = graph.getTripsEnd(atf);
                    const int* suffixBase = graph.getSuffixMinBegin(atf);

                    auto it = std::lower_bound(begin, end, t,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    
                    int bestLocalArrival = never;
                    const int bufferAtV = (v < numberOfStops) ? graph.getMinTransferTimeAt(v) : 0;

                    for (; it != end; ++it) {
                        const size_t idx = std::distance(begin, it);
                        const int minPossibleArrival = suffixBase[idx];

                        if (bestLocalArrival != never) {
                            if (minPossibleArrival > bestLocalArrival + bufferAtV) break; 
                        }
                        
                        if constexpr (TARGET_PRUNING) {
                            if (targetUpperBound != never) {
                                if (minPossibleArrival >= targetUpperBound) break;
                            }
                        }

                        if (it->arrivalTime < bestLocalArrival) {
                            bestLocalArrival = it->arrivalTime;
                        }

                        scanTrip(it->tripId, it->departureStopIndex + 1, it->arrivalTime, u, targetUpperBound);
                    }
                }

                const int walkArrival = graph.getWalkArrivalFrom(e, t);
                if (walkArrival < never) {
                    relaxWalking(v, u, State::AtStop, walkArrival, curReachedByWalking, -1);
                }
            }
        }
        profiler.donePhase(TDD::PHASE_MAIN_LOOP);
    }

    inline void scanTrip(const int tripId, const uint16_t startStopIndex, const int arrivalAtStart, const Vertex boardStop, const int targetUpperBound) noexcept {
        int currentArrivalTime = arrivalAtStart;
        Vertex parentStop = boardStop;
        
        uint32_t currentAbsIndex = graph.getTripOffset(tripId) + startStopIndex;
        uint32_t endAbsIndex = graph.getTripOffset(tripId + 1);
        
        for (uint32_t idx = currentAbsIndex; idx < endAbsIndex; ++idx) {
            if constexpr (TARGET_PRUNING) {
                if (targetUpperBound != never && currentArrivalTime >= targetUpperBound) return; 
            }

            VehicleLabel& L = globalVehicleLabels[idx];
            if (L.timeStamp == timeStamp && L.arrivalTime <= currentArrivalTime) {
                return;
            }
            
            L.arrivalTime = currentArrivalTime;
            L.timeStamp = timeStamp;
            L.tripId = tripId;
            L.parent = parentStop;

            const auto& currentLeg = graph.getTripLeg(idx);
            Vertex currentStopVertex = currentLeg.stopId;

            relaxWalking(currentStopVertex, currentStopVertex, State::OnVehicle, currentArrivalTime, false, tripId);
            
            if (idx + 1 < endAbsIndex) {
                parentStop = currentStopVertex;
                const auto& nextLeg = graph.getTripLeg(idx + 1);
                currentArrivalTime = nextLeg.arrivalTime;
            }
        }
    }

    inline void relaxWalking(const Vertex v, const Vertex parent, const State parentState, const int newTime, const bool reachedByWalking, const int parentTripId) noexcept {
        relaxCount++;
        profiler.countMetric(TDD::METRIC_RELAXES_WALKING);
        
        HotLabel& L = getAtStopHot(v);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.reachedByWalking = reachedByWalking;
            Q.update(&L);
            
            ColdLabel& C = atStopCold[v];
            C.parent = parent;
            C.parentState = parentState;
            C.tripId = parentTripId;
            
            profiler.countMetric(TDD::METRIC_ENQUEUES);
        }
    }

public:
    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    const Graph& graph;
    const size_t numberOfStops;
    ExternalKHeap<2, HotLabel> Q;
    
    std::vector<HotLabel> atStopHot;
    std::vector<ColdLabel> atStopCold;
    
    std::vector<VehicleLabel> globalVehicleLabels;

    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
    Profiler profiler;
};