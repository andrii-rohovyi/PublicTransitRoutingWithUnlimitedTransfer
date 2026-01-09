#pragma once

#include <vector>
#include <memory>
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/BucketQuery.h"
#include "Profiler.h"

template<typename GRAPH, typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStatefulBucketCH {
public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;

    // KEY CHANGE: Use BucketQuery instead of Query
    using BucketCHInitialTransfers = CH::BucketQuery<CHGraph, true, false>;

    enum class State : uint8_t { AtStop = 0, OnVehicle = 1 };

    struct NodeLabel : public ExternalKHeapElement {
        NodeLabel() : ExternalKHeapElement(), arrivalTime(intMax), timeStamp(-1),
                      parent(noVertex), parentState(State::AtStop), reachedByWalking(false) {}

        inline void reset(int ts) {
            arrivalTime = intMax;
            timeStamp = ts;
            parent = noVertex;
            parentState = State::AtStop;
            reachedByWalking = false;
        }

        inline bool hasSmallerKey(const NodeLabel* other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }

        int arrivalTime;
        int timeStamp;
        Vertex parent;
        State parentState;
        bool reachedByWalking;
    };

    struct VehicleLabel {
        int arrivalTime = intMax;
        int timeStamp = -1;
    };

public:
    TimeDependentDijkstraStatefulBucketCH(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices())
        , nodeLabels(g.numVertices())
        , globalVehicleLabels(g.getNumStopEvents())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex) {
            if (chData) {
                // KEY CHANGE: Create BucketQuery - this builds buckets for all stops
                initialTransfers = std::make_unique<BucketCHInitialTransfers>(*chData, FORWARD, numberOfStops);
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
        NodeLabel& L = getNodeLabel(s);
        if (time < L.arrivalTime) {
            L.arrivalTime = time;
            L.reachedByWalking = true;
            L.parent = noVertex;
            L.parentState = State::AtStop;
            Q.update(&L);
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

            // Get forward POIs (stops reachable from source)
            for (const Vertex stopVertex : initialTransfers->getForwardPOIs()) {
                const int arrivalTime = departureTime + initialTransfers->getForwardDistance(stopVertex);
                addSource(stopVertex, arrivalTime);
            }

            // Check direct source-to-target walk
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
        if (v >= nodeLabels.size()) return false;
        const NodeLabel& a = nodeLabels[v];
        return (a.timeStamp == timeStamp && a.arrivalTime != never);
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        if (v >= nodeLabels.size()) return never;
        const NodeLabel& a = nodeLabels[v];
        if (a.timeStamp == timeStamp) return a.arrivalTime;
        return never;
    }

    inline int getSettleCount() const noexcept { return settleCount; }
    inline int getRelaxCount() const noexcept { return relaxCount; }
    inline double getElapsedMilliseconds() const noexcept { return timer.elapsedMilliseconds(); }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline NodeLabel& getNodeLabel(const Vertex v) noexcept {
        NodeLabel& L = nodeLabels[v];
        if (L.timeStamp != timeStamp) L.reset(timeStamp);
        return L;
    }

    template<typename STOP>
    inline void runRelaxation(const Vertex target, const STOP& stop) noexcept {
        profiler.startPhase(TDD::PHASE_MAIN_LOOP);

        while (!Q.empty()) {
            const NodeLabel* cur = Q.extractFront();
            const Vertex u = Vertex(cur - nodeLabels.data());
            const int t = cur->arrivalTime;

            settleCount++;
            profiler.countMetric(TDD::METRIC_SETTLES);

            int targetUpperBound = never;
            if constexpr (TARGET_PRUNING) {
                if (target != noVertex) {
                    const NodeLabel& targetLabel = nodeLabels[target];
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

            // Backward transfer to target via Bucket-CH
            if (targetVertex != noVertex && initialTransfers && u < numberOfStops) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxWalking(targetVertex, u, State::AtStop, arrivalAtTarget, curReachedByWalking);
                }
            }

            // Scan Edges
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);

                if (u < numberOfStops) {
                    const auto& atf = graph.get(Function, e);

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

                // Walk
                const int walkArrival = graph.getWalkArrivalFrom(e, t);
                if (walkArrival < never) {
                    relaxWalking(v, u, State::AtStop, walkArrival, curReachedByWalking);
                }
            }
        }
        profiler.donePhase(TDD::PHASE_MAIN_LOOP);
    }

    inline void scanTrip(const int tripId, const uint16_t startStopIndex, const int arrivalAtStart, const Vertex boardStop, const int targetUpperBound) noexcept {
        int currentArrivalTime = arrivalAtStart;

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

            const auto& currentLeg = graph.getTripLeg(idx);
            Vertex currentStopVertex = currentLeg.stopId;

            relaxWalking(currentStopVertex, boardStop, State::OnVehicle, currentArrivalTime, false);

            if (idx + 1 < endAbsIndex) {
                const auto& nextLeg = graph.getTripLeg(idx + 1);
                currentArrivalTime = nextLeg.arrivalTime;
            }
        }
    }

    inline void relaxWalking(const Vertex v, const Vertex parent, const State parentState, const int newTime, const bool reachedByWalking) noexcept {
        relaxCount++;
        profiler.countMetric(TDD::METRIC_RELAXES_WALKING);

        NodeLabel& L = getNodeLabel(v);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.reachedByWalking = reachedByWalking;
            L.parent = parent;
            L.parentState = parentState;
            Q.update(&L);
            profiler.countMetric(TDD::METRIC_ENQUEUES);
        }
    }

private:
    const Graph& graph;
    const size_t numberOfStops;
    ExternalKHeap<2, NodeLabel> Q;

    std::vector<NodeLabel> nodeLabels;
    std::vector<VehicleLabel> globalVehicleLabels;

    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;

    // KEY CHANGE: BucketQuery instead of Query
    std::unique_ptr<BucketCHInitialTransfers> initialTransfers;

    Vertex targetVertex;
    Profiler profiler;
};