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
#include "../../DataStructures/Graph/TimeDependentGraphJTS.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "Profiler.h"

// JumpTripSearch: Route-based trip scanning with binary search
//
// Key algorithm:
// 1. Use CoreCH for initial/final transfers
// 2. At each stop, iterate through ROUTES (not edges)
// 3. Binary search to find first catchable trip per route
// 4. Scan trip to relax all subsequent stops
//
// This is the corrected version using route-based iteration.

template<typename GRAPH, typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class JumpTripSearch {
public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

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
    JumpTripSearch(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
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

    struct PathEntry {
        Vertex vertex;
        State state;
        int arrivalTime;
        int tripId;
    };

    inline std::vector<PathEntry> getPath(const Vertex target) const noexcept {
        std::vector<PathEntry> path;
        if (!reachable(target)) return path;

        const NodeLabel& endNode = nodeLabels[target];
        Vertex curVertex = target;
        int curTime = endNode.arrivalTime;

        while (true) {
            path.push_back({curVertex, State::AtStop, curTime, -1});

            const NodeLabel& L = nodeLabels[curVertex];
            if (L.parent == noVertex) break;

            Vertex prevVertex = L.parent;
            State viaState = L.parentState;

            if (viaState == State::AtStop) {
                curVertex = prevVertex;
                curTime = nodeLabels[curVertex].arrivalTime;
            }
            else if (viaState == State::OnVehicle) {
                int boardTime = nodeLabels[prevVertex].arrivalTime;
                int alightTime = curTime;
                int foundTripId = -1;

                auto trips = graph.findMatchingTrip(prevVertex, curVertex, boardTime, alightTime);
                if (trips.tripId != -1) {
                    foundTripId = trips.tripId;
                    path.back().state = State::OnVehicle;
                    path.back().tripId = foundTripId;
                }

                curVertex = prevVertex;
                curTime = boardTime;
            }
        }

        std::reverse(path.begin(), path.end());
        return path;
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

            // 1. CoreCH Backward (if enabled)
            if (targetVertex != noVertex && initialTransfers && u < numberOfStops) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxWalking(targetVertex, u, State::AtStop, arrivalAtTarget, curReachedByWalking);
                }
            }

            // 2. Process stop: iterate through ROUTES serving this stop
            if (u < numberOfStops) {
                const auto& routesFromStop = graph.getRoutesFromStop(u);

                for (const auto& routeAtStop : routesFromStop) {
                    // Binary search to find the first trip on this route that we can catch
                    const auto* begin = graph.getRouteTripsBegin(routeAtStop);
                    const auto* end = graph.getRouteTripsEnd(routeAtStop);

                    auto it = std::lower_bound(begin, end, t,
                        [](const typename Graph::RouteTripInfo& trip, int time) {
                            return trip.departureTime < time;
                        });

                    if (it == end) {
                        continue; // No catchable trip on this route
                    }

                    // Target pruning: check if this trip can improve
                    if constexpr (TARGET_PRUNING) {
                        if (targetUpperBound != never) {
                            if (it->minArrivalTime >= targetUpperBound) continue;
                        }
                    }

                    // Scan this trip starting from the next stop after boarding
                    scanTrip(it->tripId, routeAtStop.stopIndexInRoute + 1,
                             it->arrivalTimeAtNextStop, u, targetUpperBound);
                }
            }

            // 3. Walk to neighbors
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);
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

public:
    inline const Profiler& getProfiler() const noexcept {
        return profiler;
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
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
    Profiler profiler;
};