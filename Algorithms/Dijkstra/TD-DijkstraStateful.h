#pragma once

#include <vector>
#include <set>
#include <memory>
#include <deque>
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/CHQuery.h"

// A two-state time-dependent Dijkstra that applies per-stop min transfer time (buffer)
// only when boarding a scheduled trip from an "at-stop" state. Continuing in-vehicle
// does not incur the buffer. Walking never incurs a buffer.

template<typename GRAPH, bool DEBUG = false>
class TimeDependentDijkstraStateful {
public:
    using Graph = GRAPH;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

    enum class State : uint8_t { AtStop = 0, OnVehicle = 1 };

    struct Label : public ExternalKHeapElement {
        Label() : ExternalKHeapElement(), arrivalTime(intMax), parent(noVertex), parentState(State::AtStop), timeStamp(-1), reachedByWalking(false), tripId(-1), vertex(noVertex), state(State::AtStop) {}
        inline void reset(int ts, Vertex v, State s) {
            arrivalTime = intMax;
            parent = noVertex;
            parentState = State::AtStop;
            timeStamp = ts;
            reachedByWalking = false;
            tripId = -1;
            vertex = v;
            state = s;
        }
        inline bool hasSmallerKey(const Label* other) const noexcept { return arrivalTime < other->arrivalTime; }
        int arrivalTime;
        Vertex parent;
        State parentState;
        int timeStamp;
        bool reachedByWalking;  // True if reached via pure walking (no transit used)
        int tripId;
        Vertex vertex;
        State state;
    };

public:
    TimeDependentDijkstraStateful(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices() * 2)
        , atStopLabels(g.numVertices())
        , onVehicleLabels(g.numVertices())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex) {
            if (chData) {
                initialTransfers = std::make_unique<CoreCHInitialTransfers>(*chData, FORWARD, numberOfStops);
            }
        }

    inline void clear() noexcept {
        Q.clear();
        timeStamp++;
        settleCount = 0;
        relaxCount = 0;
        timer.restart();
        targetVertex = noVertex;
        
        for (const Vertex v : touchedOnVehicleVertices) {
            onVehicleLabels[v].clear();
        }
        touchedOnVehicleVertices.clear();
        labelPool.clear();
    }

    inline void addSource(const Vertex s, const int time) noexcept {
        Label& L = getAtStopLabel(s);
        if (time < L.arrivalTime) {
            L.arrivalTime = time;
            L.reachedByWalking = true;  // Source is reached by walking
            Q.update(&L);
        }
    }

    template<typename STOP = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const STOP& stop = NoOperation) noexcept {
        clear();
        targetVertex = target;  // Store target for backward distance lookups
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
        runRelaxation(target, stop);
    }

    inline bool reachable(const Vertex v) const noexcept {
        const Label& a = atStopLabels[v];
        bool reachable = (a.timeStamp == timeStamp && a.arrivalTime != never);
        if (reachable) return true;
        
        if (v < onVehicleLabels.size()) {
            for (const auto& pair : onVehicleLabels[v]) {
                if (pair.second->timeStamp == timeStamp && pair.second->arrivalTime != never) return true;
            }
        }
        return false;
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        int best = never;
        const Label& a = atStopLabels[v];
        if (a.timeStamp == timeStamp && a.arrivalTime != never) best = std::min(best, a.arrivalTime);
        
        if (v < onVehicleLabels.size()) {
            for (const auto& pair : onVehicleLabels[v]) {
                const Label* b = pair.second;
                if (b->timeStamp == timeStamp && b->arrivalTime != never) best = std::min(best, b->arrivalTime);
            }
        }
        return best;
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

        // Find best ending state
        const Label* bestLabel = nullptr;
        int bestTime = never;

        const Label& atStop = atStopLabels[target];
        if (atStop.timeStamp == timeStamp && atStop.arrivalTime < bestTime) {
            bestTime = atStop.arrivalTime;
            bestLabel = &atStop;
        }

        if (target < onVehicleLabels.size()) {
            for (const auto& pair : onVehicleLabels[target]) {
                const Label* L = pair.second;
                if (L->timeStamp == timeStamp && L->arrivalTime < bestTime) {
                    bestTime = L->arrivalTime;
                    bestLabel = L;
                }
            }
        }

        if (!bestLabel) return path;

        // Backtrack
        const Label* cur = bestLabel;
        while (cur) {
            path.push_back({cur->vertex, cur->state, cur->arrivalTime});
            
            if (cur->parent == noVertex) break;
            
            // Find parent label
            if (cur->parentState == State::AtStop) {
                cur = &atStopLabels[cur->parent];
            } else {
                // Parent is OnVehicle.
                if (cur->state == State::OnVehicle) {
                    if (cur->parentState == State::OnVehicle) {
                        // Same trip: find parent label by tripId
                        auto it = onVehicleLabels[cur->parent].find(cur->tripId);
                        if (it != onVehicleLabels[cur->parent].end()) {
                            cur = it->second;
                        } else {
                            break; // Should not happen
                        }
                    } else {
                        // Boarding from AtStop
                        cur = &atStopLabels[cur->parent];
                    }
                } else {
                    // Current state is AtStop. Parent is OnVehicle (alighting) or AtStop (walking).
                    if (cur->parentState == State::OnVehicle) {
                        // Alighting. Use the stored tripId to find the parent OnVehicle label
                        auto it = onVehicleLabels[cur->parent].find(cur->tripId);
                        if (it != onVehicleLabels[cur->parent].end()) {
                            cur = it->second;
                        } else {
                            break; // Should not happen
                        }
                    } else {
                        // Walking
                        cur = &atStopLabels[cur->parent];
                    }
                }
            }
            
            if (cur->timeStamp != timeStamp) break;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

private:
    inline Label& getAtStopLabel(const Vertex v) noexcept {
        Label& L = atStopLabels[v];
        if (L.timeStamp != timeStamp) L.reset(timeStamp, v, State::AtStop);
        return L;
    }

    template<typename STOP>
    inline void runRelaxation(const Vertex target, const STOP& stop) noexcept {
        while (!Q.empty()) {
            const Label* cur = Q.extractFront();
            const Vertex u = cur->vertex;
            const State s = cur->state;

            settleCount++;
            
            // Target pruning - prune when current arrival >= best arrival at target
            if (target != noVertex) {
                const Label& targetLabel = atStopLabels[target];
                if (targetLabel.timeStamp == timeStamp && cur->arrivalTime >= targetLabel.arrivalTime) {
                    // If we can't improve the AtStop label at the target, we can't improve the overall best time.
                    break; 
                }
            }
            
            if (stop()) break;

            const int t = cur->arrivalTime;
            const bool curReachedByWalking = cur->reachedByWalking;
            
            // Allow alighting from vehicle to stop (zero cost state change)
            if (s == State::OnVehicle) {
                // Alight: OnVehicle -> AtStop. Pass tripId to store in parentTripId for backtracking.
                relaxWalking(u, u, s, t, false, cur->tripId);

                // Optimization: Allow continuing on the same vehicle (zero buffer)
                const int buffer = (u < numberOfStops) ? graph.getMinTransferTimeAt(u) : 0;
                const int searchTime = t - buffer;

                for (const Edge e : graph.edgesFrom(u)) {
                    const Vertex v = graph.get(ToVertex, e);
                    const auto& atf = graph.get(Function, e);
                    
                    // Find trip departing >= searchTime
                    auto it = std::lower_bound(atf.discreteTrips.begin(), atf.discreteTrips.end(), searchTime, 
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    
                    // Scan forward for matching tripId
                    for (; it != atf.discreteTrips.end(); ++it) {
                        if (it->tripId == cur->tripId) {
                             relaxTransit(v, u, s, it->arrivalTime, it->tripId);
                             break; // Found the trip segment
                        }
                    }
                }
            }
            
            // Check if we can reach the target via CoreCH backward distance
            if (s == State::AtStop && u < numberOfStops && targetVertex != noVertex && initialTransfers) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxWalking(targetVertex, u, s, arrivalAtTarget, curReachedByWalking, -1);
                }
            }
            
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);

                // 1) Scheduled (discrete) candidate
                if (u < numberOfStops && s == State::AtStop) {
                    const auto& atf = graph.get(Function, e);
                    
                    // Find the first eligible trip
                    auto it = std::lower_bound(atf.discreteTrips.begin(), atf.discreteTrips.end(), t,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    
                    // Iterate while departure time is identical to the first found trip
                    // (This ensures we don't miss a "continuing" trip just because a "terminating" trip was sorted first)
                    if (it != atf.discreteTrips.end()) {
                        const int firstDeparture = it->departureTime;
                        for (; it != atf.discreteTrips.end() && it->departureTime == firstDeparture; ++it) {
                            relaxTransit(v, u, s, it->arrivalTime, it->tripId);
                        }
                    }
                }

                // 2) Walking candidate - always available
                const int walkArrival = graph.getWalkArrivalFrom(e, t);
                if (walkArrival < never) {
                    relaxWalking(v, u, s, walkArrival, curReachedByWalking && s == State::AtStop, -1);
                }
            }
        }
    }

    inline void relaxTransit(const Vertex v, const Vertex parent, const State parentState, const int newTime, const int tripId) noexcept {
        relaxCount++;
        
        Label* L = nullptr;
        auto& map = onVehicleLabels[v];
        auto it = map.find(tripId);
        if (it != map.end()) {
            L = it->second;
            if (L->timeStamp != timeStamp) L->reset(timeStamp, v, State::OnVehicle);
        } else {
            labelPool.emplace_back();
            L = &labelPool.back();
            L->reset(timeStamp, v, State::OnVehicle);
            L->tripId = tripId;
            map[tripId] = L;
            if (map.size() == 1) touchedOnVehicleVertices.push_back(v);
        }

        if (L->arrivalTime > newTime) {
            L->arrivalTime = newTime;
            L->parent = parent;
            L->parentState = parentState;
            L->reachedByWalking = false;
            L->tripId = tripId;
            Q.update(L);
        }
    }

    inline void relaxWalking(const Vertex v, const Vertex parent, const State parentState, const int newTime, const bool reachedByWalking, const int parentTripId) noexcept {
        relaxCount++;
        Label& L = getAtStopLabel(v);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.parent = parent;
            L.parentState = parentState;
            L.reachedByWalking = reachedByWalking;
            L.tripId = parentTripId; // Store parent trip ID for backtracking
            Q.update(&L);
        }
    }

private:
    const Graph& graph;
    size_t numberOfStops;
    ExternalKHeap<2, Label> Q;
    std::vector<Label> atStopLabels;
    std::vector<std::unordered_map<int, Label*>> onVehicleLabels;
    std::deque<Label> labelPool;
    std::vector<Vertex> touchedOnVehicleVertices;
    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
};

