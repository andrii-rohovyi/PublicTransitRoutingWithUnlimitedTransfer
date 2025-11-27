#pragma once

#include <vector>
#include <set>
#include <memory>
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
        Label() : ExternalKHeapElement(), arrivalTime(intMax), parent(noVertex), parentState(State::AtStop), timeStamp(-1), reachedByWalking(false), tripId(-1) {}
        inline void reset(int ts) {
            arrivalTime = intMax;
            parent = noVertex;
            parentState = State::AtStop;
            timeStamp = ts;
            reachedByWalking = false;
            tripId = -1;
        }
        inline bool hasSmallerKey(const Label* other) const noexcept { return arrivalTime < other->arrivalTime; }
        int arrivalTime;
        Vertex parent;
        State parentState;
        int timeStamp;
        bool reachedByWalking;  // True if reached via pure walking (no transit used)
        int tripId;
    };

public:
    TimeDependentDijkstraStateful(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices() * 2)
        , label(g.numVertices() * 2)
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
    }

    inline void addSource(const Vertex s, const int time) noexcept {
        Label& L = getLabel(indexOf(s, State::AtStop));
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
        const Label& a = label[indexOf(v, State::AtStop)];
        const Label& b = label[indexOf(v, State::OnVehicle)];
        return ((a.timeStamp == timeStamp && a.arrivalTime != never) || (b.timeStamp == timeStamp && b.arrivalTime != never));
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        const Label& a = label[indexOf(v, State::AtStop)];
        const Label& b = label[indexOf(v, State::OnVehicle)];
        int best = never;
        if (a.timeStamp == timeStamp && a.arrivalTime != never) best = std::min(best, a.arrivalTime);
        if (b.timeStamp == timeStamp && b.arrivalTime != never) best = std::min(best, b.arrivalTime);
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
        const Label& atStop = label[indexOf(target, State::AtStop)];
        const Label& onVehicle = label[indexOf(target, State::OnVehicle)];
        
        size_t idx;
        if (atStop.timeStamp == timeStamp && atStop.arrivalTime < never &&
            (onVehicle.timeStamp != timeStamp || onVehicle.arrivalTime >= never || atStop.arrivalTime <= onVehicle.arrivalTime)) {
            idx = indexOf(target, State::AtStop);
        } else {
            idx = indexOf(target, State::OnVehicle);
        }

        // Backtrack
        while (idx < label.size()) {
            const Label& L = label[idx];
            if (L.timeStamp != timeStamp) break;
            
            const Vertex v = Vertex(idx / 2);
            const State s = (idx % 2 == 1) ? State::OnVehicle : State::AtStop;
            
            path.push_back({v, s, L.arrivalTime});
            
            if (L.parent == noVertex) break;
            idx = indexOf(L.parent, L.parentState);
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

private:
    inline size_t indexOf(const Vertex v, const State s) const noexcept { return size_t(v) * 2 + (s == State::OnVehicle ? 1 : 0); }

    inline Label& getLabel(const size_t idx) noexcept {
        Label& L = label[idx];
        if (L.timeStamp != timeStamp) L.reset(timeStamp);
        return L;
    }

    template<typename STOP>
    inline void runRelaxation(const Vertex target, const STOP& stop) noexcept {
        while (!Q.empty()) {
            const Label* cur = Q.extractFront();
            const size_t uidx = size_t(cur - &label[0]);
            const Vertex u = Vertex(uidx / 2);
            const State s = (uidx % 2 == 1) ? State::OnVehicle : State::AtStop;

            settleCount++;
            
            // Target pruning: if current arrival time exceeds the target's best arrival time, stop
            // This ensures we explore all stops that could potentially improve the target arrival
            if (target != noVertex) {
                const Label& targetLabel = label[indexOf(target, State::AtStop)];
                if (targetLabel.timeStamp == timeStamp && cur->arrivalTime >= targetLabel.arrivalTime) {
                    break;  // No point exploring further - all remaining vertices have worse arrival times
                }
            }
            
            if (stop()) break;

            const int t = cur->arrivalTime;
            const bool curReachedByWalking = cur->reachedByWalking;
            
            // Allow alighting from vehicle to stop (zero cost state change)
            // This enables transfers at the same stop (OnVehicle -> AtStop -> Board next trip)
            if (s == State::OnVehicle) {
                relaxWalking(indexOf(u, State::AtStop), u, s, t, false);

                // Optimization: Allow continuing on the same vehicle (zero buffer)
                // We check if there's a trip departing at or very shortly after our arrival.
                // We MUST match the tripId to ensure we stay on the same vehicle.
                for (const Edge e : graph.edgesFrom(u)) {
                    const Vertex v = graph.get(ToVertex, e);
                    const auto& atf = graph.get(Function, e);
                    
                    // Find trip departing >= t
                    auto it = std::lower_bound(atf.discreteTrips.begin(), atf.discreteTrips.end(), t, 
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    
                    // Scan forward for matching tripId
                    for (; it != atf.discreteTrips.end(); ++it) {
                        if (it->tripId == cur->tripId) {
                             relaxTransit(indexOf(v, State::OnVehicle), u, s, it->arrivalTime, it->tripId);
                             break; // Found the trip segment
                        }
                    }
                }
            }
            
            // Check if we can reach the target via CoreCH backward distance
            // This is crucial for non-stop targets that aren't directly reachable via TD graph edges
            if (s == State::AtStop && u < numberOfStops && targetVertex != noVertex && initialTransfers) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxWalking(indexOf(targetVertex, State::AtStop), u, s, arrivalAtTarget, curReachedByWalking);
                }
            }
            
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);

                // 1) Scheduled (discrete) candidate
                // Can ONLY board if:
                //   a) Current vertex u is a stop (u < numberOfStops)
                //   b) We are at the stop (State::AtStop)
                if (u < numberOfStops && s == State::AtStop) {
                    // Apply the minimum transfer buffer when boarding from AtStop state.
                    // TD graph uses original departure times (no implicit shift), so we
                    // add the buffer to the arrival time to find eligible trips.
                    const int buffer = graph.getMinTransferTimeAt(u);
                    const auto& atf = graph.get(Function, e);
                    const int threshold = t + buffer;
                    
                    auto it = std::lower_bound(atf.discreteTrips.begin(), atf.discreteTrips.end(), threshold,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    
                    if (it != atf.discreteTrips.end()) {
                        const size_t idx = size_t(it - atf.discreteTrips.begin());
                        // Use suffix minimum to get the best (earliest) arrival among all eligible trips
                        int bestArrival = never;
                        if (!atf.suffixMinArrival.empty()) bestArrival = atf.suffixMinArrival[idx];
                        else bestArrival = it->arrivalTime;

                        if (bestArrival < never) {
                            // Find the trip that provides this arrival time
                            int bestTripId = -1;
                            for (auto scan = it; scan != atf.discreteTrips.end(); ++scan) {
                                if (scan->arrivalTime == bestArrival) {
                                    bestTripId = scan->tripId;
                                    break;
                                }
                            }
                            relaxTransit(indexOf(v, State::OnVehicle), u, s, bestArrival, bestTripId);
                        }
                    }
                }

                // 2) Walking candidate - always available
                const int walkArrival = graph.getWalkArrivalFrom(e, t);
                if (walkArrival < never) {
                    relaxWalking(indexOf(v, State::AtStop), u, s, walkArrival, curReachedByWalking && s == State::AtStop);
                }
            }
        }
    }

    inline void relaxTransit(const size_t vidx, const Vertex parent, const State parentState, const int newTime, const int tripId) noexcept {
        relaxCount++;
        Label& L = getLabel(vidx);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.parent = parent;
            L.parentState = parentState;
            L.reachedByWalking = false;  // Reached via transit
            L.tripId = tripId;
            Q.update(&L);
        }
    }

    inline void relaxWalking(const size_t vidx, const Vertex parent, const State parentState, const int newTime, const bool reachedByWalking) noexcept {
        relaxCount++;
        Label& L = getLabel(vidx);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.parent = parent;
            L.parentState = parentState;
            L.reachedByWalking = reachedByWalking;
            L.tripId = -1;
            Q.update(&L);
        }
    }

private:
    const Graph& graph;
    // Number of stops in the network (vertices 0 to numberOfStops-1 are stops)
    // Vertices >= numberOfStops are non-stop waypoints (can walk through but cannot board from)
    size_t numberOfStops;
    ExternalKHeap<2, Label> Q;
    std::vector<Label> label;
    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;  // Target vertex for backward distance lookups
};

