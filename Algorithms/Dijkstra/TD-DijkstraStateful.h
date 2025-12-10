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

// A two-state time-dependent Dijkstra that applies per-stop min transfer time (buffer)
// only when boarding a scheduled trip from an "at-stop" state. Continuing in-vehicle
// does not incur the buffer. Walking never incurs a buffer.

template<typename GRAPH, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStateful {
public:
    using Graph = GRAPH;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

    enum class State : uint8_t { AtStop = 0, OnVehicle = 1 };

    struct Label : public ExternalKHeapElement {
        Label() : ExternalKHeapElement(), arrivalTime(intMax), parent(noVertex), parentState(State::AtStop), timeStamp(-1), reachedByWalking(false), tripId(-1), stopIndex(0), vertex(noVertex), state(State::AtStop) {}
        
        inline void reset(int ts, Vertex v, State s) {
            arrivalTime = intMax;
            parent = noVertex;
            parentState = State::AtStop;
            timeStamp = ts;
            reachedByWalking = false;
            tripId = -1;
            stopIndex = 0;
            vertex = v;
            state = s;
        }
        
        inline bool hasSmallerKey(const Label* other) const noexcept { return arrivalTime < other->arrivalTime; }
        
        int arrivalTime;
        Vertex parent;
        State parentState;
        int timeStamp;
        bool reachedByWalking; 
        int tripId;
        uint16_t stopIndex; // Index of the CURRENT stop in the trip sequence
        Vertex vertex;
        State state;
    };

public:
    TimeDependentDijkstraStateful(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices() * 2)
        , atStopLabels(g.numVertices())
        , globalVehicleLabels(g.getNumStopEvents(), nullptr) // Flat array for O(1) state lookup
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
        
        // Fast reset of the flat array
        for (const uint32_t idx : touchedGlobalIndices) {
            globalVehicleLabels[idx] = nullptr;
        }
        touchedGlobalIndices.clear();
        
        labelPool.clear();
    }

    inline void addSource(const Vertex s, const int time) noexcept {
        Label& L = getAtStopLabel(s);
        if (time < L.arrivalTime) {
            L.arrivalTime = time;
            L.reachedByWalking = true;
            Q.update(&L);
        }
    }

    template<typename STOP = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const STOP& stop = NoOperation) noexcept {
        clear();
        targetVertex = target;  
        
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
        return (a.timeStamp == timeStamp && a.arrivalTime != never);
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        const Label& a = atStopLabels[v];
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

        // 1. Find best ending state
        const Label* bestLabel = nullptr;
        int bestTime = never;

        const Label& atStop = atStopLabels[target];
        if (atStop.timeStamp == timeStamp && atStop.arrivalTime < bestTime) {
            bestTime = atStop.arrivalTime;
            bestLabel = &atStop;
        }

        if (!bestLabel) return path;

        // 2. Backtrack
        const Label* cur = bestLabel;
        while (cur) {
            path.push_back({cur->vertex, cur->state, cur->arrivalTime});
            
            if (cur->parent == noVertex) break;
            
            if (cur->parentState == State::AtStop) {
                cur = &atStopLabels[cur->parent];
            } else {
                // Parent is OnVehicle.
                if (cur->state == State::OnVehicle) {
                    // Case A: Continue (OnVehicle -> OnVehicle)
                    if (cur->stopIndex > 0) {
                        const uint32_t parentAbsIndex = graph.getTripOffset(cur->tripId) + (cur->stopIndex - 1);
                        cur = globalVehicleLabels[parentAbsIndex];
                    } else {
                        break; 
                    }
                } else {
                    // Case B: Alight (OnVehicle -> AtStop)
                    uint32_t startOffset = graph.getTripOffset(cur->tripId);
                    uint32_t endOffset = graph.getTripOffset(cur->tripId + 1);
                    
                    Label* found = nullptr;
                    for (uint32_t idx = startOffset; idx < endOffset; ++idx) {
                        Label* candidate = globalVehicleLabels[idx];
                        if (candidate && candidate->timeStamp == timeStamp && candidate->vertex == cur->parent) {
                            found = candidate;
                            break;
                        }
                    }
                    cur = found;
                }
            }
            
            if (!cur || cur->timeStamp != timeStamp) break;
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
            
            // Global Target Pruning
            if constexpr (TARGET_PRUNING) {
                if (target != noVertex) {
                    const Label& targetLabel = atStopLabels[target];
                    if (targetLabel.timeStamp == timeStamp && cur->arrivalTime >= targetLabel.arrivalTime) {
                        continue; 
                    }
                }
            }
            
            if (stop()) break;

            const int t = cur->arrivalTime;
            const bool curReachedByWalking = cur->reachedByWalking;
            
            // --- EXPANSION: FROM VEHICLE ---
            if (s == State::OnVehicle) {
                // 1. Alight: OnVehicle -> AtStop
                relaxWalking(u, u, s, t, false, cur->tripId);

                // 2. Continue: OnVehicle -> OnVehicle (O(1) Optimization)
                Vertex nextStop;
                int nextArrival;
                
                // Use the O(1) graph accessor
                if (graph.getNextStop(cur->tripId, cur->stopIndex, nextStop, nextArrival)) {
                    relaxTransit(nextStop, u, s, nextArrival, cur->tripId, cur->stopIndex + 1);
                }
            }
            
            // --- EXPANSION: CH WALKING SHORTCUTS ---
            if (s == State::AtStop && u < numberOfStops && targetVertex != noVertex && initialTransfers) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxWalking(targetVertex, u, s, arrivalAtTarget, curReachedByWalking, -1);
                }
            }
            
            // --- EXPANSION: FROM STOP (Walking & Boarding) ---
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);

                // 1) BOARDING A VEHICLE
                if (u < numberOfStops && s == State::AtStop) {
                    const auto& atf = graph.get(Function, e);
                    
                    // Implicit buffers are handled by graph data. Search for first trip >= t.
                    auto it = std::lower_bound(atf.discreteTrips.begin(), atf.discreteTrips.end(), t,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    
                    // A. Initialize Target Upper Bound
                    int targetUpperBound = never;
                    if constexpr (TARGET_PRUNING) {
                        if (target != noVertex) {
                            const Label& targetLabel = atStopLabels[target];
                            if (targetLabel.timeStamp == timeStamp) {
                                targetUpperBound = targetLabel.arrivalTime;
                            }
                        }
                    }

                    // B. Initialize Local Upper Bound
                    int bestLocalArrival = never;
                    const int bufferAtV = (v < numberOfStops) ? graph.getMinTransferTimeAt(v) : 0;

                    // C. Iterate
                    for (; it != atf.discreteTrips.end(); ++it) {
                        
                        // --- PRUNING BLOCK ---
                        const size_t idx = std::distance(atf.discreteTrips.begin(), it);
                        const int minPossibleArrival = atf.suffixMinArrival[idx];

                        // Local Pruning
                        if (bestLocalArrival != never) {
                            if (minPossibleArrival > bestLocalArrival + bufferAtV) {
                                break; 
                            }
                        }

                        // Target Pruning
                        if constexpr (TARGET_PRUNING) {
                            if (targetUpperBound != never) {
                                if (minPossibleArrival >= targetUpperBound) {
                                    break;
                                }
                            }
                        }
                        // ---------------------

                        // Update local best
                        if (it->arrivalTime < bestLocalArrival) {
                            bestLocalArrival = it->arrivalTime;
                        }

                        // O(1) Index Lookup
                        const uint16_t nextStopIndex = it->departureStopIndex + 1;
                        relaxTransit(v, u, s, it->arrivalTime, it->tripId, nextStopIndex);
                    }
                }

                // 2) WALKING
                const int walkArrival = graph.getWalkArrivalFrom(e, t);
                if (walkArrival < never) {
                    relaxWalking(v, u, s, walkArrival, curReachedByWalking && s == State::AtStop, -1);
                }
            }
        }
    }

    inline void relaxTransit(const Vertex v, const Vertex parent, const State parentState, const int newTime, const int tripId, const uint16_t stopIndex) noexcept {
        relaxCount++;
        
        // O(1) Absolute Index Calculation
        const uint32_t absIndex = graph.getTripOffset(tripId) + stopIndex;
        
        Label* L = globalVehicleLabels[absIndex];
        
        if (!L) {
            labelPool.emplace_back();
            L = &labelPool.back();
            L->reset(timeStamp, v, State::OnVehicle);
            L->tripId = tripId;
            globalVehicleLabels[absIndex] = L;
            touchedGlobalIndices.push_back(absIndex);
        } else if (L->timeStamp != timeStamp) {
            L->reset(timeStamp, v, State::OnVehicle);
            L->tripId = tripId;
        }

        if (L->arrivalTime > newTime) {
            L->arrivalTime = newTime;
            L->parent = parent;
            L->parentState = parentState;
            L->reachedByWalking = false;
            L->tripId = tripId;
            L->stopIndex = stopIndex;
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
            L.tripId = parentTripId; 
            Q.update(&L);
        }
    }

private:
    const Graph& graph;
    const size_t numberOfStops;
    ExternalKHeap<2, Label> Q;
    std::vector<Label> atStopLabels;
    std::vector<Label*> globalVehicleLabels; // Indexed by [TripOffset + StopIndex]
    std::vector<uint32_t> touchedGlobalIndices; // For fast reset
    std::deque<Label> labelPool;
    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
};