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
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "Profiler.h"

namespace TDD {

// =========================================================================
// Multi-Criteria Time-Dependent Dijkstra (MC-TDD)
// =========================================================================
// This algorithm optimizes for TWO criteria:
//   1. Arrival Time (minimize)
//   2. Walking Distance (minimize)
//
// It maintains Pareto-optimal labels at each node, similar to MCR.
// =========================================================================

template<typename GRAPH, typename PROFILER = NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentMCDijkstra {
public:
    using Graph = GRAPH;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

    // =========================================================================
    // Label Structures
    // =========================================================================

    // Multi-criteria label for Pareto-optimal search
    struct MCLabel {
        int arrivalTime = never;
        int walkingDistance = INFTY;

        // Parent tracking for path reconstruction
        Vertex parentStop = noVertex;
        size_t parentIndex = -1;
        int parentDepartureTime = never;
        int tripId = -1;  // -1 means walking

        MCLabel() = default;

        MCLabel(int arr, int walk, Vertex parent = noVertex, size_t pIdx = -1, int pDep = never, int trip = -1)
            : arrivalTime(arr), walkingDistance(walk),
              parentStop(parent), parentIndex(pIdx), parentDepartureTime(pDep), tripId(trip) {}

        // Dominance check: this dominates other if better/equal in ALL criteria
        inline bool dominates(const MCLabel& other) const noexcept {
            return arrivalTime <= other.arrivalTime && walkingDistance <= other.walkingDistance;
        }

        // Strict dominance (for pruning)
        inline bool strictlyDominates(const MCLabel& other) const noexcept {
            return dominates(other) && (arrivalTime < other.arrivalTime || walkingDistance < other.walkingDistance);
        }

        // For heap ordering - use sum as priority (can be tuned)
        inline int getKey() const noexcept {
            return arrivalTime + walkingDistance;
        }

        inline bool operator<(const MCLabel& other) const noexcept {
            return getKey() < other.getKey();
        }
    };

    // Bag of Pareto-optimal labels for a node
    struct LabelBag {
        std::vector<MCLabel> labels;

        inline void clear() noexcept {
            labels.clear();
        }

        inline size_t size() const noexcept {
            return labels.size();
        }

        inline bool empty() const noexcept {
            return labels.empty();
        }

        inline const MCLabel& operator[](size_t i) const noexcept {
            return labels[i];
        }

        // Check if any label in bag dominates the given label
        inline bool dominates(const MCLabel& label) const noexcept {
            for (const MCLabel& existing : labels) {
                if (existing.dominates(label)) return true;
            }
            return false;
        }

        // Merge a new label, maintaining Pareto optimality
        // Returns true if the label was added (improved Pareto front)
        inline bool merge(const MCLabel& newLabel) noexcept {
            // Check if dominated by existing labels
            for (const MCLabel& existing : labels) {
                if (existing.dominates(newLabel)) return false;
            }

            // Remove labels dominated by the new one
            labels.erase(
                std::remove_if(labels.begin(), labels.end(),
                    [&newLabel](const MCLabel& existing) {
                        return newLabel.strictlyDominates(existing);
                    }),
                labels.end()
            );

            labels.push_back(newLabel);
            return true;
        }

        // Merge without dominance check (for source initialization)
        inline void mergeUndominated(const MCLabel& label) noexcept {
            labels.push_back(label);
        }

        // Get the best label by a specific criterion
        inline const MCLabel* getBestByArrival() const noexcept {
            if (labels.empty()) return nullptr;
            const MCLabel* best = &labels[0];
            for (const MCLabel& l : labels) {
                if (l.arrivalTime < best->arrivalTime) best = &l;
            }
            return best;
        }
    };

    // Heap element wrapper for priority queue
    struct HeapLabel : public ExternalKHeapElement {
        Vertex vertex;
        MCLabel label;

        HeapLabel() : vertex(noVertex) {}

        inline bool hasSmallerKey(const HeapLabel* other) const noexcept {
            return label.getKey() < other->label.getKey();
        }
    };

    // Vehicle (on-trip) label - minimal for scanning
    struct VehicleLabel {
        int arrivalTime = intMax;
        int walkingDistance = INFTY;
        int timeStamp = -1;

        inline bool dominates(int arr, int walk) const noexcept {
            return arrivalTime <= arr && walkingDistance <= walk;
        }
    };

    // Result structure matching RAPTOR::WalkingParetoLabel
    struct WalkingParetoLabel {
        int arrivalTime;
        int walkingDistance;
        size_t numberOfTrips;

        WalkingParetoLabel() : arrivalTime(never), walkingDistance(INFTY), numberOfTrips(0) {}
        WalkingParetoLabel(int arr, int walk, size_t trips = 0)
            : arrivalTime(arr), walkingDistance(walk), numberOfTrips(trips) {}
        WalkingParetoLabel(const MCLabel& label)
            : arrivalTime(label.arrivalTime), walkingDistance(label.walkingDistance), numberOfTrips(0) {}
    };

public:
    TimeDependentMCDijkstra(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , nodeBags(g.numVertices())
        , globalVehicleLabels(g.getNumStopEvents())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex)
        , sourceDepartureTime(0) {
            if (chData) {
                initialTransfers = std::make_unique<CoreCHInitialTransfers>(*chData, FORWARD, numberOfStops);
            }
            // Pre-allocate heap labels
            heapLabels.resize(g.numVertices() * 10); // Allow multiple labels per vertex
            heapLabelCount = 0;
        }

    inline void clear() noexcept {
        profiler.startPhase(PHASE_CLEAR);

        // Clear bags using timestamp
        timeStamp++;

        // Reset bags that were used
        for (Vertex v : touchedVertices) {
            nodeBags[v].clear();
        }
        touchedVertices.clear();

        // Clear heap
        heapLabelCount = 0;
        activeHeapLabels.clear();

        settleCount = 0;
        relaxCount = 0;
        timer.restart();
        targetVertex = noVertex;

        profiler.donePhase(PHASE_CLEAR);
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target, const size_t maxRounds = INFTY) noexcept {
        profiler.start();
        clear();

        targetVertex = target;
        sourceDepartureTime = departureTime;
        sourceVertex = source;

        profiler.startPhase(PHASE_INITIALIZATION);
        initialize(source, departureTime, target);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase(PHASE_MAIN_LOOP);
        runMainLoop();
        profiler.donePhase(PHASE_MAIN_LOOP);

        profiler.done();
    }

    inline std::vector<WalkingParetoLabel> getResults() const noexcept {
        return getResults(targetVertex);
    }

    inline std::vector<WalkingParetoLabel> getResults(const Vertex vertex) const noexcept {
        std::vector<WalkingParetoLabel> results;
        if (vertex >= nodeBags.size()) return results;

        const LabelBag& bag = nodeBags[vertex];
        for (const MCLabel& label : bag.labels) {
            results.emplace_back(label);
        }
        return results;
    }

    inline bool reachable(const Vertex v) const noexcept {
        if (v >= nodeBags.size()) return false;
        return !nodeBags[v].empty();
    }

    inline int getArrivalTime(const Vertex v) const noexcept {
        if (v >= nodeBags.size()) return never;
        const MCLabel* best = nodeBags[v].getBestByArrival();
        return best ? best->arrivalTime : never;
    }

    inline int getSettleCount() const noexcept { return settleCount; }
    inline int getRelaxCount() const noexcept { return relaxCount; }
    inline double getElapsedMilliseconds() const noexcept { return timer.elapsedMilliseconds(); }

    inline Profiler& getProfiler() noexcept {
        return profiler;
    }

private:
    inline void initialize(const Vertex source, const int departureTime, const Vertex target) noexcept {
        if (initialTransfers) {
            initialTransfers->run(source, target);

            // Add source with initial transfers
            for (const Vertex stop : initialTransfers->getForwardPOIs()) {
                const int dist = initialTransfers->getForwardDistance(stop);
                if (dist != INFTY) {
                    MCLabel label(departureTime + dist, dist, Vertex(source), 0, departureTime, -1);
                    addLabel(stop, label);
                }
            }

            // Direct path to target
            if (target != noVertex) {
                const int dist = initialTransfers->getDistance();
                if (dist != INFTY) {
                    MCLabel label(departureTime + dist, dist, Vertex(source), 0, departureTime, -1);
                    addLabel(target, label);
                }
            }
        } else {
            // Simple source initialization
            MCLabel sourceLabel(departureTime, 0, noVertex, -1, departureTime, -1);
            addLabel(source, sourceLabel);
        }
    }

    inline void addLabel(const Vertex v, const MCLabel& label) noexcept {
        // Target pruning - check if dominated by any label at target
        if constexpr (TARGET_PRUNING) {
            if (targetVertex != noVertex && nodeBags[targetVertex].dominates(label)) {
                profiler.countMetric(METRIC_PRUNED_LABELS);
                return;
            }
        }

        // Try to merge into bag - this handles dominance checking internally
        if (nodeBags[v].merge(label)) {
            touchedVertices.push_back(v);
            enqueueLabel(v, label);
            profiler.countMetric(METRIC_ENQUEUES);
        }
    }

    inline void enqueueLabel(const Vertex v, const MCLabel& label) noexcept {
        if (heapLabelCount >= heapLabels.size()) {
            heapLabels.resize(heapLabels.size() * 2);
        }

        HeapLabel& hl = heapLabels[heapLabelCount++];
        hl.vertex = v;
        hl.label = label;
        activeHeapLabels.push_back(&hl);
        std::push_heap(activeHeapLabels.begin(), activeHeapLabels.end(),
            [](const HeapLabel* a, const HeapLabel* b) { return b->hasSmallerKey(a); });
    }

    inline void runMainLoop() noexcept {
        while (!activeHeapLabels.empty()) {
            std::pop_heap(activeHeapLabels.begin(), activeHeapLabels.end(),
                [](const HeapLabel* a, const HeapLabel* b) { return b->hasSmallerKey(a); });
            HeapLabel* cur = activeHeapLabels.back();
            activeHeapLabels.pop_back();

            const Vertex u = cur->vertex;
            const MCLabel& curLabel = cur->label;
            const int t = curLabel.arrivalTime;

            settleCount++;
            profiler.countMetric(METRIC_SETTLES);

            // Target pruning - check if this label is dominated by target's Pareto front
            if constexpr (TARGET_PRUNING) {
                if (targetVertex != noVertex && nodeBags[targetVertex].dominates(curLabel)) {
                    profiler.countMetric(METRIC_PRUNED_LABELS);
                    continue;
                }
            }

            // Check if this label was superseded (dominated and removed from bag)
            // A label is stale if the bag no longer contains it
            bool labelStillValid = false;
            for (const MCLabel& l : nodeBags[u].labels) {
                if (l.arrivalTime == curLabel.arrivalTime &&
                    l.walkingDistance == curLabel.walkingDistance) {
                    labelStillValid = true;
                    break;
                }
            }
            if (!labelStillValid) {
                continue; // Label was dominated and removed, skip it
            }

            // CoreCH Backward (target shortcut)
            if (targetVertex != noVertex && initialTransfers && u < numberOfStops) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    MCLabel targetLabel(
                        t + backwardDist,
                        curLabel.walkingDistance + backwardDist,
                        u,
                        findLabelIndex(u, curLabel),
                        t,
                        -1  // walking
                    );
                    addLabel(targetVertex, targetLabel);
                }
            }

            // Scan outgoing edges
            for (const Edge e : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, e);

                // Transit edges (if this is a stop)
                if (u < numberOfStops) {
                    const auto& atf = graph.get(Function, e);

                    const DiscreteTrip* begin = graph.getTripsBegin(atf);
                    const DiscreteTrip* end = graph.getTripsEnd(atf);
                    const int* suffixBase = graph.getSuffixMinBegin(atf);

                    // Get minimum transfer time at destination for buffer consideration
                    const int bufferAtV = (v < numberOfStops) ? graph.getMinTransferTimeAt(v) : 0;

                    auto it = std::lower_bound(begin, end, t,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

                    // Track best local Pareto front for this edge (for dominance-based pruning)
                    // We track the best arrival time seen so far for the current walking distance
                    int bestLocalArrival = never;

                    for (; it != end; ++it) {
                        const size_t idx = std::distance(begin, it);
                        const int minPossibleArrival = suffixBase[idx];

                        // Multi-criteria dominance pruning (replaces single-criteria break)
                        // If the best possible arrival from remaining trips cannot improve
                        // on what we've already found locally, we can stop
                        if (bestLocalArrival != never) {
                            // Check if remaining trips are dominated by local best
                            // Since walking distance is constant (curLabel.walkingDistance),
                            // we only need to check arrival time with buffer
                            if (minPossibleArrival > bestLocalArrival + bufferAtV) {
                                profiler.countMetric(METRIC_PRUNED_LABELS);
                                break;
                            }
                        }

                        if constexpr (TARGET_PRUNING) {
                            if (targetVertex != noVertex) {
                                // Check if any path through this trip could be non-dominated at target
                                MCLabel potentialLabel(minPossibleArrival, curLabel.walkingDistance);
                                if (nodeBags[targetVertex].dominates(potentialLabel)) {
                                    profiler.countMetric(METRIC_PRUNED_LABELS);
                                    break;
                                }
                            }
                        }

                        // Update best local arrival
                        if (it->arrivalTime < bestLocalArrival) {
                            bestLocalArrival = it->arrivalTime;
                        }

                        // Scan this trip
                        scanTrip(it->tripId, it->departureStopIndex + 1, it->arrivalTime,
                                 u, curLabel, it->departureTime);
                    }
                }

                // Walking edges
                const int walkArrival = graph.getWalkArrivalFrom(e, t);
                if (walkArrival < never) {
                    const auto& atf = graph.get(Function, e);
                    const int walkTime = atf.walkTime;

                    MCLabel walkLabel(
                        walkArrival,
                        curLabel.walkingDistance + walkTime,
                        u,
                        findLabelIndex(u, curLabel),
                        t,
                        -1  // walking
                    );
                    addLabel(v, walkLabel);
                    relaxCount++;
                    profiler.countMetric(METRIC_RELAXES_WALKING);
                }
            }
        }
    }

    inline void scanTrip(const int tripId, const uint16_t startStopIndex, const int arrivalAtStart,
                         const Vertex boardStop, const MCLabel& boardLabel, const int departureTime) noexcept {
        int currentArrivalTime = arrivalAtStart;

        uint32_t currentAbsIndex = graph.getTripOffset(tripId) + startStopIndex;
        uint32_t endAbsIndex = graph.getTripOffset(tripId + 1);

        for (uint32_t idx = currentAbsIndex; idx < endAbsIndex; ++idx) {
            profiler.countMetric(METRIC_RELAXES_TRANSIT);

            // Target pruning
            if constexpr (TARGET_PRUNING) {
                if (targetVertex != noVertex) {
                    MCLabel potentialLabel(currentArrivalTime, boardLabel.walkingDistance);
                    if (nodeBags[targetVertex].dominates(potentialLabel)) {
                        return;
                    }
                }
            }

            // NOTE: In multi-criteria search, we cannot use single vehicle label dominance
            // because different scans may have different walking distances.
            // The addLabel function will handle dominance checking at the stop level.

            const auto& currentLeg = graph.getTripLeg(idx);
            Vertex currentStopVertex = currentLeg.stopId;

            // Create label for alighting at this stop
            MCLabel alightLabel(
                currentArrivalTime,
                boardLabel.walkingDistance,
                boardStop,
                findLabelIndex(boardStop, boardLabel),
                departureTime,
                tripId
            );
            addLabel(currentStopVertex, alightLabel);

            // Get next arrival time
            if (idx + 1 < endAbsIndex) {
                const auto& nextLeg = graph.getTripLeg(idx + 1);
                currentArrivalTime = nextLeg.arrivalTime;
            }
        }
    }

    inline size_t findLabelIndex(const Vertex v, const MCLabel& label) const noexcept {
        const LabelBag& bag = nodeBags[v];
        for (size_t i = 0; i < bag.size(); ++i) {
            if (bag[i].arrivalTime == label.arrivalTime &&
                bag[i].walkingDistance == label.walkingDistance) {
                return i;
            }
        }
        return 0; // fallback
    }

private:
    const Graph& graph;
    const size_t numberOfStops;

    std::vector<LabelBag> nodeBags;
    std::vector<VehicleLabel> globalVehicleLabels;

    std::vector<HeapLabel> heapLabels;
    size_t heapLabelCount;
    std::vector<HeapLabel*> activeHeapLabels;

    std::vector<Vertex> touchedVertices;

    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;

    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
    Vertex sourceVertex;
    int sourceDepartureTime;

    Profiler profiler;
};

} // namespace TDD