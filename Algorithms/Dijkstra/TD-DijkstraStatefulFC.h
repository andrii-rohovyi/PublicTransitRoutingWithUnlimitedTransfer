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
#include "../../DataStructures/Graph/TimeDependentGraphFC.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "Profiler.h"

template<typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStatefulFC {
public:
    using Graph = TimeDependentGraphFC;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;
    using CoreCHInitialTransfers = CH::Query<CHGraph, true, false, true>;

    struct NodeLabel : public ExternalKHeapElement {
        NodeLabel() : ExternalKHeapElement(), arrivalTime(intMax), timeStamp(-1),
                      parent(noVertex) {}

        inline void reset(int ts) {
            arrivalTime = intMax;
            timeStamp = ts;
            parent = noVertex;
        }

        inline bool hasSmallerKey(const NodeLabel* other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }

        int arrivalTime;
        int timeStamp;
        Vertex parent;
    };

public:
    TimeDependentDijkstraStatefulFC(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices())
        , nodeLabels(g.numVertices())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex)
        , fcSearchCount(0)
        , regularSearchCount(0) {
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
        fcSearchCount = 0;
        regularSearchCount = 0;
        timer.restart();
        targetVertex = noVertex;
        profiler.donePhase(TDD::PHASE_CLEAR);
    }

    inline void addSource(const Vertex s, const int time) noexcept {
        NodeLabel& L = getNodeLabel(s);
        if (time < L.arrivalTime) {
            L.arrivalTime = time;
            L.parent = noVertex;
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
        int arrivalTime;
    };

    inline std::vector<PathEntry> getPath(const Vertex target) const noexcept {
        std::vector<PathEntry> path;
        if (!reachable(target)) return path;

        Vertex curVertex = target;

        while (curVertex != noVertex) {
            const NodeLabel& L = nodeLabels[curVertex];
            path.push_back({curVertex, L.arrivalTime});
            curVertex = L.parent;
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    inline void printFCStatistics() const noexcept {
        std::cout << "FC searches: " << fcSearchCount << std::endl;
        std::cout << "Regular searches: " << regularSearchCount << std::endl;
        if (fcSearchCount + regularSearchCount > 0) {
            double fcPercent = 100.0 * fcSearchCount / (fcSearchCount + regularSearchCount);
            std::cout << "FC usage: " << fcPercent << "%" << std::endl;
        }
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

            // 1. CoreCH Backward (if enabled)
            if (targetVertex != noVertex && initialTransfers && u < numberOfStops) {
                const int backwardDist = initialTransfers->getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxEdge(targetVertex, u, arrivalAtTarget);
                }
            }

            // 2. Scan edges - use FC if available
            if (graph.hasFCData(u)) {
                relaxEdgesWithFC(u, t);
            } else {
                relaxEdgesRegular(u, t);
            }
        }
        profiler.donePhase(TDD::PHASE_MAIN_LOOP);
    }

    // [FRACTIONAL CASCADING] Optimized edge relaxation
    // Based on Python: get_positions_fractional_cascading + Dijkstra loop
    inline void relaxEdgesWithFC(const Vertex u, const int departureTime) noexcept {
        fcSearchCount++;

        const FractionalCascadingData& fc = graph.getFCData(u);

        if (fc.m_arr.empty() || fc.reachableEdges.empty()) return;

        const size_t numLevels = fc.m_arr.size();

        // ONE binary search on the first cascaded array
        int loc = fc.bisectLeft(departureTime);

        // Bounds check for first level
        if (loc >= (int)fc.pointers[0].size()) {
            loc = (int)fc.pointers[0].size() - 1;
        }
        if (loc < 0) loc = 0;

        // Get pointer for level 0
        const FCPointer& ptr0 = fc.pointers[0][loc];

        // Relax edge at level 0
        relaxEdgeWithStartIndex(fc.reachableEdges[0], fc.reachableNodes[0], u, departureTime, ptr0.startIndex);

        int nextLoc = ptr0.nextLoc;

        // Cascade through remaining levels - O(1) per level
        for (size_t i = 1; i < numLevels; ++i) {
            // Bounds check
            if (nextLoc >= (int)fc.m_arr[i].size()) {
                nextLoc = (int)fc.m_arr[i].size() - 1;
            }
            if (nextLoc < 0) nextLoc = 0;

            // Python logic: choose pointer based on comparison
            // if winner_weight <= m_arr[i][next_loc - 1]: use next_loc - 1
            // else: use next_loc
            int ptrIndex;
            if (nextLoc > 0 && departureTime <= fc.m_arr[i][nextLoc - 1]) {
                ptrIndex = nextLoc - 1;
            } else {
                ptrIndex = nextLoc;
            }

            // Bounds check for pointer index
            if (ptrIndex >= (int)fc.pointers[i].size()) {
                ptrIndex = (int)fc.pointers[i].size() - 1;
            }
            if (ptrIndex < 0) ptrIndex = 0;

            const FCPointer& ptr_i = fc.pointers[i][ptrIndex];

            // Relax edge at level i
            relaxEdgeWithStartIndex(fc.reachableEdges[i], fc.reachableNodes[i], u, departureTime, ptr_i.startIndex);

            nextLoc = ptr_i.nextLoc;
        }

        // Process walking-only edges (no FC needed, just walk time)
        for (size_t i = 0; i < fc.walkingEdges.size(); ++i) {
            const Edge e = fc.walkingEdges[i];
            const Vertex v = fc.walkingNodes[i];

            const int arrivalAtV = graph.getWalkArrivalFrom(e, departureTime);
            if (arrivalAtV < never) {
                relaxEdge(v, u, arrivalAtV);
            }
        }
    }

    // Relax edge using the pre-computed startIndex from FC
    // startIndex points directly into the edge's trip array
    inline void relaxEdgeWithStartIndex(const Edge e, const Vertex v, const Vertex parent,
                                         const int departureTime, const int startIndex) noexcept {
        const EdgeTripsHandle& h = graph.get(Function, e);

        int bestArrival = never;

        // Use suffix minimum for O(1) best arrival lookup
        if (startIndex >= 0 && (uint32_t)startIndex < h.tripCount) {
            const int* suffixMin = graph.getSuffixMinBegin(h);
            bestArrival = suffixMin[startIndex];
        }

        // Check walking option
        if (h.walkTime != never) {
            int walkArrival = departureTime + h.walkTime;
            bestArrival = std::min(bestArrival, walkArrival);
        }

        if (bestArrival < never) {
            relaxEdge(v, parent, bestArrival);
        }
    }

    // Standard edge relaxation (when no FC data available)
    inline void relaxEdgesRegular(const Vertex u, const int departureTime) noexcept {
        regularSearchCount++;

        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            const int arrivalAtV = graph.getWalkArrivalFrom(e, departureTime);

            if (arrivalAtV < never) {
                relaxEdge(v, u, arrivalAtV);
            }
        }
    }

    inline void relaxEdge(const Vertex v, const Vertex parent, const int newTime) noexcept {
        relaxCount++;
        profiler.countMetric(TDD::METRIC_RELAXES_WALKING);

        NodeLabel& L = getNodeLabel(v);
        if (L.arrivalTime > newTime) {
            L.arrivalTime = newTime;
            L.parent = parent;
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

    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    std::unique_ptr<CoreCHInitialTransfers> initialTransfers;
    Vertex targetVertex;
    Profiler profiler;

    size_t fcSearchCount;
    size_t regularSearchCount;
};