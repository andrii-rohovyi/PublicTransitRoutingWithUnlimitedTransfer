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
#include "../../DataStructures/Graph/TimeDependentGraphCST.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/CHQuery.h"
#include "Profiler.h"

template<typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStatefulCST {
public:
    using Graph = TimeDependentGraphCST;
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
    TimeDependentDijkstraStatefulCST(const Graph& g, const size_t numStops = 0, const CH::CH* chData = nullptr)
        : graph(g)
        , numberOfStops(numStops == 0 ? g.numVertices() : numStops)
        , Q(g.numVertices())
        , nodeLabels(g.numVertices())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex)
        , cstSearchCount(0)
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
        cstSearchCount = 0;
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

    inline void printCSTStatistics() const noexcept {
        std::cout << "CST searches: " << cstSearchCount << std::endl;
        std::cout << "Regular searches: " << regularSearchCount << std::endl;
        if (cstSearchCount + regularSearchCount > 0) {
            double cstPercent = 100.0 * cstSearchCount / (cstSearchCount + regularSearchCount);
            std::cout << "CST usage: " << cstPercent << "%" << std::endl;
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

            // 2. Scan edges - use CST if available
            if (graph.hasCSTData(u)) {
                relaxEdgesWithCST(u, t);
            } else {
                relaxEdgesRegular(u, t);
            }
        }
        profiler.donePhase(TDD::PHASE_MAIN_LOOP);
    }

    // [COMBINED SEARCH TREE] Optimized edge relaxation
    // ONE binary search on schedule, then O(1) lookup per edge
    inline void relaxEdgesWithCST(const Vertex u, const int departureTime) noexcept {
        cstSearchCount++;

        const CombinedSearchTreeData& cst = graph.getCSTData(u);

        if (cst.schedule.empty()) return;

        // ONE binary search to find position in schedule
        int schedIdx = cst.bisectLeft(departureTime);

        // Process all edges with trips
        if (schedIdx >= 0 && (size_t)schedIdx < cst.positionInEdge.size()) {
            const auto& startIndices = cst.positionInEdge[schedIdx];

            for (size_t edgeIdx = 0; edgeIdx < cst.edges.size(); ++edgeIdx) {
                const Edge e = cst.edges[edgeIdx];
                const Vertex v = cst.targets[edgeIdx];
                const int startIndex = startIndices[edgeIdx];

                relaxEdgeWithStartIndex(e, v, u, departureTime, startIndex);
            }
        } else {
            // No valid schedule entry, but still check walking edges
            // For edges with trips, if schedIdx == -1, no valid bus departure
            for (size_t edgeIdx = 0; edgeIdx < cst.edges.size(); ++edgeIdx) {
                const Edge e = cst.edges[edgeIdx];
                const Vertex v = cst.targets[edgeIdx];
                // Only walk time (startIndex beyond all trips)
                const EdgeTripsHandle& h = graph.get(Function, e);
                if (h.walkTime != never) {
                    int walkArrival = departureTime + h.walkTime;
                    relaxEdge(v, u, walkArrival);
                }
            }
        }

        // Process walking-only edges
        for (size_t i = 0; i < cst.walkingEdges.size(); ++i) {
            const Edge e = cst.walkingEdges[i];
            const Vertex v = cst.walkingTargets[i];

            const int arrivalAtV = graph.getWalkArrivalFrom(e, departureTime);
            if (arrivalAtV < never) {
                relaxEdge(v, u, arrivalAtV);
            }
        }
    }

    // Relax edge using the pre-computed startIndex from CST
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

    // Standard edge relaxation (when no CST data available)
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

    size_t cstSearchCount;
    size_t regularSearchCount;
};