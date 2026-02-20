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
#include "../../DataStructures/Graph/TimeDependentGraphBST.h"
#include "../../Algorithms/CH/CH.h"
#include "../../Algorithms/CH/Query/BucketQuery.h"
#include "Profiler.h"

template<typename GRAPH_TYPE, typename PROFILER = TDD::NoProfiler, bool DEBUG = false, bool TARGET_PRUNING = true>
class TimeDependentDijkstraStatefulBSTBucketCH {
public:
    using Graph = GRAPH_TYPE;
    using Profiler = PROFILER;
    static constexpr bool Debug = DEBUG;
    using BucketCHQuery = CH::BucketQuery<CHGraph, true, false>;

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
    TimeDependentDijkstraStatefulBSTBucketCH(const Graph& g, const size_t numStops, const CH::CH* chData)
        : graph(g)
        , numberOfStops(numStops)
        , Q(g.numVertices())
        , nodeLabels(g.numVertices())
        , timeStamp(0)
        , settleCount(0)
        , relaxCount(0)
        , targetVertex(noVertex)
        , bstSearchCount(0)
        , regularSearchCount(0)
        , bucketQuery(*chData, FORWARD, numberOfStops) {
        }

    inline void clear() noexcept {
        profiler.startPhase(TDD::PHASE_CLEAR);
        Q.clear();
        timeStamp++;
        settleCount = 0;
        relaxCount = 0;
        bstSearchCount = 0;
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
        bucketQuery.run(source, target);
        for (const Vertex stop : bucketQuery.getForwardPOIs()) {
            const int arrivalTime = departureTime + bucketQuery.getForwardDistance(stop);
            addSource(stop, arrivalTime);
        }
        if (target != noVertex) {
            const int dist = bucketQuery.getDistance();
            if (dist != INFTY) {
                addSource(target, departureTime + dist);
            }
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

            // 1. Bucket-CH Backward
            if (targetVertex != noVertex && u < numberOfStops) {
                const int backwardDist = bucketQuery.getBackwardDistance(u);
                if (backwardDist != INFTY) {
                    const int arrivalAtTarget = t + backwardDist;
                    relaxEdge(targetVertex, u, arrivalAtTarget);
                }
            }

            // 2. Scan edges - use BST if available
            if (graph.hasBSTData(u)) {
                relaxEdgesWithBST(u, t);
            } else {
                relaxEdgesRegular(u, t);
            }
        }
        profiler.donePhase(TDD::PHASE_MAIN_LOOP);
    }

    inline void relaxEdgesWithBST(const Vertex u, const int departureTime) noexcept {
        bstSearchCount++;

        const BalancedSearchTreeData& bst = graph.getBSTData(u);

        if (bst.empty()) return;

        auto it = bst.bisectLeft(departureTime);

        if (it != bst.tree.end()) {
            const std::vector<int>& startIndices = it->second;

            for (size_t edgeIdx = 0; edgeIdx < bst.edges.size(); ++edgeIdx) {
                const Edge e = bst.edges[edgeIdx];
                const Vertex v = bst.targets[edgeIdx];
                const int startIndex = startIndices[edgeIdx];

                relaxEdgeWithStartIndex(e, v, u, departureTime, startIndex);
            }
        } else {
            for (size_t edgeIdx = 0; edgeIdx < bst.edges.size(); ++edgeIdx) {
                const Edge e = bst.edges[edgeIdx];
                const Vertex v = bst.targets[edgeIdx];
                const EdgeTripsHandle& h = graph.get(Function, e);
                if (h.walkTime != never) {
                    int walkArrival = departureTime + h.walkTime;
                    relaxEdge(v, u, walkArrival);
                }
            }
        }

        for (size_t i = 0; i < bst.walkingEdges.size(); ++i) {
            const Edge e = bst.walkingEdges[i];
            const Vertex v = bst.walkingTargets[i];

            const int arrivalAtV = graph.getWalkArrivalFrom(e, departureTime);
            if (arrivalAtV < never) {
                relaxEdge(v, u, arrivalAtV);
            }
        }
    }

    inline void relaxEdgeWithStartIndex(const Edge e, const Vertex v, const Vertex parent,
                                         const int departureTime, const int startIndex) noexcept {
        const EdgeTripsHandle& h = graph.get(Function, e);

        int bestArrival = never;

        if (startIndex >= 0 && (uint32_t)startIndex < h.tripCount) {
            const int* suffixMin = graph.getSuffixMinBegin(h);
            bestArrival = suffixMin[startIndex];
        }

        if (h.walkTime != never) {
            int walkArrival = departureTime + h.walkTime;
            bestArrival = std::min(bestArrival, walkArrival);
        }

        if (bestArrival < never) {
            relaxEdge(v, parent, bestArrival);
        }
    }

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

private:
    const Graph& graph;
    const size_t numberOfStops;
    ExternalKHeap<2, NodeLabel> Q;
    std::vector<NodeLabel> nodeLabels;
    int timeStamp;
    int settleCount;
    int relaxCount;
    Timer timer;
    Vertex targetVertex;
    Profiler profiler;
    size_t bstSearchCount;
    size_t regularSearchCount;
    BucketCHQuery bucketQuery;
};