#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <set>

#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Attributes/AttributeNames.h"

/**
 * @brief An implementation of a time-dependent Dijkstra's algorithm.
 *
 * @details This class finds the earliest arrival time from a source vertex
 * given a specific departure time. It is adapted from a static Dijkstra's
 * implementation and uses the project's existing TimeDependentGraph structure.
 *
 * @tparam GRAPH The type of the graph, expected to be a TimeDependentGraph.
 * @tparam DEBUG A compile-time boolean flag for performance metrics.
*/
template<typename GRAPH, bool DEBUG = false>
class TimeDependentDijkstra {

public:
    using Graph = GRAPH;
    static constexpr bool Debug = DEBUG;
    using Type = TimeDependentDijkstra<Graph, Debug>;

public:
    /**
     * @brief A label for each vertex, tracking arrival time and parent.
     * @details The 'distance' field from the static version has been renamed to
     * 'arrivalTime' to better reflect its purpose in a time-dependent context.
     * The priority queue will use this value to find the next vertex to settle.
     */
    struct VertexLabel : public ExternalKHeapElement {
        VertexLabel() : ExternalKHeapElement(), arrivalTime(intMax), parent(noVertex), timeStamp(-1) {}

        inline void reset(int time) {
            arrivalTime = intMax;
            parent = noVertex;
            timeStamp = time;
        }

        // The comparison function prioritizes the earliest arrival time
        inline bool hasSmallerKey(const VertexLabel* other) const noexcept {
            return arrivalTime < other->arrivalTime;
        }

        int arrivalTime;
        Vertex parent;
        int timeStamp;
    };

public:
    /**
     * @brief Main constructor for the TimeDependentDijkstra class.
     * @details This constructor no longer takes a static weight vector. Instead, it
     * directly takes a reference to the time-dependent graph, which contains the
     * arrival time functions for each edge.
     * @param graph A constant reference to the TimeDependentGraph object.
     */
    TimeDependentDijkstra(const GRAPH& graph) :
        graph(graph),
        Q(graph.numVertices()),
        label(graph.numVertices()),
        timeStamp(0),
        settleCount(0) {
    }

    // --- Deleted Constructors from static version are kept for safety ---
    TimeDependentDijkstra(const GRAPH&&) = delete;


    // --- Public run methods ---

    /**
     * @brief Main entry point to run the algorithm.
     * @param source The starting vertex.
     * @param departureTime The time at which the journey begins.
     * @param target The optional target vertex.
     */
    template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(source, departureTime);
        runRelaxation(target, settle, stop, pruneEdge);
    }


    inline void clear() noexcept {
        if constexpr (Debug) {
            timer.restart();
            settleCount = 0;
        }
        Q.clear();
        timeStamp++;
    }

    /**
     * @brief Initializes the source vertex with a specific departure time.
     * @param source The source vertex.
     * @param time The departure time, which is the initial arrival time at the source.
     */
    inline void addSource(const Vertex source, const int time) noexcept {
        VertexLabel& sourceLabel = getLabel(source);
        sourceLabel.arrivalTime = time;
        Q.update(&sourceLabel);
    }


    // --- Core TD-Dijkstra Relaxation Logic ---

    /**
     * @brief The main algorithm loop where relaxation occurs.
     * @details This is the workhorse of the algorithm. The key change is that it
     * no longer adds a static weight. Instead, it uses the arrival time at the
     * current vertex 'u' as the departure time to query the edge's arrival time function.
     */
    template<typename SETTLE, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION, typename = decltype(std::declval<SETTLE>()(std::declval<Vertex>()))>
    inline void runRelaxation(const Vertex target, const SETTLE& settle, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        while(!Q.empty()) {
            if (stop()) break;
            const VertexLabel* uLabel = Q.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (u == target) break;

            // The arrival time at u is the departure time for all outgoing edges.
            const int departureTime = uLabel->arrivalTime;

            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, edge);
                VertexLabel& vLabel = getLabel(v);
                if (pruneEdge(u, edge)) continue;

                // CORE CHANGE: Use the graph's time-dependent query
                const int newArrivalTime = graph.getArrivalTime(edge, departureTime);

                // Check for unreachable connections
                if (newArrivalTime == intMax) continue;

                // Standard relaxation check
                if (vLabel.arrivalTime > newArrivalTime) {
                    vLabel.arrivalTime = newArrivalTime;
                    vLabel.parent = u;
                    Q.update(&vLabel);
                }
            }
            settle(u);
            if constexpr (Debug) settleCount++;
        }
        if constexpr (Debug) {
            std::cout << "Settled Vertices = " << String::prettyInt(settleCount) << std::endl;
            std::cout << "Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
        }
    }

    // --- Accessor methods ---
    // (Most remain the same, but 'getDistance' is renamed for clarity)

    inline bool visited(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].arrivalTime;
        return -1;
    }

    inline Vertex getParent(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].parent;
        return noVertex;
    }

    inline std::set<Vertex> getChildren(const Vertex vertex) const noexcept {
        if (visited(vertex)) {
            std::set<Vertex> children;
            for (Vertex child : graph.outgoingNeighbors(vertex)) {
                if (label[child].parent == vertex) {
                    children.insert(child);
                }
            }
            return children;
        }
        return std::set<Vertex>();
    }

    inline Vertex getQFront() const noexcept {
        if (Q.empty()) return noVertex;
        return Vertex(Q.front() - &(label[0]));
    }

    inline std::vector<Vertex> getReversePath(const Vertex to) const noexcept {
        std::vector<Vertex> path;
        if (!visited(to)) return path;
        path.push_back(to);
        while (label[path.back()].parent != noVertex) {
            path.push_back(label[path.back()].parent);
        }
        return path;
    }

    inline std::vector<Vertex> getPath(const Vertex to) const noexcept {
        return Vector::reverse(getReversePath(to));
    }

    inline int getSettleCount() const noexcept {
        return settleCount;
    }

private:
    inline VertexLabel& getLabel(const Vertex vertex) noexcept {
        VertexLabel& result = label[vertex];
        if (result.timeStamp != timeStamp) result.reset(timeStamp);
        return result;
    }

private:
    const GRAPH& graph; // Now expects a TimeDependentGraph

    ExternalKHeap<2, VertexLabel> Q;

    std::vector<VertexLabel> label;
    int timeStamp;

    int settleCount;
    Timer timer;

};