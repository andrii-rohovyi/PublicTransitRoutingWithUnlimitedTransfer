// Prevent multiple inclusions of the same header file
#pragma once

// --- STANDARD INCLUDES ---
#include <iostream>
#include <vector>
#include <string>
#include <set>

// --- PROJECT INCLUDES ---
#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/IndexedSet.h"
#include "../../DataStructures/Attributes/AttributeNames.h"


/**
 * @brief A highly generic and efficient implementation of Dijkstra's algorithm.
 * 
 * @details This class implements Dijkstra's algorithm for finding the shortest paths
 * from a source vertex to all other vertices in a weighted graph with non-negative 
 * edge weights. It works with any graph type with a basic interface. This 
 * implementation uses a timestamp mechanism for lazy initialization, avoiding the need
 * to reset all vertex labels between runs. 
 * 
 * The algorithm's behavior can be customized by providing callback functions (functors)
 * for settling vertices, pruning edges, or stopping the search early.
 * 
 * @tparam GRAPH The type of the graph data structure. Must provide methods like
 * `numVertices()`, `edgesFrom(Vertex)`, and `get(ToVertex, Edge)`.
 * @tparam DEBUG A compile-time boolean flag. If true, the class will collect and print
 * performance metrics like execution time and the number of settled vertices. Defaults
 * to false.
*/
template<typename GRAPH, bool DEBUG = false>
class Dijkstra {

public:
    using Graph = GRAPH;

    // Create a class member variable to hold the debug flag
    static constexpr bool Debug = DEBUG;
    using Type = Dijkstra<Graph, Debug>;

/** 
 * @brief A label for each vertex in the graph, used in the priority queue.
 * @details This struct extends the ExternalKHeapElement so that it can be stored
 * in the ExternalKHeap priority queue. It contains the current shortest distance
 * from the source vertex, the parent vertex in the shortest path tree, and a
 * timestamp for lazy clearing.
 */
public:
    struct VertexLabel : public ExternalKHeapElement {
        // 
        VertexLabel() : ExternalKHeapElement(), distance(intMax), parent(noVertex), timeStamp(-1) {}
        inline void reset(int time) {
            distance = intMax;
            parent = noVertex;
            timeStamp = time;
        }
        inline bool hasSmallerKey(const VertexLabel* other) const noexcept {
            return distance < other->distance;
        }

        int distance;
        Vertex parent;
        int timeStamp;
    };

/**
 * @brief Primary constructor for the Dijkstra algorithm class.
 * @details This constructor initializes all necessary data structures for the algorithm.
 * It stores references to the graph and its weights and pre-allocates
 * memory for the priority queue and vertex labels for efficiency.
 * * @param graph A constant reference to the graph object. The algorithm will run on this graph.
 * It's a reference to avoid making an expensive copy.
 * @param weight A constant reference to a vector of edge weights. It's assumed that the
 * weight for the edge with ID 'i' is located at weight[i].
 */
public:
    Dijkstra(const GRAPH& graph, const std::vector<int>& weight) :
// Store references to the user-provided graph and weight data.
        graph(graph),
        weight(weight),
        
        // Pre-allocate memory for the priority queue to hold all vertices.
        Q(graph.numVertices()),
        
        // Pre-allocate memory for the label of each vertex.
        label(graph.numVertices()),
        
        // Initialize the timestamp for the lazy-clearing mechanism.
        timeStamp(0),
        
        // Initialize the debug counter for settled vertices.
        settleCount(0) {
        // The constructor body is empty because all initialization is done
        // in the member initializer list above, which is more efficient.
    }

    Dijkstra(const GRAPH& graph) :
        Dijkstra(graph, graph[TravelTime]){
    }

    template<AttributeNameType ATTRIBUTE_NAME>
    Dijkstra(const GRAPH& graph, const AttributeNameWrapper<ATTRIBUTE_NAME> weight) :
        Dijkstra(graph, graph[weight]){
    }

    Dijkstra(const GRAPH&&, const std::vector<int>&) = delete;
    Dijkstra(const GRAPH&, const std::vector<int>&&) = delete;
    Dijkstra(const GRAPH&&) = delete;

    template<AttributeNameType ATTRIBUTE_NAME>
    Dijkstra(const GRAPH&&, const AttributeNameWrapper<ATTRIBUTE_NAME>) = delete;

    template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, const Vertex target = noVertex, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(source);
        run(target, settle, stop, pruneEdge);
    }

    template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, IndexedSet<false, Vertex>& targets, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(source);
        run(noVertex, [&](const Vertex u) {
            settle(u);
            targets.remove(u);
        }, [&]() {
            return stop() || targets.empty();
        }, pruneEdge);
    }

    template<typename SOURCE_CONTAINER, typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION, typename = decltype(std::declval<SOURCE_CONTAINER>().begin())>
    inline void run(const SOURCE_CONTAINER& sources, const Vertex target = noVertex, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        for (const Vertex source : sources) {
            addSource(source);
        }
        run(target, settle, stop, pruneEdge);
    }

    inline void clear() noexcept {
        if constexpr (Debug) {
            timer.restart();
            settleCount = 0;
        }
        Q.clear();
        timeStamp++;
    }

    inline void addSource(const Vertex source, const int distance = 0) noexcept {
        VertexLabel& sourceLabel = getLabel(source);
        sourceLabel.distance = distance;
        Q.update(&sourceLabel);
    }

    inline void run() noexcept {
        run(noVertex, NoOperation, NoOperation, NoOperation);
    }

    template<typename SETTLE, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION, typename = decltype(std::declval<SETTLE>()(std::declval<Vertex>()))>
    inline void run(const Vertex target, const SETTLE& settle, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        while(!Q.empty()) {
            if (stop()) break;
            const VertexLabel* uLabel = Q.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (u == target) break;
            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, edge);
                VertexLabel& vLabel = getLabel(v);
                if (pruneEdge(u, edge)) continue;
                const int distance = uLabel->distance + weight[edge];
                if (vLabel.distance > distance) {
                    vLabel.distance = distance;
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

    inline bool reachable(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline bool visited(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline int getDistance(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].distance;
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
    const GRAPH& graph;
    const std::vector<int>& weight;

    ExternalKHeap<2, VertexLabel> Q;

    std::vector<VertexLabel> label;
    int timeStamp;

    int settleCount;
    Timer timer;

};
