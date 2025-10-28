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
        settleCount(0),
        relaxCount(0) {
    }

    // --- Deleted Constructors from static version are kept for safety ---
    TimeDependentDijkstra(const GRAPH&&) = delete;


    // --- Public run methods ---

    /**
     * @brief Main entry point to run the algorithm.
     * @param source The starting vertex.
     * @param departureTime The time at which the journey begins.
     * @param target The optional target vertex (default: noVertex for one-to-all).
     */
    template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();
        addSource(source, departureTime);
        runRelaxation(target, settle, stop, pruneEdge);
    }

    /**
     * @brief Batch query helper - runs multiple queries efficiently.
     * @param queries Container of VertexQuery objects (must have source, departureTime, target fields)
     * @return Vector of arrival times for each query
     */
    template<typename QUERY_CONTAINER>
    inline std::vector<int> runQueries(const QUERY_CONTAINER& queries) noexcept {
        std::vector<int> results;
        results.reserve(queries.size());
        
        for (const auto& query : queries) {
            run(query.source, query.departureTime, query.target);
            results.push_back(getArrivalTime(query.target));
        }
        
        return results;
    }


    inline void clear() noexcept {
        Q.clear();
        timeStamp++;
        settleCount = 0;
        relaxCount = 0;
        if constexpr (Debug) {
            timer.restart();
        }
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
        if constexpr (Debug) {
            std::cout << "Starting TD-Dijkstra relaxation..." << std::endl;
            std::cout << "  Target: " << (target == noVertex ? -1 : target) << std::endl;
        }
        
        while(!Q.empty()) {
            if (stop()) {
                if constexpr (Debug) std::cout << "  Stopped by stop condition" << std::endl;
                break;
            }
            
            const VertexLabel* uLabel = Q.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            
            settleCount++;
            
            if constexpr (Debug) {
                std::cout << "  Settling vertex " << u 
                          << " at time " << uLabel->arrivalTime 
                          << " (settle #" << settleCount << ")" << std::endl;
                settle(u);
            }
            
            if (u == target) {
                if constexpr (Debug) std::cout << "  Reached target!" << std::endl;
                break;
            }

            // The arrival time at u is the departure time for all outgoing edges.
            const int departureTime = uLabel->arrivalTime;

            for (const Edge edge : graph.edgesFrom(u)) {
                if (pruneEdge(u, edge)) continue;
                
                relaxCount++;  // Count every relaxation attempt
                
                const Vertex v = graph.get(ToVertex, edge);
                VertexLabel& vLabel = getLabel(v);

                // CORE CHANGE: Use the graph's time-dependent query
                const int newArrivalTime = graph.getArrivalTime(edge, departureTime);

                // Check for unreachable connections
                if (newArrivalTime == intMax) continue;

                // Standard relaxation check
                if (vLabel.arrivalTime > newArrivalTime) {
                    if constexpr (Debug) {
                        std::cout << "    Relaxing edge " << u << " -> " << v 
                                  << ": " << vLabel.arrivalTime << " -> " << newArrivalTime << std::endl;
                    }
                    vLabel.arrivalTime = newArrivalTime;
                    vLabel.parent = u;
                    Q.update(&vLabel);
                }
            }
        }
        
        if constexpr (Debug) {
            std::cout << "TD-Dijkstra completed:" << std::endl;
            std::cout << "  Settled Vertices = " << String::prettyInt(settleCount) << std::endl;
            std::cout << "  Relaxed Edges = " << String::prettyInt(relaxCount) << std::endl;
            std::cout << "  Time = " << String::msToString(timer.elapsedMilliseconds()) << std::endl;
        }
    }

    // --- Accessor methods ---
    // (Most remain the same, but 'getDistance' is renamed for clarity)

    inline bool visited(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        return visited(vertex);
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].arrivalTime;
        return intMax;  // Return intMax instead of -1 for unreachable vertices (more consistent)
    }

    // Alias methods for compatibility with different calling conventions
    inline int getArrivalTime(const Vertex vertex) const noexcept {
        return getEarliestArrivalTime(vertex);
    }

    inline int getDistance(const Vertex vertex) const noexcept {
        return getEarliestArrivalTime(vertex);
    }

    inline Vertex getParent(const Vertex vertex) const noexcept {
        if (visited(vertex)) return label[vertex].parent;
        return noVertex;
    }

    inline std::set<Vertex> getChildren(const Vertex vertex) const noexcept {
        if (visited(vertex)) {
            std::set<Vertex> children;
            for (const Edge edge : graph.edgesFrom(vertex)) {
                const Vertex child = graph.get(ToVertex, edge);
                if (visited(child) && label[child].parent == vertex) {
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

    inline int getRelaxCount() const noexcept {
        return relaxCount;
    }

    inline double getElapsedMilliseconds() const noexcept {
        return timer.elapsedMilliseconds();
    }

    // --- Validation and utility methods ---

    /**
     * @brief Check if a vertex has a valid result (visited and arrival time is not never).
     * @param vertex The vertex to check
     * @return true if vertex was reached with a finite arrival time
     */
    inline bool hasValidResult(const Vertex vertex) const noexcept {
        return visited(vertex) && label[vertex].arrivalTime != never;
    }

    /**
     * @brief Count the number of reachable vertices from the last query.
     * @return Number of vertices that were visited
     */
    inline size_t numReachableVertices() const noexcept {
        size_t count = 0;
        for (const auto& l : label) {
            if (l.timeStamp == timeStamp && l.arrivalTime != never) {
                count++;
            }
        }
        return count;
    }

    /**
     * @brief Get all reachable vertices from the last query.
     * @return Vector of vertices that were reached
     */
    inline std::vector<Vertex> getReachableVertices() const noexcept {
        std::vector<Vertex> reachable;
        for (size_t i = 0; i < label.size(); ++i) {
            if (label[i].timeStamp == timeStamp && label[i].arrivalTime != never) {
                reachable.push_back(Vertex(i));
            }
        }
        return reachable;
    }

    /**
     * @brief Validate that the path is consistent (each vertex's parent leads to it).
     * @param vertex The vertex to validate the path to
     * @return true if path is valid, false otherwise
     */
    inline bool validatePath(const Vertex vertex) const noexcept {
        if (!visited(vertex)) return false;
        
        Vertex current = vertex;
        std::set<Vertex> seen;
        
        while (label[current].parent != noVertex) {
            // Check for cycles
            if (seen.count(current) > 0) return false;
            seen.insert(current);
            
            Vertex parent = label[current].parent;
            
            // Check parent is visited
            if (!visited(parent)) return false;
            
            // Check parent has earlier arrival time
            if (label[parent].arrivalTime > label[current].arrivalTime) return false;
            
            current = parent;
        }
        
        return true;
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
    int relaxCount;
    Timer timer;

};