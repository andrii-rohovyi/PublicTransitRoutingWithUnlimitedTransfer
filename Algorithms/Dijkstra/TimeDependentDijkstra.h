#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <algorithm>
#include <concepts>

#include "../../Helpers/Types.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Vector.h"

#include "../../DataStructures/Container/ExternalKHeap.h"
#include "../../DataStructures/Container/IndexedSet.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"

// Note: Ensure the necessary header for INITIAL_TRANSFERS (e.g., InitialTransfers.h) is included

// New template: added INITIAL_TRANSFERS type
template<typename GRAPH, typename INITIAL_TRANSFERS, bool DEBUG = false>
class CHTimeDependentDijkstra {

public:
    using Graph = GRAPH;
    using InitialTransferType = INITIAL_TRANSFERS;
    static constexpr bool Debug = DEBUG;
    // Renamed Type alias for the combined class
    using Type = CHTimeDependentDijkstra<Graph, InitialTransferType, Debug>;

public:
    // VertexLabel structure remains the same
    struct VertexLabel : public ExternalKHeapElement {
        VertexLabel() : ExternalKHeapElement(), distance(intMax), parent(noVertex), timeStamp(-1) {}
        inline void reset(int time) {
            distance = intMax;
            parent = noVertex;
            timeStamp = time;
        }
        inline bool hasSmallerKey(const VertexLabel* other) const noexcept {
            return distance < other->distance;
        }

        int distance; // Stores the earliest arrival time (EAT)
        Vertex parent;
        int timeStamp;
    };

public:
    // 1. Core Constructor (takes both the TD graph and the CH structure)
    CHTimeDependentDijkstra(const GRAPH& graph, const INITIAL_TRANSFERS& initialTransfers) :
        graph(graph),
        initialTransfers(initialTransfers), // New member initialization
        Q(graph.numVertices()),
        label(graph.numVertices()),
        timeStamp(0),
        settleCount(0) {
    }

    // --- Deleted Constructors (Kept for completeness, ensuring no fixed weights are used) ---

    // Delete all constructors that take a fixed weight vector
    CHTimeDependentDijkstra(const GRAPH& graph) = delete;
    CHTimeDependentDijkstra(const GRAPH& graph, const std::vector<int>& weight) = delete;
    CHTimeDependentDijkstra(const GRAPH&&, const std::vector<int>&) = delete;
    CHTimeDependentDijkstra(const GRAPH&, const std::vector<int>&&) = delete;
    CHTimeDependentDijkstra(const GRAPH&&) = delete;

    // Delete constructors using AttributeNameWrapper
    template<AttributeNameType ATTRIBUTE_NAME>
    CHTimeDependentDijkstra(const GRAPH& graph, const ::AttributeNameWrapper<ATTRIBUTE_NAME> weight) = delete;
    template<AttributeNameType ATTRIBUTE_NAME>
    CHTimeDependentDijkstra(const GRAPH&&, const ::AttributeNameWrapper<ATTRIBUTE_NAME>) = delete;

    // --- Public run methods ---

    /**
     * @brief Performs initial CH transfers and then runs TD-Dijkstra from reachable stops.
     * @param source The starting vertex.
     * @param departureTime The time of departure from the source.
     * @param target The target vertex.
     */

    template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        clear();

        // Step 1: Run static Initial Transfers (CH) from the source to all Stops/POIs.
        // Assumes INITIAL_TRANSFERS::run() signature from DijkstraRAPTOR is available.
        // NOTE: If target is a stop, the initial transfer can also provide the final EAT.
        initialTransfers.run(source, target);

        // Step 2: Add all stops reachable by the initial transfer as TD-Dijkstra sources.
        // This is the "initialization" step that starts the time-dependent search.
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            const int arrivalTime = departureTime + initialTransfers.getForwardDistance(stop);
            if (arrivalTime < intMax) {
                // Initial arrival time at a stop/POI from the source via CH is the new source departure time for TD-Dijkstra.
                addSource(stop, arrivalTime);
            }
        }

        // Step 3: Run the core TD-Dijkstra relaxation from all initialized sources.
        runRelaxation(target, settle, stop, pruneEdge);
    }

    // Legacy single-source run is now internal or renamed to avoid confusion
    template<typename SETTLE = NO_OPERATION, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION>
    inline void run(const Vertex source, const Vertex target = noVertex, const SETTLE& settle = NoOperation, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        // This method needs an initial time, so let's enforce using the three-parameter run()
        // OR define it to use a default departure time of 0
        run(source, 0, target, settle, stop, pruneEdge);
    }

    // Other run overloads (simplified to call the main run or deleted for brevity)

    inline void clear() noexcept {
        if constexpr (Debug) {
            timer.restart();
            settleCount = 0;
        }
        Q.clear();
        timeStamp++;
    }

    // 'distance' is now the initial departure time (EAT from a preceding transfer)
    inline void addSource(const Vertex source, const int distance) noexcept {
        VertexLabel& sourceLabel = getLabel(source);
        sourceLabel.distance = distance;
        Q.update(&sourceLabel);
    }

    // --- Core TD-Dijkstra Relaxation Logic (renamed and corrected) ---
    template<typename SETTLE, typename STOP = NO_OPERATION, typename PRUNE_EDGE = NO_OPERATION, typename = decltype(std::declval<SETTLE>()(std::declval<Vertex>()))>
    inline void runRelaxation(const Vertex target, const SETTLE& settle, const STOP& stop = NoOperation, const PRUNE_EDGE& pruneEdge = NoOperation) noexcept {
        while(!Q.empty()) {
            if (stop()) break;
            const VertexLabel* uLabel = Q.extractFront();
            const Vertex u = Vertex(uLabel - &(label[0]));
            if (u == target) break;

            // The arrival time at u (uLabel->distance) is the departure time for all outgoing edges.
            const int departureTime = uLabel->distance;

            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, edge);
                VertexLabel& vLabel = getLabel(v);
                if (pruneEdge(u, edge)) continue;

                // 1. TD-Dijkstra Step: Look up the earliest arrival time (EAT) at v.
                // NOTE: TimeDependentGraph::getArrivalTime returns the final ARRIVAL time!
                const int newArrivalTime = graph.getArrivalTime(edge, departureTime);

                // Check for unreachable (time = never)
                if (newArrivalTime == intMax) continue;

                // 2. Relaxation: Update if a better arrival time is found.
                if (vLabel.distance > newArrivalTime) {
                    vLabel.distance = newArrivalTime;
                    vLabel.parent = u;
                    Q.update(&vLabel);
                }
            }
            settle(u);
            if constexpr (Debug) settleCount++;
        }
        // ... (Debug output remains the same) ...
    }

    // --- Accessor methods (remain largely unchanged) ---

    inline bool reachable(const Vertex vertex) const noexcept {
        // Also check if the target was reached by the initial CH transfer
        if (label[vertex].timeStamp == timeStamp) return true;
        // Check initial transfer distance if the target is reachable by it
        return initialTransfers.getBackwardDistance(vertex) != intMax;
    }

    inline bool visited(const Vertex vertex) const noexcept {
        return label[vertex].timeStamp == timeStamp;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        // Get the best time: either from TD-Dijkstra or the initial CH transfer
        const int tdResult = (visited(vertex)) ? label[vertex].distance : intMax;
        const int chResult = initialTransfers.getDistance(vertex); // Assuming a method to get S-T distance

        return std::min(tdResult, chResult);
    }

    // ... (other accessors remain the same) ...


private:
    inline VertexLabel& getLabel(const Vertex vertex) noexcept {
        VertexLabel& result = label[vertex];
        if (result.timeStamp != timeStamp) result.reset(timeStamp);
        return result;
    }

private:
    const GRAPH& graph;
    const INITIAL_TRANSFERS& initialTransfers; // New member to hold the CH/Initial Transfer object

    ExternalKHeap<2, VertexLabel> Q;

    std::vector<VertexLabel> label;
    int timeStamp;

    int settleCount;
    Timer timer;
};