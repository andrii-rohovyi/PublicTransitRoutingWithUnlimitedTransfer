#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <map>
#include <iostream>

// --- CORE INCLUDES ---
#include "../../Helpers/Types.h"
#include "../../Helpers/Meta.h"
#include "../../DataStructures/Attributes/AttributeNames.h" // <-- Contains TravelTimeFunctionIndex
#include "../../DataStructures/Graph/Classes/DynamicGraph.h"

// --- CONSTRUCTOR DEPENDENCIES ---
#include "../../DataStructures/RAPTOR/Data.h"
// NOTE: Must include Intermediate::Data if FromIntermediate is implemented here.
// Assuming it's available via an include or alias not shown, e.g.:
// #include "../Intermediate/Data.h"

// Define never if not already in Types.h
#ifndef never
#define never std::numeric_limits<int>::max()
#endif

using TransferGraph = ::TransferGraph;


// =========================================================================
// 1. Core Time-Dependent Data Structure (Discrete Schedule Arrival Function)
// =========================================================================

/**
 * @brief Represents a single discrete trip (Bus) segment between two stops.
 */
struct DiscreteTrip {
    int departureTime;
    int arrivalTime;

    inline bool operator<(const DiscreteTrip& other) const noexcept {
        return departureTime < other.departureTime;
    }
    inline bool operator<(const int time) const noexcept {
        return departureTime < time;
    }
};

/**
 * @brief Arrival Time Function (ATF) based on discrete trips and static walk/transfer.
 */
struct ArrivalTimeFunction {
    std::vector<DiscreteTrip> discreteTrips;
    int walkTime = never; // Initialize walkTime to a large value

    inline int computeArrivalTime(const int departureTime) const noexcept {
        int minArrivalTime = never;

        // 1. Check for the first relevant discrete trip (Bus)
        // Equivalent to: bisect_left(self.buses, t, key=lambda x: x.d)
        auto it = std::lower_bound(discreteTrips.begin(), discreteTrips.end(), departureTime,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });

        if (it != discreteTrips.end()) {
            minArrivalTime = it->arrivalTime;
        }

        // 2. Check for the static walk/transfer
        if (walkTime != never) {
            const int currentWalkTime = departureTime + walkTime;
            minArrivalTime = std::min(minArrivalTime, currentWalkTime);
        }

        return minArrivalTime;
    }
};

// =========================================================================
// 2. TimeDependentGraph Wrapper Class
// =========================================================================

class TimeDependentGraph {
private:
    using TDEdgeAttributes = Meta::List<
        ::Attribute<FromVertex, Vertex>,
        ::Attribute<ToVertex, Vertex>,
        ::Attribute<Valid, bool>,
        ::Attribute<IncomingEdgePointer, size_t>,
        ::Attribute<ReverseEdge, Edge>,
        ::Attribute<TravelTimeFunctionIndex, int>
    >;

    // FIX: Changed Meta::Attribute to ::Attribute to reference the global definition.
    using TDVertexAttributes = Meta::List<
        ::Attribute<BeginOut, Edge>,
        ::Attribute<OutDegree, size_t>,
        ::Attribute<IncomingEdges, std::vector<Edge>>
    >;

    using UnderlyingGraph = DynamicGraphImplementation<TDVertexAttributes, TDEdgeAttributes>;

    UnderlyingGraph graph;
    std::vector<ArrivalTimeFunction> functionPool;

public:
    TimeDependentGraph() = default;

    /**
     * @brief Constructs a TimeDependentGraph directly from Intermediate::Data
     * using Discrete Arrival Time Functions (ATF).
     */
    inline static TimeDependentGraph FromIntermediate(const Intermediate::Data& inter) noexcept {
        TimeDependentGraph tdGraph;
        const size_t numVertices = inter.numberOfStops();

        // 1. Initialize vertices
        for (size_t i = 0; i < numVertices; ++i) {
            tdGraph.graph.addVertex();
        }

        // 2. Collect Discrete Trips
        std::map<std::pair<Vertex, Vertex>, std::vector<DiscreteTrip>> tripSegments;

        for (const Intermediate::Trip& trip : inter.trips) {
            for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
                const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
                const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];

                const Vertex u = stopEventU.stopId;
                const Vertex v = stopEventV.stopId;

                tripSegments[{u, v}].emplace_back(DiscreteTrip{
                    .departureTime = stopEventU.departureTime,
                    .arrivalTime = stopEventV.arrivalTime
                });
            }
        }

        // 3. Map Minimum Transfer Times (Walk Time)
        std::map<std::pair<Vertex, Vertex>, int> minTransferTimes;
        const TransferGraph& interTransferGraph = inter.transferGraph;

        for (const Vertex u : interTransferGraph.vertices()) {
            for (const Edge edge : interTransferGraph.edgesFrom(u)) {
                const Vertex v = interTransferGraph.get(ToVertex, edge);
                const int travelTime = interTransferGraph.get(TravelTime, edge);

                auto key = std::make_pair(u, v);
                minTransferTimes[key] = std::min(minTransferTimes.count(key) ? minTransferTimes.at(key) : never, travelTime);
            }
        }

        // 4. Create Edges for all segments (Trip and Transfer)
        for (auto& pair : tripSegments) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            std::vector<DiscreteTrip>& trips = pair.second;

            int walkTime = minTransferTimes.count({u, v}) ? minTransferTimes.at({u, v}) : never;

            // Remove transfer entry to avoid duplication in step 5
            if (minTransferTimes.count({u, v})) {
                minTransferTimes.erase({u, v});
            }

            // Create and add the ArrivalTimeFunction (ATF)
            ArrivalTimeFunction func;
            func.discreteTrips = std::move(trips);
            std::sort(func.discreteTrips.begin(), func.discreteTrips.end());
            func.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, func);
        }

        // 5. Handle Edges with ONLY Transfers (Walk)
        for (const auto& pair : minTransferTimes) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            const int walkTime = pair.second;

            // Create a function with an empty trip list and only the walk time
            ArrivalTimeFunction func;
            func.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, func);
        }

        // 6. Add remaining non-stop vertices from the transfer graph if necessary
        if (inter.transferGraph.numVertices() > numVertices) {
            for (Vertex i = Vertex(numVertices); i < inter.transferGraph.numVertices(); ++i) {
                tdGraph.graph.addVertex();
            }
        }

        return tdGraph;
    }


    // --- Accessors for TD-Dijkstra ---

    inline size_t numVertices() const noexcept {
        return graph.numVertices();
    }

    inline auto edgesFrom(const Vertex u) const noexcept {
        return graph.edgesFrom(u);
    }

    inline size_t numEdges() const noexcept {
        return graph.numEdges();
    }

    /**
     * @brief Computes the arrival time given the edge and departure time.
     */
    inline int getArrivalTime(const Edge edge, const int departureTime) const noexcept {
        const int funcIndex = graph.get(TravelTimeFunctionIndex, edge);
        if (funcIndex == -1 || funcIndex >= (int)functionPool.size()) {
            return never;
        }
        return functionPool[funcIndex].computeArrivalTime(departureTime);
    }

    // --- Public Utility/Manipulation Functions ---

    inline Vertex addVertex() {
        return graph.addVertex();
    }

    inline typename UnderlyingGraph::EdgeHandle addTimeDependentEdge(const Vertex from, const Vertex to, const ArrivalTimeFunction& func) {
        const int funcIndex = functionPool.size();
        functionPool.emplace_back(func);

        typename UnderlyingGraph::EdgeHandle handle = graph.addEdge(from, to);

        handle.set(TravelTimeFunctionIndex, funcIndex);

        return handle;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        // Correctly serialize the two member variables of TimeDependentGraph:
        // 1. The DynamicGraph (graph)
        // 2. The function pool (functionPool)
        IO::serialize(fileName, graph, functionPool);
    }

    /**
     * @brief Deserializes the TimeDependentGraph from a binary file.
     * * @param fileName The path to the input file.
     */
    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, graph, functionPool);
    }

    /**
     * @brief Static method to create a TimeDependentGraph from a binary file.
     */
    inline static TimeDependentGraph FromBinary(const std::string& fileName) noexcept {
        TimeDependentGraph tdGraph;
        tdGraph.deserialize(fileName);
        return tdGraph;
    }
};