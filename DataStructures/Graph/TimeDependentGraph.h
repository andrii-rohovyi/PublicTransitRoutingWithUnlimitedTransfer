#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <map>
#include <unordered_map>
#include <iostream>

// --- CORE INCLUDES ---
#include "../../Helpers/Types.h"
#include "../../Helpers/Meta.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/Classes/DynamicGraph.h"
#include "../../Helpers/IO/Serialization.h"

// --- CONSTRUCTOR DEPENDENCIES ---
#include "../../DataStructures/RAPTOR/Data.h"
// NOTE: Must include Intermediate::Data if FromIntermediate is implemented here.
// Intermediate::Data is included where this header is used (Benchmark commands),
// and Helpers/Types.h already defines the constant 'never'. Avoid redefining it here.

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
    // Suffix minimum of arrival times over discreteTrips sorted by departureTime.
    // suffixMinArrival[i] = min_{j >= i} discreteTrips[j].arrivalTime
    std::vector<int> suffixMinArrival;

    inline void finalize() noexcept {
        // Ensure discreteTrips are sorted by departure time before computing suffix mins
        std::sort(discreteTrips.begin(), discreteTrips.end());
        suffixMinArrival.resize(discreteTrips.size());
        if (!discreteTrips.empty()) {
            suffixMinArrival.back() = discreteTrips.back().arrivalTime;
            for (int i = int(discreteTrips.size()) - 2; i >= 0; --i) {
                suffixMinArrival[i] = std::min(discreteTrips[i].arrivalTime, suffixMinArrival[i + 1]);
            }
        }
    }

    inline int computeArrivalTime(const int departureTime) const noexcept {
        int minArrivalTime = never;

        // 1. Check for the first relevant discrete trip (Bus)
        // Equivalent to: bisect_left(self.buses, t, key=lambda x: x.d)
        auto it = std::lower_bound(discreteTrips.begin(), discreteTrips.end(), departureTime,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });

        if (it != discreteTrips.end()) {
            const size_t idx = size_t(it - discreteTrips.begin());
            // Use suffix minimum to get the best (earliest) arrival among all eligible trips
            if (!suffixMinArrival.empty()) minArrivalTime = suffixMinArrival[idx];
            else minArrivalTime = it->arrivalTime; // Fallback (shouldn't happen if finalize() was called)
        }

        // 2. Check for the static walk/transfer
        if (walkTime != never) {
            const int currentWalkTime = departureTime + walkTime;
            minArrivalTime = std::min(minArrivalTime, currentWalkTime);
        }

        return minArrivalTime;
    }

    // Compute arrival time when a boarding buffer is required before boarding a scheduled trip.
    // The buffer applies ONLY to discrete trips (boarding vehicles), not to walking transfers.
    inline int computeArrivalTimeWithBoardBuffer(const int departureTime, const int boardBuffer) const noexcept {
        int minArrivalTime = never;

        // 1) Scheduled trips: must depart at or after (departureTime + boardBuffer)
        const int threshold = (boardBuffer > 0) ? (departureTime + boardBuffer) : departureTime;
        auto it = std::lower_bound(discreteTrips.begin(), discreteTrips.end(), threshold,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });
        if (it != discreteTrips.end()) {
            const size_t idx = size_t(it - discreteTrips.begin());
            if (!suffixMinArrival.empty()) minArrivalTime = suffixMinArrival[idx];
            else minArrivalTime = it->arrivalTime;
        }

        // 2) Walking: no boarding buffer applies
        if (walkTime != never) {
            const int currentWalkTime = departureTime + walkTime;
            minArrivalTime = std::min(minArrivalTime, currentWalkTime);
        }

        return minArrivalTime;
    }

    // Return earliest arrival among discrete trips departing at or after 'threshold'.
    inline int computeDiscreteArrivalFrom(const int threshold) const noexcept {
        if (discreteTrips.empty()) return never;
        auto it = std::lower_bound(discreteTrips.begin(), discreteTrips.end(), threshold,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });
        if (it == discreteTrips.end()) return never;
        const size_t idx = size_t(it - discreteTrips.begin());
        if (!suffixMinArrival.empty()) return suffixMinArrival[idx];
        return it->arrivalTime;
    }

    // Return arrival time for walking from departureTime; never if walk is not available.
    inline int computeWalkArrivalFrom(const int departureTime) const noexcept {
        if (walkTime == never) return never;
        return departureTime + walkTime;
    }

    // Custom serialization to ensure nested vectors are persisted correctly
    inline void serialize(IO::Serialization& s) const noexcept {
        s(discreteTrips, walkTime, suffixMinArrival);
    }

    inline void deserialize(IO::Deserialization& d) noexcept {
        d(discreteTrips, walkTime, suffixMinArrival);
        // Backward compatibility: if suffixMinArrival missing (older binaries), rebuild it.
        if (suffixMinArrival.size() != discreteTrips.size()) finalize();
    }
};

// =========================================================================
// 2. TimeDependentGraph Wrapper Class
// =========================================================================

// Helper for hashing vertex pairs
struct VertexPairHash {
    std::size_t operator()(const std::pair<Vertex, Vertex>& p) const {
        return std::hash<size_t>()(size_t(p.first)) ^ (std::hash<size_t>()(size_t(p.second)) << 1);
    }
};

class TimeDependentGraph {
private:
    using TDEdgeAttributes = Meta::List<
        ::Attribute<FromVertex, Vertex>,
        ::Attribute<ToVertex, Vertex>,
        ::Attribute<Valid, bool>,
        ::Attribute<IncomingEdgePointer, size_t>,
        ::Attribute<ReverseEdge, Edge>,
        // Store the arrival-time function directly on the edge to avoid index mismatches.
        ::Attribute<Function, ArrivalTimeFunction>
    >;

    // FIX: Changed Meta::Attribute to ::Attribute to reference the global definition.
    using TDVertexAttributes = Meta::List<
        ::Attribute<BeginOut, Edge>,
        ::Attribute<OutDegree, size_t>,
        ::Attribute<IncomingEdges, std::vector<Edge>>
    >;

    using UnderlyingGraph = DynamicGraphImplementation<TDVertexAttributes, TDEdgeAttributes>;

    UnderlyingGraph graph;
    // Per-vertex minimum transfer time (change time) used as implicit departure buffer when boarding trips.
    // Initialized from Intermediate::Data::stops; defaults to 0 for non-stop vertices.
    std::vector<int> minTransferTimeByVertex;

public:
    TimeDependentGraph() = default;

    /**
     * @brief Constructs a TimeDependentGraph directly from Intermediate::Data
     * using Discrete Arrival Time Functions (ATF).
     */
    inline static TimeDependentGraph FromIntermediate(const Intermediate::Data& inter) noexcept {
        TimeDependentGraph tdGraph;
        const size_t numStops = inter.numberOfStops();
        const size_t numVertices = inter.transferGraph.numVertices();

        // 1. Initialize all vertices (stops + intermediate transfer vertices)
        for (size_t i = 0; i < numVertices; ++i) {
            tdGraph.graph.addVertex();
        }

        // Initialize per-vertex min transfer times from Intermediate stops if available
        tdGraph.minTransferTimeByVertex.assign(numVertices, 0);
        for (size_t s = 0; s < std::min(numStops, numVertices); ++s) {
            tdGraph.minTransferTimeByVertex[s] = inter.stops[s].minTransferTime;
        }

        // 2. Collect Discrete Trips more efficiently using unordered_map with custom hash
        std::unordered_map<std::pair<Vertex, Vertex>, std::vector<DiscreteTrip>, VertexPairHash> tripSegments;
        tripSegments.reserve(inter.trips.size() * 10); // Pre-allocate to reduce rehashing

        std::cout << "Building trip segments from " << inter.trips.size() << " trips..." << std::flush;
        for (const Intermediate::Trip& trip : inter.trips) {
            for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
                const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
                const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];

                const Vertex u = Vertex(stopEventU.stopId);
                const Vertex v = Vertex(stopEventV.stopId);

                // Use original departure times from intermediate data.
                // The TD-Dijkstra will apply the buffer when boarding.
                tripSegments[{u, v}].emplace_back(DiscreteTrip{
                    .departureTime = stopEventU.departureTime,
                    .arrivalTime = stopEventV.arrivalTime
                });
            }
        }
        std::cout << " done (" << tripSegments.size() << " unique segments)" << std::endl;

        // 3. Build transfer map more efficiently
        std::unordered_map<std::pair<Vertex, Vertex>, int, VertexPairHash> minTransferTimes;
        minTransferTimes.reserve(inter.transferGraph.numEdges());
        
        std::cout << "Building transfer times..." << std::flush;
        const Intermediate::TransferGraph& interTransferGraph = inter.transferGraph;

        for (const Vertex u : interTransferGraph.vertices()) {
            for (const Edge edge : interTransferGraph.edgesFrom(u)) {
                const Vertex v = interTransferGraph.get(ToVertex, edge);
                const int travelTime = interTransferGraph.get(TravelTime, edge);

                auto key = std::make_pair(u, v);
                auto it = minTransferTimes.find(key);
                if (it == minTransferTimes.end()) {
                    minTransferTimes[key] = travelTime;
                } else {
                    it->second = std::min(it->second, travelTime);
                }
            }
        }
        std::cout << " done (" << minTransferTimes.size() << " transfers)" << std::endl;

        // 4. Create Edges for all segments (Trip and Transfer combined)
        std::cout << "Creating time-dependent edges..." << std::flush;
        size_t edgeCount = 0;
        
        for (auto& pair : tripSegments) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            std::vector<DiscreteTrip>& trips = pair.second;

            // Check if there's also a transfer for this edge
            auto transferIt = minTransferTimes.find({u, v});
            int walkTime = (transferIt != minTransferTimes.end()) ? transferIt->second : never;
            
            // Remove transfer to avoid duplication in step 5
            if (transferIt != minTransferTimes.end()) {
                minTransferTimes.erase(transferIt);
            }

            // Create and add the ArrivalTimeFunction (ATF)
            ArrivalTimeFunction func;
            func.discreteTrips = std::move(trips);
            func.walkTime = walkTime;
            func.finalize();

            tdGraph.addTimeDependentEdge(u, v, func);
            edgeCount++;
        }

        // 5. Handle Edges with ONLY Transfers (Walk)
        for (const auto& pair : minTransferTimes) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            const int walkTime = pair.second;

            // Create a function with an empty trip list and only the walk time
            ArrivalTimeFunction func;
            func.walkTime = walkTime;
            func.finalize();

            tdGraph.addTimeDependentEdge(u, v, func);
            edgeCount++;
        }
        
        std::cout << " done (" << edgeCount << " edges created)" << std::endl;

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

    template<typename ATTRIBUTE>
    inline auto get(const ATTRIBUTE& attribute, const Edge edge) const noexcept {
        return graph.get(attribute, edge);
    }

    /**
     * @brief Computes the arrival time given the edge and departure time.
     */
    inline int getArrivalTime(const Edge edge, const int departureTime) const noexcept {
        const ArrivalTimeFunction& f = graph.get(Function, edge);
        // Do NOT apply a generic per-stop boarding buffer here, as that would incorrectly penalize
        // continuing on the same vehicle across consecutive stops. Buffer semantics are handled in MR by
        // implicit departure times and require stateful modeling to apply only on transfers. For TD-Dijkstra
        // (single-label), we use the pure arrival-time function without buffer to avoid over-constraining.
        return f.computeArrivalTime(departureTime);
    }

    // Stateful evaluation utilities
    inline int getDiscreteArrivalFromThreshold(const Edge edge, const int threshold) const noexcept {
        const ArrivalTimeFunction& f = graph.get(Function, edge);
        return f.computeDiscreteArrivalFrom(threshold);
    }

    inline int getWalkArrivalFrom(const Edge edge, const int departureTime) const noexcept {
        const ArrivalTimeFunction& f = graph.get(Function, edge);
        return f.computeWalkArrivalFrom(departureTime);
    }

    inline int getMinTransferTimeAt(const Vertex u) const noexcept {
        return (size_t(u) < minTransferTimeByVertex.size()) ? minTransferTimeByVertex[u] : 0;
    }

    // --- Public Utility/Manipulation Functions ---

    inline Vertex addVertex() {
        return graph.addVertex();
    }

    inline typename UnderlyingGraph::EdgeHandle addTimeDependentEdge(const Vertex from, const Vertex to, const ArrivalTimeFunction& func) {
        typename UnderlyingGraph::EdgeHandle handle = graph.addEdge(from, to);
        handle.set(Function, func);
        return handle;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        // Persist the underlying dynamic graph (includes per-edge ArrivalTimeFunction attribute)
        graph.writeBinary(fileName);
    }

    /**
     * @brief Deserializes the TimeDependentGraph from a binary file.
     * * @param fileName The path to the input file.
     */
    inline void deserialize(const std::string& fileName) noexcept {
        // Load the underlying dynamic graph (includes per-edge ArrivalTimeFunction attribute)
        graph.readBinary(fileName);
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