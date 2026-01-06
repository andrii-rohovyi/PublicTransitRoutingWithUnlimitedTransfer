#pragma once

// ============================================================================
// IMPORTANT: Compatibility with TimeDependentGraph.h
//
// If TimeDependentGraph.h has already been included, we don't redefine the
// structs. Both graph classes use the same struct definitions and are
// compatible with TimeDependentDijkstraStateful.
// ============================================================================

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

using TransferGraph = ::TransferGraph;

// =========================================================================
// IMPORTANT: DiscreteTrip and EdgeTripsHandle structs
// =========================================================================
// These structs are defined in TimeDependentGraph.h and are identical for both
// TimeDependentGraph and TimeDependentGraphClassic. We do NOT redefine them here.
// VertexPairHash is also defined there and reused here.
// Make sure TimeDependentGraph.h is included before this header.
// =========================================================================

// =========================================================================
// TimeDependentGraphClassic Class
// =========================================================================

/*
[CORE LOGIC] Represents the graph data structure for time-dependent queries with dominated edge filtering:
1. Vertices/Edges use a standard adjacency list (DynamicGraph) for topology.
2. Timetable data is stored in separate, flattened vectors for cache locality.
3. Edges store 'EdgeTripsHandle' indices into these flattened vectors.
4. ENHANCEMENT: Implements dominated connection filtering (cut operation) to reduce graph size.

[USAGE] The main data structure instantiated by the routing engine
(TimeDependentDijkstraStateful) to answer queries with optimized edge sets.
*/
class TimeDependentGraphClassic {
public:
    using EdgeTripsHandle = ::EdgeTripsHandle;  // Export handle type for generic algorithms

private:
    using TDEdgeAttributes = Meta::List<            // The schema for edges
        ::Attribute<FromVertex, Vertex>,            // The ID of the source vertex (u)
        ::Attribute<ToVertex, Vertex>,              // The ID of the target vertex (v)
        ::Attribute<Valid, bool>,                   // Whether an edge is deleted/disabled
        ::Attribute<IncomingEdgePointer, size_t>,   // Links an edge to the target node's list of incoming edges
        ::Attribute<ReverseEdge, Edge>,             // The ID edge of the reverse edge (v -> u)
        ::Attribute<Function, EdgeTripsHandle>  // Handle to the trip data
    >;

    using TDVertexAttributes = Meta::List<              // The schema for vertices
        ::Attribute<BeginOut, Edge>,                    // The ID of the first outgoing edge
        ::Attribute<OutDegree, size_t>,                 // Number of outgoing edges for iterating over neighbors
        ::Attribute<IncomingEdges, std::vector<Edge>>   // List of incoming edge IDs
    >;

    // A generic generator for creating a DynamicGraph with specified vertex and edge attributes
    using UnderlyingGraph = DynamicGraphImplementation<TDVertexAttributes, TDEdgeAttributes>;

    UnderlyingGraph graph;                    // The topology engine that manages vertices and edges
    std::vector<int> minTransferTimeByVertex; // Auxiliary data: min transfer times at each stop vertex

public:
    /*
    [CORE LOGIC] Represents a single stop within a full vehicle journey.

    [USAGE] Stored in the flattened 'allTripLegs' vector. Accessed via linear scan
    inside 'scanTrip' (Dijkstra) to traverse the route after boarding.
    */
    struct TripLeg {
        int arrivalTime;
        Vertex stopId;
    };

    // [OPTIMIZATION] Flattened data store (array of structs).
    // Instead of each Edge having a std::vector<Trip>, we store ALL trips for ALL edges
    // in one massive std::vector. The Edges store indices (begin/count).
    //
    // [USAGE] The primary storage for all schedule events.
    std::vector<DiscreteTrip> allDiscreteTrips;

    // [OPTIMIZATION] Suffix minima (range minimum queries) for trip arrival times.
    // For every trip list, we pre-calculate the minimum arrival time of all subsequent trips.
    // allSuffixMinArrivals[i] = min(trips[i].arr, trips[i+1].arr, ...)
    //
    // [USAGE] Accessed during query relaxation inside 'getArrivalTime' to skip linear scans.
    std::vector<int> allSuffixMinArrivals;

private:
    // [OPTIMIZATION] Flattened trip data storage (Compressed Sparse Row style).
    // tripOffsets[tripId] gives the starting index in allTripLegs for that trip.
    std::vector<uint32_t> tripOffsets;

    // [USAGE] Stores the actual sequence of stops (Time, StopID) for all trips contiguously.
    // Important for cache locality during trip scans.
    std::vector<TripLeg> allTripLegs;

    // [STATISTICS] Track filtering effectiveness
    size_t totalTripsBeforeFilter = 0;
    size_t totalTripsAfterFilter = 0;

public:
    // Create an empty constructor now, fill via FromIntermediate or FromBinary later
    TimeDependentGraphClassic() = default;

    // [CORE LOGIC] Trip traversal: given a tripId and current stop index,
    // retrieve the next stop and its arrival time. This function answers the question:
    // "If I'm on trip X at stop index Y, where do I go next and when do I get there?"
    //
    // [USAGE] Used by Dijkstra's 'scanTrip' function to traverse a specific vehicle's route.
    inline bool getNextStop(const int tripId, const uint16_t currentStopIndex, Vertex& outStop, int& outArrival) const noexcept {
        // Verify tripId validity
        if (tripId < 0 || (size_t)tripId + 1 >= tripOffsets.size()) return false;
        // Retrieve the absolute index where this specific trip begins in allTripLegs
        const uint32_t currentTripStart = tripOffsets[tripId];
        // Retrieve the starting index of the next trip to determine bounds
        const uint32_t nextTripStart = tripOffsets[tripId + 1];
        // Calculate the absolute index of the next stop
        const uint32_t absoluteIndex = currentTripStart + currentStopIndex + 1;
        // "Does the next stop exist within the trip's bounds?"
        if (absoluteIndex < nextTripStart) {
            // Fetch the next stop's data
            const TripLeg& leg = allTripLegs[absoluteIndex];
            // Copy the data into the reference variables provided by the caller
            outStop = leg.stopId;
            outArrival = leg.arrivalTime;
            return true;
        }
        // The next stop does not exist (end of trip reached)
        return false;
    }

    // [USAGE] Internal lookup for stop details. Used when iterating trips linearly.
    inline const TripLeg& getTripLeg(const size_t index) const noexcept {
        return allTripLegs[index];
    }

    // [USAGE] Returns total number of stop events (size of flattened schedule).
    // Essential for sizing the 'globalVehicleLabels' vector in the Dijkstra algorithm to match the graph size.
    inline size_t getNumStopEvents() const noexcept {
        return allTripLegs.size();
    }

    // [USAGE] Maps a logical Trip ID (from DiscreteTrip) to its start index in the flat 'allTripLegs' array.
    inline uint32_t getTripOffset(const int tripId) const noexcept {
        return tripOffsets[tripId];
    }

    // [USAGE] Returns iterator to the start of an edge's trip list. Used for std::lower_bound in 'getArrivalTime'.
    // Direct pointer arithmetic into the global array avoids std::vector iterator overhead.
    inline const DiscreteTrip* getTripsBegin(const EdgeTripsHandle& h) const noexcept {
        return &allDiscreteTrips[h.firstTripIndex];
    }

    // [USAGE] Returns iterator to the end of an edge's trip list.
    inline const DiscreteTrip* getTripsEnd(const EdgeTripsHandle& h) const noexcept {
        return &allDiscreteTrips[h.firstTripIndex + h.tripCount];
    }

    // [USAGE] Returns pointer to the pre-calculated suffix-min array for an edge. Used for O(1) lookups.
    inline const int* getSuffixMinBegin(const EdgeTripsHandle& h) const noexcept {
        return &allSuffixMinArrivals[h.firstSuffixIndex];
    }

    // [USAGE] Return type for 'findMatchingTrip'.
    // Wraps the result of the backward search used during path reconstruction.
    // If no trip matches the specific (u, v, dep, arr) tuple, tripId remains -1.
    struct FoundTrip {
        int tripId = -1;
    };

    // [CORE LOGIC] Helper for path reconstruction.
    // Finds a tripId that goes from u -> v, departing >= minDepTime and arriving == atArrTime.
    //
    // [USAGE] Called by TimeDependentDijkstraStateful::getPath() during the backward pass
    // to recover the tripId since we don't store it during the hot forward scan.
    inline FoundTrip findMatchingTrip(Vertex u, Vertex v, int minDepTime, int atArrTime) const noexcept {
        // Iterate edges from u to find connection to v
        for (const Edge e : graph.edgesFrom(u)) {
            if (graph.get(ToVertex, e) == v) {
                const EdgeTripsHandle& h = graph.get(Function, e);
                const DiscreteTrip* begin = getTripsBegin(h);
                const DiscreteTrip* end = getTripsEnd(h);

                // Use lower_bound to find potential trips efficiently
                auto it = std::lower_bound(begin, end, minDepTime,
                    [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

                for (; it != end; ++it) {
                    // Check if this trip arrives exactly at the recorded time
                    if (it->arrivalTime == atArrTime) {
                         return {it->tripId};
                    }
                }
            }
        }
        return {-1};
    }

    // [NEW FEATURE] Dominated connection filtering (ATF cut operation)
    // Removes connections that are strictly dominated by other connections.
    // A connection (d1, a1) dominates (d2, a2) if d1 <= d2 and a1 < a2
    //
    // [USAGE] Called during graph construction to reduce the number of connections
    // that need to be stored and evaluated during queries.
    //
    // Algorithm (based on Python ATF.cut()):
    // - Iterate through sorted connections (by departure time)
    // - Keep a result list of non-dominated connections
    // - For each connection, check if it's dominated by the last kept connection
    // - Also filter out connections slower than walking (if walk exists)
    inline static std::vector<DiscreteTrip> filterDominatedConnections(
        std::vector<DiscreteTrip>& trips,
        int walkTime) noexcept {

        if (trips.empty()) return trips;

        std::vector<DiscreteTrip> result;
        result.reserve(trips.size());

        for (size_t i = 0; i < trips.size(); ++i) {
            const auto& currentTrip = trips[i];

            // Check if this trip is slower than walking
            if (walkTime != never) {
                int transitTime = currentTrip.arrivalTime - currentTrip.departureTime;
                if (transitTime > walkTime) {
                    // This connection is slower than walking, skip it
                    continue;
                }
            }

            // Check domination against the last kept connection
            // Python logic: if result is not empty and currentTrip.arrival > lastTrip.arrival, check departure
            // If departure is later, add current. Otherwise (same or earlier departure but worse arrival), pop last or skip
            while (!result.empty()) {
                const auto& lastTrip = result.back();

                // If current arrives STRICTLY earlier, it dominates last (same or later departure, earlier arrival)
                if (currentTrip.arrivalTime < lastTrip.arrivalTime) {
                    // Current dominates last - remove last
                    result.pop_back();
                } else {
                    // Current arrives later or same as last
                    // Only keep current if it departs strictly later
                    if (lastTrip.departureTime < currentTrip.departureTime) {
                        // Different departure times, both should be kept
                        break;
                    } else {
                        // Same departure time (or current is earlier?) but worse/same arrival - skip current
                        goto skip_current;
                    }
                }
            }

            // Add this trip if it survived domination check
            result.push_back(currentTrip);

            skip_current:;
        }

        return result;
    }

    // [FACTORY] Builds a TimeDependentGraphClassic from an Intermediate::Data representation.
    // Converts an "Intermediate" (RAPTOR-style) representation into this graph format.
    // ENHANCEMENT: Applies dominated connection filtering during construction.
    //
    // [USAGE] Called during the preprocessing phase to build the optimized graph structure.
    inline static TimeDependentGraphClassic FromIntermediate(const Intermediate::Data& inter) noexcept {
        TimeDependentGraphClassic tdGraph;
        const size_t numStops = inter.numberOfStops();
        const size_t numVertices = inter.transferGraph.numVertices();

        // 1. TOPOLOGY SETUP
        // Initialize graph with vertices
        for (size_t i = 0; i < numVertices; ++i) {
            tdGraph.graph.addVertex();
        }

        // Initialize stop-specific transfer buffers
        tdGraph.minTransferTimeByVertex.assign(numVertices, 0);
        for (size_t s = 0; s < std::min(numStops, numVertices); ++s) {
            tdGraph.minTransferTimeByVertex[s] = inter.stops[s].minTransferTime;
        }

        // 2. ROUTE DECOMPOSITION INTO EDGES
        // We iterate through every trip in the input. For every connection A->B in a trip,
        // we extract it and bucket it into a map key (A, B).
        std::unordered_map<std::pair<Vertex, Vertex>, std::vector<DiscreteTrip>, VertexPairHash> tripSegments;
        tripSegments.reserve(inter.trips.size() * 10);

        std::cout << "Building trip segments..." << std::flush;
        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];

            // Iterate through every stop in the trip to create edges (segments).
            // A trip with stops A->B->C creates segments A->B and B->C.
            for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
                const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
                const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];
                const Vertex u = Vertex(stopEventU.stopId);
                const Vertex v = Vertex(stopEventV.stopId);

                // To support transfer times we normally check:
                // ArrivalTime + TransferTime <= DepartureTime.
                //
                // [OPTIMIZATION] We pre-subtract the transfer time from the departure time here.
                // New Check: ArrivalTime <= (DepartureTime - TransferTime).
                // This saves an addition operation during the hot query loop.
                const int buffer = (u < inter.stops.size()) ? inter.stops[u].minTransferTime : 0;

                tripSegments[{u, v}].emplace_back(DiscreteTrip{
                    .departureTime = stopEventU.departureTime - buffer,   // Buffer applied here
                    .arrivalTime = stopEventV.arrivalTime,
                    .tripId = (int)tripId,
                    .departureStopIndex = (uint16_t)i
                });

                tdGraph.totalTripsBeforeFilter++;
            }
        }
        std::cout << " done." << std::endl;

        // 3. FLATTEN TRIP LEGS (CSR CONSTRUCTION)
        // Build the 'tripOffsets' and 'allTripLegs' vectors for the "On-Vehicle" phase.

        // Memory allocation
        tdGraph.tripOffsets.reserve(inter.trips.size() + 1);

        // Track the cumulative count of stops seen so far
        size_t totalStops = 0;
        for (const auto& trip : inter.trips) {
            tdGraph.tripOffsets.push_back(totalStops);
            totalStops += trip.stopEvents.size();
        }

        // Now that we know how many stops there are, allocate the allTripLegs vector
        // This way, we guarantee that all stop events are laid out sequentially in physical memory.
        tdGraph.tripOffsets.push_back(totalStops);
        tdGraph.allTripLegs.resize(totalStops);

        // Outer loop: retrieve baseOffset for each trip,
        // which tells us where to write each specific trip's stop events in allTripLegs.
        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];
            const size_t baseOffset = tdGraph.tripOffsets[tripId];

            // Inner loop: linear fill of allTripLegs for this trip
            for (size_t i = 0; i < trip.stopEvents.size(); ++i) {
                tdGraph.allTripLegs[baseOffset + i] = {
                    trip.stopEvents[i].arrivalTime,
                    Vertex(trip.stopEvents[i].stopId)
                };
            }
        }

        // 4. TRANSFER GRAPH PROCESSING
        // Load static walking edges (transfers) into a map for easy merging.
        std::unordered_map<std::pair<Vertex, Vertex>, int, VertexPairHash> minTransferTimes;
        minTransferTimes.reserve(inter.transferGraph.numEdges());

        const Intermediate::TransferGraph& interTransferGraph = inter.transferGraph;

        // Outer loop: iterate every vertex in the transfer graph
        for (const Vertex u : interTransferGraph.vertices()) {
            // Inner loop: iterate every outgoing edge from u
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

        // 5. GRAPH COMPILATION WITH DOMINATED CONNECTION FILTERING
        std::cout << "Creating time-dependent edges with domination filtering..." << std::flush;
        size_t edgeCount = 0;

        tdGraph.allDiscreteTrips.reserve(tripSegments.size() * 5);
        tdGraph.allSuffixMinArrivals.reserve(tripSegments.size() * 5);

        for (auto& pair : tripSegments) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            std::vector<DiscreteTrip>& trips = pair.second;

            // Check if there is also a walking option for this edge
            auto transferIt = minTransferTimes.find({u, v});
            int walkTime = (transferIt != minTransferTimes.end()) ? transferIt->second : never;
            if (transferIt != minTransferTimes.end()) {
                minTransferTimes.erase(transferIt);     // Erase to avoid double-adding later
            }

            // A. Sorting - REQUIRED for both binary search and domination filtering
            std::sort(trips.begin(), trips.end());

            // B. NEW: Apply dominated connection filtering (ATF cut)
            // This reduces the number of connections by removing dominated ones
            std::vector<DiscreteTrip> filteredTrips = filterDominatedConnections(trips, walkTime);

            tdGraph.totalTripsAfterFilter += filteredTrips.size();

            if (filteredTrips.empty() && walkTime == never) {
                // No connections on this edge at all - skip it
                continue;
            }

            // C. Flattening into global arrays
            uint32_t firstTripIdx = tdGraph.allDiscreteTrips.size();
            uint32_t firstSuffixIdx = tdGraph.allSuffixMinArrivals.size();

            tdGraph.allDiscreteTrips.insert(tdGraph.allDiscreteTrips.end(),
                                           filteredTrips.begin(), filteredTrips.end());

            // D. Suffix minima (REQUIRED for O(1) arrival query)
            size_t startSize = tdGraph.allSuffixMinArrivals.size();
            tdGraph.allSuffixMinArrivals.resize(startSize + filteredTrips.size());
            if (!filteredTrips.empty()) {
                tdGraph.allSuffixMinArrivals.back() = filteredTrips.back().arrivalTime;
                for (int i = int(filteredTrips.size()) - 2; i >= 0; --i) {
                    tdGraph.allSuffixMinArrivals[startSize + i] =
                        std::min(filteredTrips[i].arrivalTime,
                                tdGraph.allSuffixMinArrivals[startSize + i + 1]);
                }
            }

            // E. Edge creation
            EdgeTripsHandle handle;
            handle.firstTripIndex = firstTripIdx;
            handle.tripCount = (uint32_t)filteredTrips.size();
            handle.firstSuffixIndex = firstSuffixIdx;
            handle.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, handle);
            edgeCount++;
        }

        // 6. PURE WALKING EDGES
        for (const auto& pair : minTransferTimes) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            const int walkTime = pair.second;

            EdgeTripsHandle handle;
            handle.firstTripIndex = 0;
            handle.tripCount = 0;
            handle.firstSuffixIndex = 0;
            handle.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, handle);
            edgeCount++;
        }

        std::cout << " done (" << edgeCount << " edges created)" << std::endl;

        // Print filtering statistics
        if (tdGraph.totalTripsBeforeFilter > 0) {
            double reductionPercent = 100.0 * (1.0 - (double)tdGraph.totalTripsAfterFilter / tdGraph.totalTripsBeforeFilter);
            std::cout << "Domination filtering: " << tdGraph.totalTripsBeforeFilter
                      << " -> " << tdGraph.totalTripsAfterFilter << " trips ("
                      << reductionPercent << "% reduction)" << std::endl;
        }

        return tdGraph;
    }

    // [USAGE] Returns the total number of vertices (stops) in the graph.
    inline size_t numVertices() const noexcept {
        return graph.numVertices();
    }

    // [USAGE] Returns an iterator/range for iterating over outgoing edges from vertex 'u'.
    inline auto edgesFrom(const Vertex u) const noexcept {
        return graph.edgesFrom(u);
    }

    // [USAGE] Returns the total number of edges in the graph.
    inline size_t numEdges() const noexcept {
        return graph.numEdges();
    }

    // [USAGE] Generic accessor for edge attributes.
    template<typename ATTRIBUTE>
    inline auto get(const ATTRIBUTE& attribute, const Edge edge) const noexcept {
        return graph.get(attribute, edge);
    }

    // [CORE LOGIC] Query logic
    // Calculates the earliest arrival time at the end of an edge given a departure time.
    //
    // [USAGE] Main relaxation function called by Dijkstra (runRelaxation) to evaluate edge weights.
    inline int getArrivalTime(const Edge edge, const int departureTime) const noexcept {
        const EdgeTripsHandle& h = graph.get(Function, edge);

        int minArrivalTime = never;

        auto begin = allDiscreteTrips.begin() + h.firstTripIndex;
        auto end = begin + h.tripCount;

        // 1. Binary Search for first valid departure
        auto it = std::lower_bound(begin, end, departureTime,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });

        if (it != end) {
            size_t localIdx = std::distance(begin, it);
            // 2. Use Suffix Minima optimization
            if (h.tripCount > 0) {
                minArrivalTime = allSuffixMinArrivals[h.firstSuffixIndex + localIdx];
            } else {
                minArrivalTime = it->arrivalTime;
            }
        }

        if (h.walkTime != never) {
            minArrivalTime = std::min(minArrivalTime, departureTime + h.walkTime);
        }
        return minArrivalTime;
    }

    // [USAGE] Helper called by Dijkstra to check pure walking connections.
    inline int getWalkArrivalFrom(const Edge edge, const int departureTime) const noexcept {
        const EdgeTripsHandle& h = graph.get(Function, edge);
        if (h.walkTime == never) return never;
        return departureTime + h.walkTime;
    }

    // [USAGE] Returns the minimum transfer time at a given stop vertex.
    inline int getMinTransferTimeAt(const Vertex u) const noexcept {
        return (size_t(u) < minTransferTimeByVertex.size()) ? minTransferTimeByVertex[u] : 0;
    }

    // [USAGE] Graph construction helper.
    inline Vertex addVertex() {
        return graph.addVertex();
    }

    // [USAGE] Graph construction helper.
    inline typename UnderlyingGraph::EdgeHandle addTimeDependentEdge(const Vertex from, const Vertex to, const EdgeTripsHandle& func) {
        typename UnderlyingGraph::EdgeHandle handle = graph.addEdge(from, to);
        handle.set(Function, func);
        return handle;
    }

    // [USAGE] Persists the flattened graph topology and schedule to disk.
    inline void serialize(const std::string& fileName) const noexcept {
        graph.writeBinary(fileName);
        IO::serialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips, allSuffixMinArrivals);
    }

    // [USAGE] Loads the flattened graph from disk.
    inline void deserialize(const std::string& fileName) noexcept {
        graph.readBinary(fileName);
        IO::deserialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips, allSuffixMinArrivals);
    }

    // [FACTORY] Loads a TimeDependentGraphClassic from a binary file on disk.
    inline static TimeDependentGraphClassic FromBinary(const std::string& fileName) noexcept {
        TimeDependentGraphClassic tdGraph;
        tdGraph.deserialize(fileName);
        return tdGraph;
    }

    // [STATISTICS] Get filtering statistics
    inline void printStatistics() const noexcept {
        std::cout << "=== TimeDependentGraphClassic Statistics ===" << std::endl;
        std::cout << "Vertices: " << numVertices() << std::endl;
        std::cout << "Edges: " << numEdges() << std::endl;
        std::cout << "Total trips (before filtering): " << totalTripsBeforeFilter << std::endl;
        std::cout << "Total trips (after filtering): " << totalTripsAfterFilter << std::endl;
        if (totalTripsBeforeFilter > 0) {
            double reductionPercent = 100.0 * (1.0 - (double)totalTripsAfterFilter / totalTripsBeforeFilter);
            std::cout << "Reduction: " << reductionPercent << "%" << std::endl;
        }
        std::cout << "============================================" << std::endl;
    }
};