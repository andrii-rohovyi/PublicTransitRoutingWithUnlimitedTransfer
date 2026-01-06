#pragma once

#include <vector>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <map>
#include <unordered_map>
#include <iostream>

#include "../../Helpers/Types.h"
#include "../../Helpers/Meta.h"
#include "../../DataStructures/Attributes/AttributeNames.h"
#include "../../DataStructures/Graph/Classes/DynamicGraph.h"
#include "../../Helpers/IO/Serialization.h"

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../DataStructures/Graph/TimeDependentGraphClassic.h"

using TransferGraph = ::TransferGraph;

// CST Data for a single vertex
struct CombinedSearchTreeData {
    // Sorted list of ALL unique departure times from all outgoing edges
    std::vector<int> schedule;

    // For each position i in schedule, stores startIndex for each edge
    // position_in_edge[scheduleIndex][edgeIndex] = startIndex into that edge's trips
    std::vector<std::vector<int>> positionInEdge;

    // List of edges with trips (in order matching positionInEdge)
    std::vector<Edge> edges;
    std::vector<Vertex> targets;

    // Walking-only edges (no trips, just walk time)
    std::vector<Edge> walkingEdges;
    std::vector<Vertex> walkingTargets;

    // Binary search on schedule
    inline int bisectLeft(int departureTime) const noexcept {
        if (schedule.empty()) return -1;
        auto it = std::lower_bound(schedule.begin(), schedule.end(), departureTime);
        if (it == schedule.end()) return -1;  // No valid departure
        return std::distance(schedule.begin(), it);
    }
};

class TimeDependentGraphCST {
public:
    using EdgeTripsHandle = ::EdgeTripsHandle;

private:
    using TDEdgeAttributes = Meta::List<
        ::Attribute<FromVertex, Vertex>,
        ::Attribute<ToVertex, Vertex>,
        ::Attribute<Valid, bool>,
        ::Attribute<IncomingEdgePointer, size_t>,
        ::Attribute<ReverseEdge, Edge>,
        ::Attribute<Function, EdgeTripsHandle>
    >;

    using TDVertexAttributes = Meta::List<
        ::Attribute<BeginOut, Edge>,
        ::Attribute<OutDegree, size_t>,
        ::Attribute<IncomingEdges, std::vector<Edge>>
    >;

    using UnderlyingGraph = DynamicGraphImplementation<TDVertexAttributes, TDEdgeAttributes>;

    UnderlyingGraph graph;
    std::vector<int> minTransferTimeByVertex;

public:
    struct TripLeg {
        int arrivalTime;
        Vertex stopId;
    };

    std::vector<DiscreteTrip> allDiscreteTrips;
    std::vector<int> allSuffixMinArrivals;

private:
    std::vector<uint32_t> tripOffsets;
    std::vector<TripLeg> allTripLegs;
    std::vector<CombinedSearchTreeData> cstData;

public:
    TimeDependentGraphCST() = default;

    inline bool getNextStop(const int tripId, const uint16_t currentStopIndex, Vertex& outStop, int& outArrival) const noexcept {
        if (tripId < 0 || (size_t)tripId + 1 >= tripOffsets.size()) return false;
        const uint32_t currentTripStart = tripOffsets[tripId];
        const uint32_t nextTripStart = tripOffsets[tripId + 1];
        const uint32_t absoluteIndex = currentTripStart + currentStopIndex + 1;
        if (absoluteIndex < nextTripStart) {
            const TripLeg& leg = allTripLegs[absoluteIndex];
            outStop = leg.stopId;
            outArrival = leg.arrivalTime;
            return true;
        }
        return false;
    }

    inline const TripLeg& getTripLeg(const size_t index) const noexcept {
        return allTripLegs[index];
    }

    inline size_t getNumStopEvents() const noexcept {
        return allTripLegs.size();
    }

    inline uint32_t getTripOffset(const int tripId) const noexcept {
        return tripOffsets[tripId];
    }

    inline const DiscreteTrip* getTripsBegin(const EdgeTripsHandle& h) const noexcept {
        return &allDiscreteTrips[h.firstTripIndex];
    }

    inline const DiscreteTrip* getTripsEnd(const EdgeTripsHandle& h) const noexcept {
        return &allDiscreteTrips[h.firstTripIndex + h.tripCount];
    }

    inline const int* getSuffixMinBegin(const EdgeTripsHandle& h) const noexcept {
        return &allSuffixMinArrivals[h.firstSuffixIndex];
    }

    inline const CombinedSearchTreeData& getCSTData(Vertex u) const noexcept {
        static const CombinedSearchTreeData emptyData;
        if (u >= cstData.size()) return emptyData;
        return cstData[u];
    }

    inline bool hasCSTData(Vertex u) const noexcept {
        return u < cstData.size() && !cstData[u].schedule.empty();
    }

    inline static TimeDependentGraphCST FromIntermediate(const Intermediate::Data& inter) noexcept {
        TimeDependentGraphCST tdGraph;
        const size_t numStops = inter.numberOfStops();
        const size_t numVertices = inter.transferGraph.numVertices();

        for (size_t i = 0; i < numVertices; ++i) {
            tdGraph.graph.addVertex();
        }

        tdGraph.minTransferTimeByVertex.assign(numVertices, 0);
        for (size_t s = 0; s < std::min(numStops, numVertices); ++s) {
            tdGraph.minTransferTimeByVertex[s] = inter.stops[s].minTransferTime;
        }

        std::unordered_map<std::pair<Vertex, Vertex>, std::vector<DiscreteTrip>, VertexPairHash> tripSegments;
        tripSegments.reserve(inter.trips.size() * 10);

        std::cout << "Building trip segments..." << std::flush;
        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];

            for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
                const Intermediate::StopEvent& stopEventU = trip.stopEvents[i];
                const Intermediate::StopEvent& stopEventV = trip.stopEvents[i + 1];
                const Vertex u = Vertex(stopEventU.stopId);
                const Vertex v = Vertex(stopEventV.stopId);

                const int buffer = (u < inter.stops.size()) ? inter.stops[u].minTransferTime : 0;

                tripSegments[{u, v}].emplace_back(DiscreteTrip{
                    .departureTime = stopEventU.departureTime - buffer,
                    .arrivalTime = stopEventV.arrivalTime,
                    .tripId = (int)tripId,
                    .departureStopIndex = (uint16_t)i
                });
            }
        }
        std::cout << " done." << std::endl;

        tdGraph.tripOffsets.reserve(inter.trips.size() + 1);

        size_t totalStops = 0;
        for (const auto& trip : inter.trips) {
            tdGraph.tripOffsets.push_back(totalStops);
            totalStops += trip.stopEvents.size();
        }

        tdGraph.tripOffsets.push_back(totalStops);
        tdGraph.allTripLegs.resize(totalStops);

        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];
            const size_t baseOffset = tdGraph.tripOffsets[tripId];

            for (size_t i = 0; i < trip.stopEvents.size(); ++i) {
                tdGraph.allTripLegs[baseOffset + i] = {
                    trip.stopEvents[i].arrivalTime,
                    Vertex(trip.stopEvents[i].stopId)
                };
            }
        }

        std::unordered_map<std::pair<Vertex, Vertex>, int, VertexPairHash> minTransferTimes;
        minTransferTimes.reserve(inter.transferGraph.numEdges());

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

        std::cout << "Creating time-dependent edges with domination filtering..." << std::flush;
        size_t edgeCount = 0;

        tdGraph.allDiscreteTrips.reserve(tripSegments.size() * 5);
        tdGraph.allSuffixMinArrivals.reserve(tripSegments.size() * 5);

        size_t totalTripsBeforeFilter = 0;
        size_t totalTripsAfterFilter = 0;

        for (auto& pair : tripSegments) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            std::vector<DiscreteTrip>& trips = pair.second;

            auto transferIt = minTransferTimes.find({u, v});
            int walkTime = (transferIt != minTransferTimes.end()) ? transferIt->second : never;
            if (transferIt != minTransferTimes.end()) {
                minTransferTimes.erase(transferIt);
            }

            std::sort(trips.begin(), trips.end());

            totalTripsBeforeFilter += trips.size();

            std::vector<DiscreteTrip> filteredTrips = TimeDependentGraphClassic::filterDominatedConnections(trips, walkTime);

            totalTripsAfterFilter += filteredTrips.size();

            if (filteredTrips.empty() && walkTime == never) {
                continue;
            }

            uint32_t firstTripIdx = tdGraph.allDiscreteTrips.size();
            uint32_t firstSuffixIdx = tdGraph.allSuffixMinArrivals.size();

            tdGraph.allDiscreteTrips.insert(tdGraph.allDiscreteTrips.end(), filteredTrips.begin(), filteredTrips.end());

            size_t startSize = tdGraph.allSuffixMinArrivals.size();
            tdGraph.allSuffixMinArrivals.resize(startSize + filteredTrips.size());
            if (!filteredTrips.empty()) {
                tdGraph.allSuffixMinArrivals.back() = filteredTrips.back().arrivalTime;
                for (int i = int(filteredTrips.size()) - 2; i >= 0; --i) {
                    tdGraph.allSuffixMinArrivals[startSize + i] =
                        std::min(filteredTrips[i].arrivalTime, tdGraph.allSuffixMinArrivals[startSize + i + 1]);
                }
            }

            EdgeTripsHandle handle;
            handle.firstTripIndex = firstTripIdx;
            handle.tripCount = (uint32_t)filteredTrips.size();
            handle.firstSuffixIndex = firstSuffixIdx;
            handle.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, handle);
            edgeCount++;
        }

        std::cout << " done (" << edgeCount << " edges created)" << std::endl;

        if (totalTripsBeforeFilter > 0) {
            double reductionPercent = 100.0 * (1.0 - (double)totalTripsAfterFilter / totalTripsBeforeFilter);
            std::cout << "Domination filtering: " << totalTripsBeforeFilter
                      << " -> " << totalTripsAfterFilter << " trips ("
                      << reductionPercent << "% reduction)" << std::endl;
        }

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

        std::cout << "Building Combined Search Tree structures..." << std::flush;
        tdGraph.buildCST();
        std::cout << " done." << std::endl;

        std::cout << "Verifying CST structures..." << std::flush;
        tdGraph.verifyCSTStructure();
        std::cout << " done." << std::endl;

        return tdGraph;
    }

    inline void buildCST() noexcept {
        cstData.resize(graph.numVertices());

        for (size_t i = 0; i < graph.numVertices(); ++i) {
            buildCSTForVertex(Vertex(i));
        }
    }

    inline void verifyCSTStructure() const noexcept {
        size_t errorCount = 0;
        size_t checkedVertices = 0;

        for (size_t u = 0; u < cstData.size(); ++u) {
            const CombinedSearchTreeData& cst = cstData[u];
            if (cst.edges.empty()) continue;

            checkedVertices++;

            // Verify each edge
            for (size_t edgeIdx = 0; edgeIdx < cst.edges.size(); ++edgeIdx) {
                const Edge e = cst.edges[edgeIdx];
                const EdgeTripsHandle& h = graph.get(Function, e);

                // Check that positionInEdge is consistent
                for (size_t schedIdx = 0; schedIdx < cst.schedule.size(); ++schedIdx) {
                    int startIndex = cst.positionInEdge[schedIdx][edgeIdx];
                    int depTime = cst.schedule[schedIdx];

                    // Verify: startIndex should be lower_bound position
                    const DiscreteTrip* begin = getTripsBegin(h);
                    const DiscreteTrip* end = getTripsEnd(h);
                    auto it = std::lower_bound(begin, end, depTime,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });
                    int expectedIndex = std::distance(begin, it);

                    if (startIndex != expectedIndex) {
                        if (errorCount < 10) {
                            std::cout << "\n[CST VERIFY ERROR] Vertex " << u << " edge " << edgeIdx
                                      << " schedIdx " << schedIdx << " (dep=" << depTime << ")"
                                      << ": stored=" << startIndex << " expected=" << expectedIndex << std::endl;
                        }
                        errorCount++;
                    }
                }
            }
        }

        std::cout << "\nCST Verification: checked " << checkedVertices << " vertices, found " << errorCount << " errors" << std::endl;
    }

private:
    inline void buildCSTForVertex(Vertex u) noexcept {
        CombinedSearchTreeData& cst = cstData[u];

        // Collect all edges and their departure times
        struct EdgeInfo {
            Edge edgeId;
            Vertex target;
            std::vector<int> departureTimes;
        };

        std::vector<EdgeInfo> edgeInfos;
        std::set<int> allDepartures;  // Collect unique departure times

        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            const EdgeTripsHandle& h = graph.get(Function, e);

            if (h.tripCount > 0) {
                EdgeInfo info;
                info.edgeId = e;
                info.target = v;

                const DiscreteTrip* begin = getTripsBegin(h);
                const DiscreteTrip* end = getTripsEnd(h);
                for (const DiscreteTrip* it = begin; it != end; ++it) {
                    info.departureTimes.push_back(it->departureTime);
                    allDepartures.insert(it->departureTime);
                }

                edgeInfos.push_back(std::move(info));
            } else if (h.walkTime != never) {
                cst.walkingTargets.push_back(v);
                cst.walkingEdges.push_back(e);
            }
        }

        if (edgeInfos.empty()) return;

        // Build sorted schedule from all unique departures
        cst.schedule.assign(allDepartures.begin(), allDepartures.end());

        // Store edges and targets
        for (const auto& info : edgeInfos) {
            cst.edges.push_back(info.edgeId);
            cst.targets.push_back(info.target);
        }

        // Build positionInEdge: for each schedule position, store startIndex for each edge
        const size_t numEdges = edgeInfos.size();
        cst.positionInEdge.resize(cst.schedule.size());

        for (size_t schedIdx = 0; schedIdx < cst.schedule.size(); ++schedIdx) {
            int depTime = cst.schedule[schedIdx];
            cst.positionInEdge[schedIdx].resize(numEdges);

            for (size_t edgeIdx = 0; edgeIdx < numEdges; ++edgeIdx) {
                // Binary search to find startIndex for this edge
                const auto& departures = edgeInfos[edgeIdx].departureTimes;
                auto it = std::lower_bound(departures.begin(), departures.end(), depTime);
                cst.positionInEdge[schedIdx][edgeIdx] = std::distance(departures.begin(), it);
            }
        }
    }

public:
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

    inline int getArrivalTime(const Edge edge, const int departureTime) const noexcept {
        const EdgeTripsHandle& h = graph.get(Function, edge);

        int minArrivalTime = never;

        auto begin = allDiscreteTrips.begin() + h.firstTripIndex;
        auto end = begin + h.tripCount;

        auto it = std::lower_bound(begin, end, departureTime,
            [](const DiscreteTrip& trip, int time) {
                return trip.departureTime < time;
            });

        if (it != end) {
            size_t localIdx = std::distance(begin, it);
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

    inline int getWalkArrivalFrom(const Edge edge, const int departureTime) const noexcept {
        const EdgeTripsHandle& h = graph.get(Function, edge);
        if (h.walkTime == never) return never;
        return departureTime + h.walkTime;
    }

    inline int getMinTransferTimeAt(const Vertex u) const noexcept {
        return (size_t(u) < minTransferTimeByVertex.size()) ? minTransferTimeByVertex[u] : 0;
    }

    inline Vertex addVertex() {
        return graph.addVertex();
    }

    inline typename UnderlyingGraph::EdgeHandle addTimeDependentEdge(const Vertex from, const Vertex to, const EdgeTripsHandle& func) {
        typename UnderlyingGraph::EdgeHandle handle = graph.addEdge(from, to);
        handle.set(Function, func);
        return handle;
    }

    inline void printStatistics() const noexcept {
        std::cout << "=== TimeDependentGraphCST Statistics ===" << std::endl;
        std::cout << "Vertices: " << numVertices() << std::endl;
        std::cout << "Edges: " << numEdges() << std::endl;

        size_t totalCSTVertices = 0;
        size_t totalScheduleSize = 0;
        size_t totalPositionEntries = 0;
        for (const auto& cst : cstData) {
            if (!cst.schedule.empty()) {
                totalCSTVertices++;
                totalScheduleSize += cst.schedule.size();
                totalPositionEntries += cst.schedule.size() * cst.edges.size();
            }
        }

        std::cout << "Vertices with CST data: " << totalCSTVertices << std::endl;
        if (totalCSTVertices > 0) {
            std::cout << "Average schedule size per vertex: " << (double)totalScheduleSize / totalCSTVertices << std::endl;
            std::cout << "Total position entries: " << totalPositionEntries << std::endl;
        }
        std::cout << "==========================================" << std::endl;
    }
};