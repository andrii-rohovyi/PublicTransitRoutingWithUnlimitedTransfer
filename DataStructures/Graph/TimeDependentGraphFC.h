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

struct FCPointer {
    int startIndex;
    int nextLoc;
};

struct FractionalCascadingData {
    std::vector<std::vector<int>> m_arr;
    std::vector<std::vector<int>> arr;
    std::vector<std::vector<FCPointer>> pointers;
    std::vector<Vertex> reachableNodes;
    std::vector<Edge> reachableEdges;  // NEW: Store Edge IDs directly
    std::vector<Vertex> walkingNodes;
    std::vector<Edge> walkingEdges;    // NEW: Store Edge IDs for walking

    inline int bisectLeft(int departureTime) const noexcept {
        if (m_arr.empty() || m_arr[0].empty()) return 0;
        const auto& firstLevel = m_arr[0];
        auto it = std::lower_bound(firstLevel.begin(), firstLevel.end(), departureTime);
        return std::distance(firstLevel.begin(), it);
    }
};

class TimeDependentGraphFC {
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
    std::vector<FractionalCascadingData> fcData;

public:
    TimeDependentGraphFC() = default;

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

    struct FoundTrip {
        int tripId = -1;
    };

    inline FoundTrip findMatchingTrip(Vertex u, Vertex v, int minDepTime, int atArrTime) const noexcept {
        for (const Edge e : graph.edgesFrom(u)) {
            if (graph.get(ToVertex, e) == v) {
                const EdgeTripsHandle& h = graph.get(Function, e);
                const DiscreteTrip* begin = getTripsBegin(h);
                const DiscreteTrip* end = getTripsEnd(h);

                auto it = std::lower_bound(begin, end, minDepTime,
                    [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

                for (; it != end; ++it) {
                    if (it->arrivalTime == atArrTime) {
                         return {it->tripId};
                    }
                }
            }
        }
        return {-1};
    }

    inline const FractionalCascadingData& getFCData(Vertex u) const noexcept {
        static const FractionalCascadingData emptyData;
        if (u >= fcData.size()) return emptyData;
        return fcData[u];
    }

    inline bool hasFCData(Vertex u) const noexcept {
        return u < fcData.size() && !fcData[u].m_arr.empty();
    }

    inline static TimeDependentGraphFC FromIntermediate(const Intermediate::Data& inter) noexcept {
        TimeDependentGraphFC tdGraph;
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

        std::cout << "Building fractional cascading structures..." << std::flush;
        tdGraph.buildFractionalCascading();
        std::cout << " done." << std::endl;

        // VERIFY FC STRUCTURE
        std::cout << "Verifying fractional cascading structures..." << std::flush;
        tdGraph.verifyFCStructure();
        std::cout << " done." << std::endl;

        return tdGraph;
    }

    inline void buildFractionalCascading() noexcept {
        fcData.resize(graph.numVertices());

        for (size_t i = 0; i < graph.numVertices(); ++i) {
            buildFractionalCascadingForVertex(Vertex(i));
        }
    }

    // VERIFICATION: Check that FC structure matches actual edge data
    inline void verifyFCStructure() const noexcept {
        size_t errorCount = 0;
        size_t checkedVertices = 0;

        for (size_t u = 0; u < fcData.size(); ++u) {
            const FractionalCascadingData& fc = fcData[u];
            if (fc.reachableNodes.empty()) continue;

            checkedVertices++;

            // Verify each reachable node using stored Edge ID
            for (size_t level = 0; level < fc.reachableNodes.size(); ++level) {
                const Vertex v = fc.reachableNodes[level];
                const Edge e = fc.reachableEdges[level];  // Use stored Edge ID

                if (e == noEdge) {
                    if (errorCount < 10) {
                        std::cout << "\n[FC VERIFY ERROR] Vertex " << u << " level " << level
                                  << ": reachableNode " << v << " has no edge!" << std::endl;
                    }
                    errorCount++;
                    continue;
                }

                const EdgeTripsHandle& h = graph.get(Function, e);

                // Check that arr[level] matches the edge's trips
                if (fc.arr[level].size() != h.tripCount) {
                    if (errorCount < 10) {
                        std::cout << "\n[FC VERIFY ERROR] Vertex " << u << " -> " << v << " (level " << level << ")"
                                  << ": arr size=" << fc.arr[level].size()
                                  << " but edge tripCount=" << h.tripCount << std::endl;

                        std::cout << "  arr[" << level << "]: ";
                        for (size_t k = 0; k < std::min(fc.arr[level].size(), (size_t)5); ++k) {
                            std::cout << fc.arr[level][k] << " ";
                        }
                        std::cout << std::endl;

                        std::cout << "  edge trips: ";
                        const DiscreteTrip* trips = getTripsBegin(h);
                        for (size_t k = 0; k < std::min((size_t)h.tripCount, (size_t)5); ++k) {
                            std::cout << trips[k].departureTime << " ";
                        }
                        std::cout << std::endl;
                    }
                    errorCount++;
                    continue;
                }

                // Check that departure times match
                const DiscreteTrip* trips = getTripsBegin(h);
                for (size_t k = 0; k < fc.arr[level].size(); ++k) {
                    if (fc.arr[level][k] != trips[k].departureTime) {
                        if (errorCount < 10) {
                            std::cout << "\n[FC VERIFY ERROR] Vertex " << u << " -> " << v << " (level " << level << ")"
                                      << ": arr[" << k << "]=" << fc.arr[level][k]
                                      << " but trip[" << k << "].dep=" << trips[k].departureTime << std::endl;
                        }
                        errorCount++;
                        break;
                    }
                }
            }
        }

        std::cout << "\nFC Verification: checked " << checkedVertices << " vertices, found " << errorCount << " errors" << std::endl;
    }

private:
    inline void buildFractionalCascadingForVertex(Vertex u) noexcept {
        FractionalCascadingData& fc = fcData[u];

        struct EdgeInfo {
            Vertex target;
            Edge edgeId;  // Store the Edge ID
            std::vector<int> departureTimes;
            size_t tripCount;
        };

        std::vector<EdgeInfo> edgeInfos;

        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            const EdgeTripsHandle& h = graph.get(Function, e);

            if (h.tripCount > 0) {
                EdgeInfo info;
                info.target = v;
                info.edgeId = e;  // Store the Edge ID
                info.tripCount = h.tripCount;

                const DiscreteTrip* begin = getTripsBegin(h);
                const DiscreteTrip* end = getTripsEnd(h);
                for (const DiscreteTrip* it = begin; it != end; ++it) {
                    info.departureTimes.push_back(it->departureTime);
                }

                edgeInfos.push_back(std::move(info));
            } else if (h.walkTime != never) {
                fc.walkingNodes.push_back(v);
                fc.walkingEdges.push_back(e);  // Store walking Edge ID
            }
        }

        if (edgeInfos.empty()) return;

        // Sort by number of departures (ascending - Python strategy)
        std::sort(edgeInfos.begin(), edgeInfos.end(),
            [](const EdgeInfo& a, const EdgeInfo& b) { return a.tripCount < b.tripCount; });

        // Build cascaded arrays
        std::vector<std::vector<int>> m_arr_temp;
        std::vector<std::vector<int>> arr_temp;
        std::vector<Vertex> reachable_temp;
        std::vector<Edge> edges_temp;

        for (size_t i = 0; i < edgeInfos.size(); ++i) {
            arr_temp.push_back(edgeInfos[i].departureTimes);
            reachable_temp.push_back(edgeInfos[i].target);
            edges_temp.push_back(edgeInfos[i].edgeId);

            std::vector<int> cascaded;

            if (i == 0) {
                cascaded.push_back(-1);
                cascaded.insert(cascaded.end(), edgeInfos[i].departureTimes.begin(), edgeInfos[i].departureTimes.end());
                cascaded.push_back(1000000000);
            } else {
                cascaded = edgeInfos[i].departureTimes;

                for (size_t k = 1; k < m_arr_temp[i - 1].size(); k += 2) {
                    cascaded.push_back(m_arr_temp[i - 1][k]);
                }

                std::sort(cascaded.begin(), cascaded.end());
                cascaded.erase(std::unique(cascaded.begin(), cascaded.end()), cascaded.end());

                cascaded.insert(cascaded.begin(), -1);
                cascaded.push_back(1000000000);
            }

            m_arr_temp.push_back(std::move(cascaded));
        }

        // Reverse to match Python's order
        std::reverse(m_arr_temp.begin(), m_arr_temp.end());
        std::reverse(arr_temp.begin(), arr_temp.end());
        std::reverse(reachable_temp.begin(), reachable_temp.end());
        std::reverse(edges_temp.begin(), edges_temp.end());

        fc.m_arr = std::move(m_arr_temp);
        fc.arr = std::move(arr_temp);
        fc.reachableNodes = std::move(reachable_temp);
        fc.reachableEdges = std::move(edges_temp);

        buildPointersForVertex(fc);
    }

    inline void buildPointersForVertex(FractionalCascadingData& fc) noexcept {
        fc.pointers.resize(fc.m_arr.size());

        for (size_t i = 0; i < fc.m_arr.size(); ++i) {
            fc.pointers[i].resize(fc.m_arr[i].size());

            for (size_t j = 0; j < fc.m_arr[i].size(); ++j) {
                FCPointer ptr;

                ptr.startIndex = std::lower_bound(fc.arr[i].begin(), fc.arr[i].end(), fc.m_arr[i][j])
                                 - fc.arr[i].begin();

                if (i == fc.m_arr.size() - 1) {
                    ptr.nextLoc = 0;
                } else {
                    ptr.nextLoc = std::lower_bound(fc.m_arr[i + 1].begin(), fc.m_arr[i + 1].end(), fc.m_arr[i][j])
                                  - fc.m_arr[i + 1].begin();
                }

                fc.pointers[i][j] = ptr;
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

    inline void serialize(const std::string& fileName) const noexcept {
        graph.writeBinary(fileName);
        IO::serialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips, allSuffixMinArrivals);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        graph.readBinary(fileName);
        IO::deserialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips, allSuffixMinArrivals);
    }

    inline static TimeDependentGraphFC FromBinary(const std::string& fileName) noexcept {
        TimeDependentGraphFC tdGraph;
        tdGraph.deserialize(fileName);
        return tdGraph;
    }

    inline void printStatistics() const noexcept {
        std::cout << "=== TimeDependentGraphFC Statistics ===" << std::endl;
        std::cout << "Vertices: " << numVertices() << std::endl;
        std::cout << "Edges: " << numEdges() << std::endl;

        size_t totalFCVertices = 0;
        size_t totalFCLevels = 0;
        for (const auto& fc : fcData) {
            if (!fc.m_arr.empty()) {
                totalFCVertices++;
                totalFCLevels += fc.m_arr.size();
            }
        }

        std::cout << "Vertices with FC data: " << totalFCVertices << std::endl;
        if (totalFCVertices > 0) {
            std::cout << "Average FC levels per vertex: " << (double)totalFCLevels / totalFCVertices << std::endl;
        }
        std::cout << "==========================================" << std::endl;
    }
};