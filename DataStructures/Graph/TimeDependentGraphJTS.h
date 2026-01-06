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
#include "TimeDependentGraph.h"

using TransferGraph = ::TransferGraph;

class JTSGraph {
public:
    using EdgeTripsHandle = ::EdgeTripsHandle;

    struct TripDepartureFromStop {
        int tripId;
        uint16_t stopIndex;
        int departureTime;
        int arrivalTimeAtNextStop;
        int minArrivalTime;
    };

    struct RouteAtStop {
        int routeId;
        uint16_t stopIndexInRoute;
        uint32_t firstTripIndex;
        uint32_t tripCount;
    };

    struct RouteTripInfo {
        int tripId;
        int departureTime;
        int arrivalTimeAtNextStop;
        int minArrivalTime;
    };

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
    std::vector<std::vector<TripDepartureFromStop>> tripsFromStopIndex;
    std::vector<std::vector<RouteAtStop>> routesFromStopIndex;
    std::vector<RouteTripInfo> allRouteTrips;

    size_t totalTripsBeforeFilter = 0;
    size_t totalTripsAfterFilter = 0;
    size_t totalRoutes = 0;

public:
    JTSGraph() = default;

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

    inline const std::vector<TripDepartureFromStop>& getTripsFromStop(const Vertex stop) const noexcept {
        static const std::vector<TripDepartureFromStop> empty;
        if (stop >= tripsFromStopIndex.size()) return empty;
        return tripsFromStopIndex[stop];
    }

    inline const std::vector<RouteAtStop>& getRoutesFromStop(const Vertex stop) const noexcept {
        static const std::vector<RouteAtStop> empty;
        if (stop >= routesFromStopIndex.size()) return empty;
        return routesFromStopIndex[stop];
    }

    inline const RouteTripInfo* getRouteTripsBegin(const RouteAtStop& routeAtStop) const noexcept {
        return &allRouteTrips[routeAtStop.firstTripIndex];
    }

    inline const RouteTripInfo* getRouteTripsEnd(const RouteAtStop& routeAtStop) const noexcept {
        return &allRouteTrips[routeAtStop.firstTripIndex + routeAtStop.tripCount];
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

    inline static JTSGraph FromIntermediate(const Intermediate::Data& inter) noexcept {
        JTSGraph tdGraph;
        const size_t numStops = inter.numberOfStops();
        const size_t numVertices = inter.transferGraph.numVertices();

        for (size_t i = 0; i < numVertices; ++i) {
            tdGraph.graph.addVertex();
        }

        tdGraph.minTransferTimeByVertex.assign(numVertices, 0);
        for (size_t s = 0; s < std::min(numStops, numVertices); ++s) {
            tdGraph.minTransferTimeByVertex[s] = inter.stops[s].minTransferTime;
        }

        // Build trip segments for edges (no filtering - store all trips)
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

                tdGraph.totalTripsBeforeFilter++;
            }
        }
        std::cout << " done." << std::endl;

        // Flatten trip legs
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

        // Build stop-to-trips index
        std::cout << "Building stop-to-trips index..." << std::flush;
        tdGraph.tripsFromStopIndex.resize(numVertices);

        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];

            std::vector<int> suffixMinArrival(trip.stopEvents.size());
            suffixMinArrival.back() = trip.stopEvents.back().arrivalTime;
            for (int i = (int)trip.stopEvents.size() - 2; i >= 0; --i) {
                suffixMinArrival[i] = std::min(trip.stopEvents[i].arrivalTime, suffixMinArrival[i + 1]);
            }

            for (size_t i = 0; i + 1 < trip.stopEvents.size(); ++i) {
                const Vertex stopVertex = Vertex(trip.stopEvents[i].stopId);
                const int buffer = (stopVertex < inter.stops.size()) ? inter.stops[stopVertex].minTransferTime : 0;

                tdGraph.tripsFromStopIndex[stopVertex].push_back({
                    .tripId = (int)tripId,
                    .stopIndex = (uint16_t)i,
                    .departureTime = trip.stopEvents[i].departureTime - buffer,
                    .arrivalTimeAtNextStop = trip.stopEvents[i + 1].arrivalTime,
                    .minArrivalTime = suffixMinArrival[i + 1]
                });
            }
        }

        for (auto& trips : tdGraph.tripsFromStopIndex) {
            std::sort(trips.begin(), trips.end(),
                [](const TripDepartureFromStop& a, const TripDepartureFromStop& b) {
                    return a.departureTime < b.departureTime;
                });
        }
        std::cout << " done." << std::endl;

        // Build route-based index
        std::cout << "Building route-based index..." << std::flush;

        struct StopSequenceHash {
            size_t operator()(const std::vector<Vertex>& stops) const {
                size_t hash = 0;
                for (const Vertex& s : stops) {
                    hash ^= std::hash<size_t>()(s) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
                }
                return hash;
            }
        };

        std::unordered_map<std::vector<Vertex>, int, StopSequenceHash> stopSequenceToRouteId;
        std::vector<std::vector<int>> routeToTrips;

        for (size_t tripId = 0; tripId < inter.trips.size(); ++tripId) {
            const Intermediate::Trip& trip = inter.trips[tripId];

            std::vector<Vertex> stopSequence;
            stopSequence.reserve(trip.stopEvents.size());
            for (const auto& se : trip.stopEvents) {
                stopSequence.push_back(Vertex(se.stopId));
            }

            auto it = stopSequenceToRouteId.find(stopSequence);
            int routeId;
            if (it == stopSequenceToRouteId.end()) {
                routeId = (int)routeToTrips.size();
                stopSequenceToRouteId[stopSequence] = routeId;
                routeToTrips.push_back({});
            } else {
                routeId = it->second;
            }
            routeToTrips[routeId].push_back((int)tripId);
        }

        tdGraph.totalRoutes = routeToTrips.size();
        tdGraph.routesFromStopIndex.resize(numVertices);

        std::map<std::pair<Vertex, int>, std::vector<RouteTripInfo>> stopRouteTrips;

        // Change RouteAtStop to be built inline during the first loop:
for (int routeId = 0; routeId < (int)routeToTrips.size(); ++routeId) {
    const std::vector<int>& tripIds = routeToTrips[routeId];
    if (tripIds.empty()) continue;

    const Intermediate::Trip& firstTrip = inter.trips[tripIds[0]];

    for (size_t stopIdx = 0; stopIdx + 1 < firstTrip.stopEvents.size(); ++stopIdx) {
        const Vertex stopVertex = Vertex(firstTrip.stopEvents[stopIdx].stopId);

        std::vector<RouteTripInfo> tripInfos;

        for (int tripId : tripIds) {
            const Intermediate::Trip& trip = inter.trips[tripId];

            std::vector<int> suffixMinArrival(trip.stopEvents.size());
            suffixMinArrival.back() = trip.stopEvents.back().arrivalTime;
            for (int i = (int)trip.stopEvents.size() - 2; i >= 0; --i) {
                suffixMinArrival[i] = std::min(trip.stopEvents[i].arrivalTime, suffixMinArrival[i + 1]);
            }

            const int buffer = (stopVertex < inter.stops.size()) ? inter.stops[stopVertex].minTransferTime : 0;

            tripInfos.push_back({
                .tripId = tripId,
                .departureTime = trip.stopEvents[stopIdx].departureTime - buffer,
                .arrivalTimeAtNextStop = trip.stopEvents[stopIdx + 1].arrivalTime,
                .minArrivalTime = suffixMinArrival[stopIdx + 1]
            });
        }

        std::sort(tripInfos.begin(), tripInfos.end(),
            [](const RouteTripInfo& a, const RouteTripInfo& b) {
                if (a.departureTime != b.departureTime)
                    return a.departureTime < b.departureTime;
                return a.arrivalTimeAtNextStop < b.arrivalTimeAtNextStop;
            });

        uint32_t firstTripIdx = tdGraph.allRouteTrips.size();
        tdGraph.allRouteTrips.insert(tdGraph.allRouteTrips.end(), tripInfos.begin(), tripInfos.end());

        tdGraph.routesFromStopIndex[stopVertex].push_back({
            .routeId = routeId,
            .stopIndexInRoute = (uint16_t)stopIdx,  // USE stopIdx DIRECTLY!
            .firstTripIndex = firstTripIdx,
            .tripCount = (uint32_t)tripInfos.size()
        });
    }
}

        for (auto& entry : stopRouteTrips) {
            const Vertex stopVertex = entry.first.first;
            const int routeId = entry.first.second;
            std::vector<RouteTripInfo>& tripInfos = entry.second;

            if (tripInfos.empty()) continue;

            const Intermediate::Trip& firstTrip = inter.trips[routeToTrips[routeId][0]];
            uint16_t stopIndexInRoute = 0;
            for (size_t i = 0; i < firstTrip.stopEvents.size(); ++i) {
                if (Vertex(firstTrip.stopEvents[i].stopId) == stopVertex) {
                    stopIndexInRoute = (uint16_t)i;
                    break;
                }
            }

            uint32_t firstTripIdx = tdGraph.allRouteTrips.size();
            tdGraph.allRouteTrips.insert(tdGraph.allRouteTrips.end(), tripInfos.begin(), tripInfos.end());

            tdGraph.routesFromStopIndex[stopVertex].push_back({
                .routeId = routeId,
                .stopIndexInRoute = stopIndexInRoute,
                .firstTripIndex = firstTripIdx,
                .tripCount = (uint32_t)tripInfos.size()
            });
        }

        std::cout << " done (" << tdGraph.totalRoutes << " routes)" << std::endl;

        // Transfer graph processing
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

        // Create edges WITHOUT dominated connection filtering
        std::cout << "Creating time-dependent edges..." << std::flush;
        size_t edgeCount = 0;

        tdGraph.allDiscreteTrips.reserve(tripSegments.size() * 5);
        tdGraph.allSuffixMinArrivals.reserve(tripSegments.size() * 5);

        for (auto& pair : tripSegments) {
            const Vertex u = pair.first.first;
            const Vertex v = pair.first.second;
            std::vector<DiscreteTrip>& trips = pair.second;

            auto transferIt = minTransferTimes.find({u, v});
            int walkTime = (transferIt != minTransferTimes.end()) ? transferIt->second : never;
            if (transferIt != minTransferTimes.end()) {
                minTransferTimes.erase(transferIt);
            }

            // Sort by departure time (required for binary search)
            // Sort by departure time, then by arrival time (earlier arrival first for ties)
            std::sort(trips.begin(), trips.end(),
                [](const DiscreteTrip& a, const DiscreteTrip& b) {
                    if (a.departureTime != b.departureTime)
                        return a.departureTime < b.departureTime;
                    return a.arrivalTime < b.arrivalTime;
                });

            // NO FILTERING - keep all trips
            tdGraph.totalTripsAfterFilter += trips.size();

            if (trips.empty() && walkTime == never) {
                continue;
            }

            uint32_t firstTripIdx = tdGraph.allDiscreteTrips.size();
            uint32_t firstSuffixIdx = tdGraph.allSuffixMinArrivals.size();

            tdGraph.allDiscreteTrips.insert(tdGraph.allDiscreteTrips.end(), trips.begin(), trips.end());

            // Build suffix minima
            size_t startSize = tdGraph.allSuffixMinArrivals.size();
            tdGraph.allSuffixMinArrivals.resize(startSize + trips.size());
            if (!trips.empty()) {
                tdGraph.allSuffixMinArrivals[startSize + trips.size() - 1] = trips.back().arrivalTime;
                for (int i = int(trips.size()) - 2; i >= 0; --i) {
                    tdGraph.allSuffixMinArrivals[startSize + i] =
                        std::min(trips[i].arrivalTime, tdGraph.allSuffixMinArrivals[startSize + i + 1]);
                }
            }

            EdgeTripsHandle handle;
            handle.firstTripIndex = firstTripIdx;
            handle.tripCount = (uint32_t)trips.size();
            handle.firstSuffixIndex = firstSuffixIdx;
            handle.walkTime = walkTime;

            tdGraph.addTimeDependentEdge(u, v, handle);
            edgeCount++;
        }

        // Pure walking edges
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

        // Statistics
        std::cout << "Total trips stored: " << tdGraph.totalTripsAfterFilter << std::endl;

        size_t totalTripDepartures = 0;
        size_t maxTripsFromStop = 0;
        for (const auto& trips : tdGraph.tripsFromStopIndex) {
            totalTripDepartures += trips.size();
            maxTripsFromStop = std::max(maxTripsFromStop, trips.size());
        }
        std::cout << "Stop-to-trips index: " << totalTripDepartures << " total departures, "
                  << "max " << maxTripsFromStop << " from single stop" << std::endl;

        size_t totalRouteStops = 0;
        size_t maxRoutesFromStop = 0;
        for (const auto& routes : tdGraph.routesFromStopIndex) {
            totalRouteStops += routes.size();
            maxRoutesFromStop = std::max(maxRoutesFromStop, routes.size());
        }
        std::cout << "Route-based index: " << totalRouteStops << " (route, stop) pairs, "
                  << "max " << maxRoutesFromStop << " routes from single stop" << std::endl;

        return tdGraph;
    }

    inline size_t numVertices() const noexcept { return graph.numVertices(); }
    inline auto edgesFrom(const Vertex u) const noexcept { return graph.edgesFrom(u); }
    inline size_t numEdges() const noexcept { return graph.numEdges(); }
    inline size_t numRoutes() const noexcept { return totalRoutes; }

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
            [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

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

    inline Vertex addVertex() { return graph.addVertex(); }

    inline typename UnderlyingGraph::EdgeHandle addTimeDependentEdge(const Vertex from, const Vertex to, const EdgeTripsHandle& func) {
        typename UnderlyingGraph::EdgeHandle handle = graph.addEdge(from, to);
        handle.set(Function, func);
        return handle;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        graph.writeBinary(fileName);
        IO::serialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips,
                      allSuffixMinArrivals, minTransferTimeByVertex);
        IO::serialize(fileName + ".stops", tripsFromStopIndex);
        IO::serialize(fileName + ".routes", routesFromStopIndex, allRouteTrips, totalRoutes);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        graph.readBinary(fileName);
        IO::deserialize(fileName + ".data", tripOffsets, allTripLegs, allDiscreteTrips,
                        allSuffixMinArrivals, minTransferTimeByVertex);
        IO::deserialize(fileName + ".stops", tripsFromStopIndex);
        IO::deserialize(fileName + ".routes", routesFromStopIndex, allRouteTrips, totalRoutes);
    }

    inline static JTSGraph FromBinary(const std::string& fileName) noexcept {
        JTSGraph tdGraph;
        tdGraph.deserialize(fileName);
        return tdGraph;
    }

    inline void printStatistics() const noexcept {
        std::cout << "=== JTSGraph Statistics ===" << std::endl;
        std::cout << "Vertices: " << numVertices() << std::endl;
        std::cout << "Edges: " << numEdges() << std::endl;
        std::cout << "Routes: " << totalRoutes << std::endl;
        std::cout << "Total discrete trips: " << allDiscreteTrips.size() << std::endl;
        std::cout << "Total trip legs: " << allTripLegs.size() << std::endl;

        size_t totalTripDepartures = 0;
        size_t maxTripsFromStop = 0;
        size_t stopsWithTrips = 0;
        for (const auto& trips : tripsFromStopIndex) {
            totalTripDepartures += trips.size();
            maxTripsFromStop = std::max(maxTripsFromStop, trips.size());
            if (!trips.empty()) stopsWithTrips++;
        }
        std::cout << "Stops with departures: " << stopsWithTrips << std::endl;
        std::cout << "Total trip departures (stop index): " << totalTripDepartures << std::endl;
        std::cout << "Max departures from single stop: " << maxTripsFromStop << std::endl;

        size_t totalRouteStops = 0;
        size_t maxRoutesFromStop = 0;
        for (const auto& routes : routesFromStopIndex) {
            totalRouteStops += routes.size();
            maxRoutesFromStop = std::max(maxRoutesFromStop, routes.size());
        }
        std::cout << "Route-based index: " << totalRouteStops << " (route, stop) pairs" << std::endl;
        std::cout << "Max routes from single stop: " << maxRoutesFromStop << std::endl;
        std::cout << "Route trip infos stored: " << allRouteTrips.size() << std::endl;
        std::cout << "============================================" << std::endl;
    }
};