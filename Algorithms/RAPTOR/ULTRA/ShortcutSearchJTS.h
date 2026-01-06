#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <algorithm>

#include "../../Dijkstra/Dijkstra.h"

#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/IndexedSet.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/Shortcut.h"
#include "../../../DataStructures/Graph/TimeDependentGraph.h"

namespace RAPTOR::ULTRA {

template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class ShortcutSearchJST {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = ShortcutSearchJST<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

public:
    // Label for Dijkstra priority queue
    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never), tripCount(0), parent(noVertex),
                         transferParent(noStop), routeParent(noStop) {}

        int arrivalTime;
        uint8_t tripCount;      // Number of trips taken (0, 1, or 2)
        Vertex parent;          // Previous vertex
        StopId transferParent;  // For candidates: stop where transfer occurred after trip 1
        StopId routeParent;     // For trip 2 arrivals: boarding stop of trip 2

        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime ||
                   (arrivalTime == other->arrivalTime && this < other);
        }

        inline void reset() {
            arrivalTime = never;
            tripCount = 0;
            parent = noVertex;
            transferParent = noStop;
            routeParent = noStop;
        }
    };

    struct DepartureLabel {
        DepartureLabel(const int departureTime = never) : departureTime(departureTime) {}
        std::vector<std::pair<Vertex, StopIndex>> boardingPoints; // (stop, stopIndex) pairs per route
        int departureTime;
        inline bool operator<(const DepartureLabel& other) const noexcept {
            return departureTime > other.departureTime;
        }
    };

    struct Station {
        Station() : representative(noStop) {}
        StopId representative;
        std::vector<StopId> stops;
        inline void add(const StopId stop) noexcept {
            if (representative > stop) {
                representative = stop;
            }
            stops.emplace_back(stop);
        }
    };

public:
    ShortcutSearchJST(const RAPTOR::Data& raptorData, const TimeDependentGraph& tdGraph,
                      DynamicTransferGraph& shortcutGraph, const int witnessTransferLimit) :
        raptorData(raptorData),
        graph(tdGraph),
        shortcutGraph(shortcutGraph),
        stationOfStop(raptorData.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        witnessTransferLimit(witnessTransferLimit),
        earliestDepartureTime(raptorData.getMinDepartureTime()),
        optimalCandidates(0),
        timestamp(0) {

        Assert(raptorData.hasImplicitBufferTimes(), "Shortcut search requires implicit departure buffer times!");

        // Build stations (groups of stops reachable by zero-cost transfers)
        Dijkstra<TransferGraph, false> dijkstra(raptorData.transferGraph);
        for (const StopId stop : raptorData.stops()) {
            dijkstra.run(stop, noVertex, [&](const Vertex u) {
                if (!raptorData.isStop(u)) return;
                stationOfStop[stop].add(StopId(u));
            }, NoOperation, [&](const Vertex, const Edge edge) {
                return raptorData.transferGraph.get(TravelTime, edge) > 0;
            });
        }
    }

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        Assert(raptorData.isStop(source), "source (" << source << ") is not a stop!");
        if (stationOfStop[source].representative != source) return;

        setSource(source);

        for (const DepartureLabel& label : collectDepartures(minTime, maxTime)) {
            runForDepartureTime(label);

            if constexpr (CountOptimalCandidates) {
                optimalCandidates += shortcuts.size();
            }

            for (const Shortcut& shortcut : shortcuts) {
                if (!shortcutGraph.hasEdge(shortcut.origin, shortcut.destination)) {
                    shortcutGraph.addEdge(shortcut.origin, shortcut.destination).set(TravelTime, shortcut.travelTime);
                } else {
                    Assert(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) == shortcut.travelTime,
                           "Edge from " << shortcut.origin << " to " << shortcut.destination <<
                           " has inconclusive travel time!");
                }
            }
        }
    }

    inline size_t getNumberOfOptimalCandidates() const noexcept requires CountOptimalCandidates {
        return optimalCandidates;
    }

private:
    inline void setSource(const StopId source) noexcept {
        Assert(stationOfStop[source].representative == source,
               "Source " << source << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[source];

        // Initialize direct transfer distances from source station
        initialDijkstra();
        sort(stopsReachedByDirectTransfer);

        if constexpr (Debug) {
            std::cout << "   Source stop: " << source << std::endl;
            std::cout << "   Number of stops reached by direct transfer: "
                      << String::prettyInt(stopsReachedByDirectTransfer.size()) << std::endl;
        }
    }

    inline void runForDepartureTime(const DepartureLabel& label) noexcept {
        if constexpr (Debug) {
            std::cout << "   Running search for departure time: " << label.departureTime
                      << " (" << String::secToTime(label.departureTime) << ")" << std::endl;
        }

        timestamp++;
        shortcuts.clear();
        shortcutDestinationCandidates.clear();

        sourceDepartureTime = label.departureTime;

        // Run unified time-dependent Dijkstra tracking trip counts
        runTimeDependentSearch();
    }

    inline std::vector<DepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        Assert(directTransferDistance[sourceStation.representative] == 0,
               "Direct transfer for source " << sourceStation.representative << " is incorrect!");

        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::set<int> departureTimes;

        // Collect all relevant departure times
        for (const StopId stop : stopsReachedByDirectTransfer) {
            const int transferTime = directTransferDistance[stop];

            for (const Edge e : graph.edgesFrom(stop)) {
                const auto& handle = graph.get(Function, e);
                const auto* begin = graph.getTripsBegin(handle);
                const auto* end = graph.getTripsEnd(handle);

                for (auto it = begin; it != end; ++it) {
                    const int departureTime = it->departureTime + transferTime;
                    if (departureTime < cutoffTime) continue;
                    if (departureTime > maxTime) break;

                    // Only add if departing from source station
                    if (stationOfStop[stop].representative == sourceStation.representative) {
                        departureTimes.insert(departureTime);
                    }
                    departureTimes.insert(departureTime);
                }
            }
        }

        std::vector<DepartureLabel> result;
        for (const int depTime : departureTimes) {
            result.emplace_back(depTime);
        }

        return result;
    }

    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<int>(raptorData.transferGraph.numVertices(), never).swap(directTransferDistance);
        stopsReachedByDirectTransfer.clear();

        std::vector<ArrivalLabel>(raptorData.transferGraph.numVertices()).swap(labels);
        std::vector<u_int16_t>(raptorData.transferGraph.numVertices(), 0).swap(timestamps);

        queue.clear();

        shortcutDestinationCandidates.clear();
        shortcuts.clear();
    }

    inline void initialDijkstra() noexcept {
        // Simple Dijkstra from source station to find all reachable stops by walking
        struct SimpleDijkstraLabel : public ExternalKHeapElement {
            int distance = never;
            inline bool hasSmallerKey(const SimpleDijkstraLabel* const other) const noexcept {
                return distance < other->distance;
            }
        };

        std::vector<SimpleDijkstraLabel> dijkstraLabels(raptorData.transferGraph.numVertices());
        ExternalKHeap<2, SimpleDijkstraLabel> dijkstraQueue;

        dijkstraLabels[sourceStation.representative].distance = 0;
        directTransferDistance[sourceStation.representative] = 0;
        dijkstraQueue.update(&dijkstraLabels[sourceStation.representative]);

        while (!dijkstraQueue.empty()) {
            SimpleDijkstraLabel* current = dijkstraQueue.extractFront();
            const Vertex u = Vertex(current - &dijkstraLabels[0]);

            for (const Edge edge : raptorData.transferGraph.edgesFrom(u)) {
                const Vertex v = raptorData.transferGraph.get(ToVertex, edge);
                const int newDist = current->distance + raptorData.transferGraph.get(TravelTime, edge);

                if (newDist < dijkstraLabels[v].distance) {
                    dijkstraLabels[v].distance = newDist;
                    directTransferDistance[v] = newDist;
                    dijkstraQueue.update(&dijkstraLabels[v]);
                }
            }

            if (raptorData.isStop(u)) {
                stopsReachedByDirectTransfer.emplace_back(StopId(u));
            }
        }
    }

    inline void runTimeDependentSearch() noexcept {
        // Initialize labels for stops reachable by direct transfer (0 trips)
        for (const StopId stop : stopsReachedByDirectTransfer) {
            const int arrivalTime = sourceDepartureTime + directTransferDistance[stop];
            ArrivalLabel& L = getLabel(stop);
            L.arrivalTime = arrivalTime;
            L.tripCount = 0;
            L.parent = sourceStation.representative;
            L.transferParent = noStop;
            L.routeParent = noStop;
            queue.update(&L);
        }

        int transferLimit = intMax;
        bool allCandidatesSettled = false;

        while (!queue.empty()) {
            ArrivalLabel* current = queue.extractFront();
            const Vertex u = Vertex(current - &labels[0]);
            const int currentTime = current->arrivalTime;
            const uint8_t currentTrips = current->tripCount;

            // Check transfer limit
            if (allCandidatesSettled && currentTime > transferLimit) break;

            // Process based on trip count
            if (currentTrips == 2) {
                // Final arrival after 2 trips - check for shortcut
                if (raptorData.isStop(u) && current->routeParent != noStop) {
                    const StopId routeParent = current->routeParent;
                    const StopId transferParent = current->transferParent;

                    if (raptorData.isStop(transferParent) && transferParent != routeParent) {
                        // Calculate walking distance for shortcut
                        const int walkingDistance = getLabel(routeParent).arrivalTime -
                                                   getLabel(transferParent).arrivalTime;

                        if constexpr (IgnoreIsolatedCandidates) {
                            if (directTransferDistance[u] < never) {
                                shortcuts.emplace_back(transferParent, routeParent, walkingDistance);
                            }
                        } else {
                            shortcuts.emplace_back(transferParent, routeParent, walkingDistance);
                        }
                    }
                }
                continue; // Don't expand further after 2 trips
            }

            // Scan transit edges (board vehicles)
            if (raptorData.isStop(u) && currentTrips < 2) {
                scanTransitEdges(u, currentTime, currentTrips, current->transferParent);
            }

            // Relax walking edges
            for (const Edge edge : raptorData.transferGraph.edgesFrom(u)) {
                const Vertex v = raptorData.transferGraph.get(ToVertex, edge);
                const int newTime = currentTime + raptorData.transferGraph.get(TravelTime, edge);

                ArrivalLabel& targetLabel = getLabel(v);
                if (newTime < targetLabel.arrivalTime ||
                    (newTime == targetLabel.arrivalTime && currentTrips < targetLabel.tripCount)) {

                    targetLabel.arrivalTime = newTime;
                    targetLabel.tripCount = currentTrips;
                    targetLabel.parent = u;
                    targetLabel.transferParent = current->transferParent;
                    targetLabel.routeParent = noStop;
                    queue.update(&targetLabel);
                }
            }

            // Check if we should set transfer limit (when first trip candidates are settled)
            if (currentTrips == 1 && raptorData.isStop(current->transferParent) && !allCandidatesSettled) {
                // This is a simplification - in practice need more sophisticated tracking
                if (transferLimit == intMax) {
                    transferLimit = currentTime + witnessTransferLimit;
                    if constexpr (Debug) {
                        std::cout << "   Transfer limit set: "
                                  << String::secToString(transferLimit - sourceDepartureTime) << std::endl;
                    }
                }
            }
        }
    }

    inline void scanTransitEdges(const Vertex u, const int departureTime,
                                  const uint8_t currentTrips, const StopId currentTransferParent) noexcept {
        for (const Edge e : graph.edgesFrom(u)) {
            const Vertex v = graph.get(ToVertex, e);
            const auto& handle = graph.get(Function, e);

            if (handle.tripCount == 0) continue; // Walking-only edge

            const auto* begin = graph.getTripsBegin(handle);
            const auto* end = graph.getTripsEnd(handle);

            // Find first valid trip
            auto it = std::lower_bound(begin, end, departureTime,
                [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

            if (it == end) continue;

            // Take earliest trip
            const int arrivalTime = it->arrivalTime;
            const uint8_t newTripCount = currentTrips + 1;

            ArrivalLabel& targetLabel = getLabel(v);

            // Determine transfer parent for this journey
            StopId newTransferParent = currentTransferParent;
            StopId newRouteParent = noStop;

            if (currentTrips == 0) {
                // First trip: mark boarding stop as potential shortcut origin
                if (stationOfStop[u].representative == sourceStation.representative) {
                    newTransferParent = StopId(u);
                }
            } else if (currentTrips == 1) {
                // Second trip: this is a candidate journey
                newRouteParent = StopId(u);
            }

            if (arrivalTime < targetLabel.arrivalTime ||
                (arrivalTime == targetLabel.arrivalTime && newTripCount < targetLabel.tripCount)) {

                targetLabel.arrivalTime = arrivalTime;
                targetLabel.tripCount = newTripCount;
                targetLabel.parent = u;
                targetLabel.transferParent = newTransferParent;
                targetLabel.routeParent = newRouteParent;
                queue.update(&targetLabel);
            }

            // Continue riding the trip to subsequent stops
            if (it->tripId >= 0) {
                scanTrip(it->tripId, it->departureStopIndex + 1, arrivalTime,
                        StopId(u), newTripCount, newTransferParent);
            }
        }
    }

    inline void scanTrip(const int tripId, const uint16_t startStopIndex, int currentArrival,
                         const StopId boardStop, const uint8_t tripCount, const StopId transferParent) noexcept {

        uint32_t currentIdx = graph.getTripOffset(tripId) + startStopIndex;
        uint32_t endIdx = graph.getTripOffset(tripId + 1);

        for (uint32_t idx = currentIdx; idx < endIdx; ++idx) {
            const auto& leg = graph.getTripLeg(idx);
            const Vertex stopVertex = leg.stopId;

            ArrivalLabel& targetLabel = getLabel(stopVertex);

            StopId routeParent = (tripCount == 2) ? boardStop : noStop;

            if (currentArrival < targetLabel.arrivalTime ||
                (currentArrival == targetLabel.arrivalTime && tripCount < targetLabel.tripCount)) {

                targetLabel.arrivalTime = currentArrival;
                targetLabel.tripCount = tripCount;
                targetLabel.parent = boardStop;
                targetLabel.transferParent = transferParent;
                targetLabel.routeParent = routeParent;
                queue.update(&targetLabel);
            }

            if (idx + 1 < endIdx) {
                const auto& nextLeg = graph.getTripLeg(idx + 1);
                currentArrival = nextLeg.arrivalTime;
            }
        }
    }

    inline ArrivalLabel& getLabel(const Vertex v) noexcept {
        ArrivalLabel& L = labels[v];
        if (timestamps[v] != timestamp) {
            L.reset();
            timestamps[v] = timestamp;
        }
        return L;
    }

private:
    const RAPTOR::Data& raptorData;
    const TimeDependentGraph& graph;
    DynamicTransferGraph& shortcutGraph;

    std::vector<Station> stationOfStop;
    Station sourceStation;
    int sourceDepartureTime;

    std::vector<int> directTransferDistance;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<ArrivalLabel> labels;
    std::vector<u_int16_t> timestamps;
    ExternalKHeap<2, ArrivalLabel> queue;

    IndexedMap<std::set<StopId>, false, StopId> shortcutDestinationCandidates;
    std::vector<Shortcut> shortcuts;

    int witnessTransferLimit;
    int earliestDepartureTime;
    size_t optimalCandidates;
    u_int16_t timestamp;
};

}