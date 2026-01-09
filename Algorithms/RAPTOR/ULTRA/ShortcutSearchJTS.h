#pragma once

#include <vector>
#include <set>
#include <algorithm>
#include <unordered_map>

#include "../../Dijkstra/Dijkstra.h"
#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Vector/Vector.h"
#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/IndexedSet.h"
#include "../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../DataStructures/Intermediate/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/Shortcut.h"
#include "../../../DataStructures/Graph/TimeDependentGraph.h"

namespace RAPTOR::ULTRA {

template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class ShortcutSearchJTS {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = ShortcutSearchJTS<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

    //==========================================================================
    // ROUTE DATA STRUCTURES
    //==========================================================================

    // Represents a route (sequence of stops with multiple trips)
    struct Route {
        std::vector<StopId> stops;                    // Sequence of stops
        std::vector<size_t> tripIds;                  // Trips on this route (indices into data.trips)
        std::vector<std::vector<int>> departureTimes; // [stopIndex][tripIndex] -> departure time (buffer-adjusted)
        std::vector<std::vector<int>> arrivalTimes;   // [stopIndex][tripIndex] -> arrival time
    };

    // Maps a stop to routes serving it
    struct RouteSegment {
        size_t routeId;
        StopIndex stopIndex;
    };

    //==========================================================================
    // TRIP ITERATOR - Matches original TripIterator behavior exactly
    //==========================================================================

    class TripIteratorJTS {
    public:
        TripIteratorJTS(const std::vector<Route>& routes, const size_t routeId, const StopIndex startStopIndex) :
            route(routes[routeId]),
            currentStopIndex(startStopIndex),
            currentTripIndex(route.tripIds.size() - 1) {  // Start at LAST trip (like original)
        }

        // Are there more stops after the current one?
        inline bool hasFurtherStops() const noexcept {
            return currentStopIndex + 1 < route.stops.size();
        }

        // Is there an earlier trip we could switch to?
        inline bool hasEarlierTrip() const noexcept {
            return currentTripIndex > 0;
        }

        // Departure time of the PREVIOUS (earlier) trip at current stop
        inline int previousDepartureTime() const noexcept {
            return route.departureTimes[currentStopIndex][currentTripIndex - 1];
        }

        // Switch to earlier trip
        inline void previousTrip() noexcept {
            Assert(hasEarlierTrip(), "No earlier trip!");
            currentTripIndex--;
        }

        // Move to next stop
        inline void nextStop() noexcept {
            Assert(hasFurtherStops(), "No further stops!");
            currentStopIndex++;
        }

        // Arrival time at current stop on current trip
        inline int arrivalTime() const noexcept {
            return route.arrivalTimes[currentStopIndex][currentTripIndex];
        }

        // Departure time at current stop on current trip
        inline int departureTime() const noexcept {
            return route.departureTimes[currentStopIndex][currentTripIndex];
        }

        // Current stop
        inline StopId stop() const noexcept {
            return route.stops[currentStopIndex];
        }

        // Stop at given index
        inline StopId stop(const StopIndex index) const noexcept {
            return route.stops[index];
        }

        // Current stop index
        inline StopIndex getStopIndex() const noexcept {
            return currentStopIndex;
        }

    private:
        const Route& route;
        StopIndex currentStopIndex;
        size_t currentTripIndex;
    };

    //==========================================================================
    // LABEL STRUCTURES (same as original)
    //==========================================================================

    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;

        inline bool hasSmallerKey(const ArrivalLabel* const other) const noexcept {
            return arrivalTime < other->arrivalTime ||
                   (arrivalTime == other->arrivalTime && this < other);
        }
    };

    struct Station {
        Station() : representative(noStop) {}
        StopId representative;
        std::vector<StopId> stops;

        inline void add(const StopId stop) noexcept {
            if (representative > stop) representative = stop;
            stops.emplace_back(stop);
        }
    };

    struct DepartureLabel {
        DepartureLabel(const size_t routeId = size_t(-1), const StopIndex stopIndex = noStopIndex,
                       const int departureTime = never)
            : routeId(routeId), stopIndex(stopIndex), departureTime(departureTime) {}
        size_t routeId;
        StopIndex stopIndex;
        int departureTime;

        inline bool operator<(const DepartureLabel& other) const noexcept {
            return (departureTime > other.departureTime) ||
                   ((departureTime == other.departureTime) && (routeId < other.routeId));
        }
    };

    struct ConsolidatedDepartureLabel {
        ConsolidatedDepartureLabel(const int departureTime = never) : departureTime(departureTime) {}
        std::vector<RouteSegment> routes;
        int departureTime;

        inline bool operator<(const ConsolidatedDepartureLabel& other) const noexcept {
            return departureTime > other.departureTime;
        }
    };

public:
    ShortcutSearchJTS(const Intermediate::Data& data, const TimeDependentGraph& tdGraph,
                      DynamicTransferGraph& shortcutGraph, const int witnessTransferLimit) :
        data(data),
        graph(tdGraph),
        shortcutGraph(shortcutGraph),
        numberOfStops(data.numberOfStops()),
        numberOfVertices(tdGraph.numVertices()),
        witnessTransferLimit(witnessTransferLimit),
        earliestDepartureTime(computeMinDepartureTime(data)),
        stationOfStop(numberOfStops),
        sourceStation(),
        sourceDepartureTime(0),
        shortcutCandidatesInQueue(0),
        shortcutDestinationCandidates(numberOfStops),
        optimalCandidates(0),
        routesServingUpdatedStops(0),  // Will resize after building routes
        stopsUpdatedByRoute(numberOfStops),
        stopsUpdatedByTransfer(numberOfStops),
        timestamp(0) {

        buildStations();
        buildRouteStructures();

        // Now resize routesServingUpdatedStops to number of routes
        routesServingUpdatedStops = IndexedMap<StopIndex, false, size_t>(routes.size());
    }

    //==========================================================================
    // PUBLIC INTERFACE
    //==========================================================================

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        if (source >= numberOfStops) return;
        if (stationOfStop[source].representative != source) return;

        setSource(source);

        for (const ConsolidatedDepartureLabel& label : collectDepartures(minTime, maxTime)) {
            runForDepartureTime(label);

            if constexpr (CountOptimalCandidates) {
                optimalCandidates += shortcuts.size();
            }

            for (const Shortcut& shortcut : shortcuts) {
                if (!shortcutGraph.hasEdge(shortcut.origin, shortcut.destination)) {
                    shortcutGraph.addEdge(shortcut.origin, shortcut.destination)
                        .set(TravelTime, shortcut.travelTime);
                } else {
                    Assert(shortcutGraph.get(TravelTime,
                        shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) == shortcut.travelTime,
                        "Inconsistent travel time!");
                }
            }
        }
    }

    inline size_t getNumberOfOptimalCandidates() const noexcept requires CountOptimalCandidates {
        return optimalCandidates;
    }

private:
    //==========================================================================
    // INITIALIZATION
    //==========================================================================

    inline static int computeMinDepartureTime(const Intermediate::Data& data) noexcept {
        int minDep = never;
        for (const Intermediate::Trip& trip : data.trips) {
            if (!trip.stopEvents.empty()) {
                minDep = std::min(minDep, trip.stopEvents.front().departureTime);
            }
        }
        return minDep;
    }

    inline void buildStations() noexcept {
        TransferGraph transferGraph;
        Graph::copy(data.transferGraph, transferGraph);

        Dijkstra<TransferGraph, false> dijkstra(transferGraph);
        for (size_t stop = 0; stop < numberOfStops; stop++) {
            dijkstra.run(Vertex(stop), noVertex, [&](const Vertex u) {
                if (u >= numberOfStops) return;
                stationOfStop[stop].add(StopId(u));
            }, NoOperation, [&](const Vertex, const Edge edge) {
                return transferGraph.get(TravelTime, edge) > 0;
            });
        }
    }

    // Build route structures from Intermediate::Data
    inline void buildRouteStructures() noexcept {
        if constexpr (Debug) {
            std::cout << "Built " << routes.size() << " routes from "
                      << data.trips.size() << " trips" << std::endl;
        }

        // Group trips by stop pattern
        std::map<std::vector<StopId>, size_t> patternToRouteId;

        for (size_t tripId = 0; tripId < data.trips.size(); ++tripId) {
            const auto& trip = data.trips[tripId];
            if (trip.stopEvents.size() < 2) continue;

            // Extract stop pattern
            std::vector<StopId> pattern;
            pattern.reserve(trip.stopEvents.size());
            for (const auto& ev : trip.stopEvents) {
                pattern.push_back(ev.stopId);
            }

            // Find or create route
            auto it = patternToRouteId.find(pattern);
            size_t routeId;
            if (it == patternToRouteId.end()) {
                routeId = routes.size();
                patternToRouteId[pattern] = routeId;

                Route route;
                route.stops = pattern;
                route.departureTimes.resize(pattern.size());
                route.arrivalTimes.resize(pattern.size());
                routes.push_back(std::move(route));
            } else {
                routeId = it->second;
            }

            // Add trip to route
            routes[routeId].tripIds.push_back(tripId);
        }

        // Fill departure/arrival times and sort trips by departure time at first stop
        for (Route& route : routes) {
            const size_t numStops = route.stops.size();
            const size_t numTrips = route.tripIds.size();

            // Resize time arrays
            for (size_t s = 0; s < numStops; s++) {
                route.departureTimes[s].resize(numTrips);
                route.arrivalTimes[s].resize(numTrips);
            }

            // Fill times
            for (size_t t = 0; t < numTrips; t++) {
                const auto& trip = data.trips[route.tripIds[t]];
                for (size_t s = 0; s < numStops; s++) {
                    // Apply buffer time to departure (like original RAPTOR data)
                    const StopId stop = route.stops[s];
                    const int buffer = (stop < data.stops.size()) ? data.stops[stop].minTransferTime : 0;

                    route.departureTimes[s][t] = trip.stopEvents[s].departureTime - buffer;
                    route.arrivalTimes[s][t] = trip.stopEvents[s].arrivalTime;
                }
            }

            // Sort trips by departure time at first stop
            std::vector<size_t> sortOrder(numTrips);
            std::iota(sortOrder.begin(), sortOrder.end(), 0);
            std::sort(sortOrder.begin(), sortOrder.end(), [&](size_t a, size_t b) {
                return route.departureTimes[0][a] < route.departureTimes[0][b];
            });

            // Apply sort order
            std::vector<size_t> newTripIds(numTrips);
            std::vector<std::vector<int>> newDepartures(numStops), newArrivals(numStops);
            for (size_t s = 0; s < numStops; s++) {
                newDepartures[s].resize(numTrips);
                newArrivals[s].resize(numTrips);
            }

            for (size_t t = 0; t < numTrips; t++) {
                newTripIds[t] = route.tripIds[sortOrder[t]];
                for (size_t s = 0; s < numStops; s++) {
                    newDepartures[s][t] = route.departureTimes[s][sortOrder[t]];
                    newArrivals[s][t] = route.arrivalTimes[s][sortOrder[t]];
                }
            }

            route.tripIds = std::move(newTripIds);
            route.departureTimes = std::move(newDepartures);
            route.arrivalTimes = std::move(newArrivals);
        }

        // Build routesContainingStop
        routesContainingStop.resize(numberOfStops);
        for (size_t routeId = 0; routeId < routes.size(); ++routeId) {
            const Route& route = routes[routeId];
            for (size_t s = 0; s < route.stops.size(); s++) {
                const StopId stop = route.stops[s];
                if (stop < numberOfStops) {
                    routesContainingStop[stop].push_back({routeId, StopIndex(s)});
                }
            }
        }

        if constexpr (Debug) {
            std::cout << " done. " << routes.size() << " routes created." << std::endl;
        }
    }

    inline bool isStop(const Vertex v) const noexcept {
        return v < numberOfStops;
    }

    inline bool isRoute(const size_t routeId) const noexcept {
        return routeId < routes.size();
    }

    inline size_t numberOfStopsInRoute(const size_t routeId) const noexcept {
        return routes[routeId].stops.size();
    }

    inline StopId stopOfRouteSegment(const RouteSegment& segment) const noexcept {
        return routes[segment.routeId].stops[segment.stopIndex];
    }

    inline int lastTripDepartureTime(const size_t routeId, const StopIndex stopIndex) const noexcept {
        const Route& route = routes[routeId];
        if (route.tripIds.empty()) return never;
        return route.departureTimes[stopIndex].back();
    }

    inline TripIteratorJTS getTripIterator(const size_t routeId, const StopIndex stopIndex) const noexcept {
        return TripIteratorJTS(routes, routeId, stopIndex);
    }

    inline void setSource(const StopId source) noexcept {
        Assert(directTransferQueue.empty(), "Queue for round 0 is not empty!");
        Assert(stationOfStop[source].representative == source,
               "Source " << source << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[source];
        initialDijkstra();
        sort(stopsReachedByDirectTransfer);

        if constexpr (Debug) {
            std::cout << "   Source stop: " << source << std::endl;
            std::cout << "   Number of stops reached by direct transfer: "
                      << String::prettyInt(stopsReachedByDirectTransfer.size()) << std::endl;
        }
    }

    inline void runForDepartureTime(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) {
            std::cout << "   Running search for departure time: " << label.departureTime
                      << " (" << String::secToTime(label.departureTime) << ")" << std::endl;
        }

        timestamp++;
        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        relaxInitialTransfers();
        collectRoutes1(label.routes);
        scanRoutes<1>();
        intermediateDijkstra();
        collectRoutes2();
        scanRoutes<2>();
        finalDijkstra();
    }

    inline std::vector<ConsolidatedDepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        Assert(directTransferArrivalLabels[sourceStation.representative].arrivalTime == 0,
               "Direct transfer for source is incorrect!");

        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::vector<DepartureLabel> departureLabels;

        for (size_t routeId = 0; routeId < routes.size(); ++routeId) {
            const Route& route = routes[routeId];
            const size_t tripSize = route.stops.size();
            int minimalTransferTime = never;

            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                const StopId stop = route.stops[stopIndex];
                if (stop >= numberOfStops) continue;

                if (directTransferArrivalLabels[stop].arrivalTime > minimalTransferTime) continue;
                minimalTransferTime = directTransferArrivalLabels[stop].arrivalTime;

                for (size_t tripIdx = 0; tripIdx < route.tripIds.size(); ++tripIdx) {
                    const int departureTime = route.departureTimes[stopIndex][tripIdx] - minimalTransferTime;
                    if (departureTime < cutoffTime) continue;
                    if (departureTime > maxTime) break;

                    if (stationOfStop[stop].representative == sourceStation.representative) {
                        departureLabels.emplace_back(size_t(-1), noStopIndex, departureTime);
                    }
                    departureLabels.emplace_back(routeId, StopIndex(stopIndex), departureTime);
                }
            }
        }

        sort(departureLabels);

        std::vector<ConsolidatedDepartureLabel> result(1);
        for (const DepartureLabel& label : departureLabels) {
            if (label.routeId == size_t(-1)) {
                if (label.departureTime == result.back().departureTime) continue;
                result.back().departureTime = label.departureTime;
                result.emplace_back(label.departureTime);
            } else {
                result.back().routes.push_back({label.routeId, label.stopIndex});
            }
        }
        result.pop_back();

        for (ConsolidatedDepartureLabel& label : result) {
            std::sort(label.routes.begin(), label.routes.end(),
                [](const RouteSegment& a, const RouteSegment& b) {
                    return a.routeId < b.routeId ||
                           (a.routeId == b.routeId && a.stopIndex < b.stopIndex);
                });
        }

        return result;
    }

private:
    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<ArrivalLabel>(numberOfVertices).swap(directTransferArrivalLabels);
        stopsReachedByDirectTransfer.clear();

        std::vector<ArrivalLabel>(numberOfStops).swap(zeroTripsArrivalLabels);

        oneTripQueue.clear();
        std::vector<ArrivalLabel>(numberOfVertices).swap(oneTripArrivalLabels);
        std::vector<uint16_t>(numberOfVertices, 0).swap(oneTripTimestamps);

        twoTripsQueue.clear();
        std::vector<ArrivalLabel>(numberOfVertices).swap(twoTripsArrivalLabels);
        std::vector<uint16_t>(numberOfVertices, 0).swap(twoTripsTimestamps);

        std::vector<StopId>(numberOfVertices, noStop).swap(oneTripTransferParent);
        std::vector<StopId>(numberOfStops, noStop).swap(twoTripsRouteParent);

        shortcutCandidatesInQueue = 0;
        shortcutDestinationCandidates.clear();
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    inline void collectRoutes1(const std::vector<RouteSegment>& segments) noexcept {
        for (const RouteSegment& segment : segments) {
            Assert(isRoute(segment.routeId), "Route " << segment.routeId << " is out of range!");
            Assert(segment.stopIndex + 1 < numberOfStopsInRoute(segment.routeId),
                   "RouteSegment is not a departure event!");
            Assert(lastTripDepartureTime(segment.routeId, segment.stopIndex) >=
                   arrivalTime<0>(stopOfRouteSegment(segment)),
                   "RouteSegment is not reachable!");

            if (routesServingUpdatedStops.contains(segment.routeId)) {
                routesServingUpdatedStops[segment.routeId] =
                    std::min(routesServingUpdatedStops[segment.routeId], segment.stopIndex);
            } else {
                routesServingUpdatedStops.insert(segment.routeId, segment.stopIndex);
            }
        }
    }

    inline void collectRoutes2() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& segment : routesContainingStop[stop]) {
                Assert(isRoute(segment.routeId), "Route " << segment.routeId << " is out of range!");

                if (segment.stopIndex + 1 == numberOfStopsInRoute(segment.routeId)) continue;
                if (lastTripDepartureTime(segment.routeId, segment.stopIndex) <
                    oneTripArrivalLabels[stop].arrivalTime) continue;

                if (routesServingUpdatedStops.contains(segment.routeId)) {
                    routesServingUpdatedStops[segment.routeId] =
                        std::min(routesServingUpdatedStops[segment.routeId], segment.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(segment.routeId, segment.stopIndex);
                }
            }
        }
        routesServingUpdatedStops.sortKeys();
    }

    //==========================================================================
    // ROUTE SCANNING - Exact match to original scanRoutes<CURRENT>()
    //==========================================================================

    template<int CURRENT>
    inline void scanRoutes() noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");

        for (const size_t routeId : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[routeId];
            TripIteratorJTS tripIterator = getTripIterator(routeId, stopIndex);
            StopIndex parentIndex = stopIndex;

            while (tripIterator.hasFurtherStops()) {
                // Find earliest trip that can be entered
                if (tripIterator.hasEarlierTrip() &&
                    (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop()))) {
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() &&
                             (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())));

                    if (!stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                        // Trip was improved by an arrival from a previous iteration.
                        // Fast forward to next updated stop that can enter current trip.
                        do {
                            tripIterator.nextStop();
                        } while (tripIterator.hasFurtherStops() &&
                                 ((!stopsUpdatedByTransfer.contains(tripIterator.stop())) ||
                                  (tripIterator.departureTime() < arrivalTime<CURRENT - 1>(tripIterator.stop()))));
                        parentIndex = tripIterator.getStopIndex();
                        continue;
                    }
                    parentIndex = tripIterator.getStopIndex();
                }
                // Candidates may dominate equivalent labels from previous iterations
                else if (stopsUpdatedByTransfer.contains(tripIterator.stop()) &&
                         !isFromCurrentIteration<CURRENT - 1>(tripIterator.stop(parentIndex)) &&
                         isCandidate<CURRENT>(tripIterator.stop()) &&
                         tripIterator.departureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())) {
                    parentIndex = tripIterator.getStopIndex();
                }

                tripIterator.nextStop();
                const int newArrivalTime = tripIterator.arrivalTime();

                if (newArrivalTime < arrivalTime<CURRENT>(tripIterator.stop())) {
                    arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, tripIterator.stop(parentIndex));
                }
                // Candidates may dominate equivalent labels from previous iterations
                else if (newArrivalTime == arrivalTime<CURRENT>(tripIterator.stop()) &&
                         !isFromCurrentIteration<CURRENT>(tripIterator.stop()) &&
                         newArrivalTime < arrivalTime<CURRENT - 1>(tripIterator.stop())) {
                    const StopId parent = tripIterator.stop(parentIndex);
                    if (isCandidate<CURRENT>(parent)) {
                        arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, parent);
                    }
                }
            }
        }

        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
    }

    template<int CURRENT>
    inline bool isCandidate(const StopId parent) const noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        if constexpr (CURRENT == 1) {
            return stationOfStop[parent].representative == sourceStation.representative;
        } else {
            return oneTripTransferParent[parent] != noStop;
        }
    }

    template<int CURRENT>
    inline bool isFromCurrentIteration(const StopId stop) const noexcept {
        static_assert((CURRENT == 0) | (CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        if constexpr (CURRENT == 0) {
            suppressUnusedParameterWarning(stop);
            return true;
        } else if constexpr (CURRENT == 1) {
            return oneTripTimestamps[stop] == timestamp;
        } else {
            return twoTripsTimestamps[stop] == timestamp;
        }
    }

    //==========================================================================
    // DIJKSTRA PHASES
    //==========================================================================

    inline void initialDijkstra() noexcept {
        directTransferArrivalLabels[sourceStation.representative].arrivalTime = 0;
        directTransferQueue.update(&directTransferArrivalLabels[sourceStation.representative]);

        while (!directTransferQueue.empty()) {
            ArrivalLabel* currentLabel = directTransferQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &directTransferArrivalLabels[0]);

            for (const Edge edge : graph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = graph.get(ToVertex, edge);
                const int walkTime = graph.getWalkArrivalFrom(edge, 0);
                if (walkTime == never) continue;

                const int newArrivalTime = currentLabel->arrivalTime + walkTime;
                if (newArrivalTime < directTransferArrivalLabels[neighborVertex].arrivalTime) {
                    directTransferArrivalLabels[neighborVertex].arrivalTime = newArrivalTime;
                    directTransferQueue.update(&directTransferArrivalLabels[neighborVertex]);
                }
            }

            if (isStop(currentVertex)) {
                stopsReachedByDirectTransfer.emplace_back(StopId(currentVertex));
            }
        }

        for (const StopId sourceStop : sourceStation.stops) {
            Assert(Vector::contains(stopsReachedByDirectTransfer, sourceStop),
                   "Source was not updated by transfer!");
        }
    }

    inline void relaxInitialTransfers() noexcept {
        Assert(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");
        for (const StopId stop : stopsReachedByDirectTransfer) {
            const int newArrivalTime = sourceDepartureTime + directTransferArrivalLabels[stop].arrivalTime;
            arrivalByEdge0(stop, newArrivalTime);
            stopsUpdatedByTransfer.insert(stop);
        }
    }

    inline void intermediateDijkstra() noexcept {
        Assert(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        shortcutCandidatesInQueue = 0;
        for (const StopId stop : stopsUpdatedByRoute) {
            oneTripQueue.update(&oneTripArrivalLabels[stop]);
            if (isStop(oneTripTransferParent[stop])) shortcutCandidatesInQueue++;
        }

        if (shortcutCandidatesInQueue == 0) {
            stopsUpdatedByRoute.clear();
            return;
        }

        int transferLimit = intMax;

        while (!oneTripQueue.empty()) {
            ArrivalLabel* currentLabel = oneTripQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &oneTripArrivalLabels[0]);

            for (const Edge edge : graph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = graph.get(ToVertex, edge);
                const int walkTime = graph.getWalkArrivalFrom(edge, 0);
                if (walkTime == never) continue;

                const int newArrivalTime = currentLabel->arrivalTime + walkTime;

                if (newArrivalTime < oneTripArrivalLabels[neighborVertex].arrivalTime) {
                    arrivalByEdge1(neighborVertex, newArrivalTime, currentVertex);
                }
                // Candidates may dominate equivalent labels from previous iterations
                else if (newArrivalTime == oneTripArrivalLabels[neighborVertex].arrivalTime) {
                    const bool candidateLabel = oneTripTransferParent[currentVertex] != noStop;
                    const bool isFromPreviousIteration = oneTripTimestamps[neighborVertex] != timestamp;
                    if (candidateLabel && isFromPreviousIteration) {
                        arrivalByEdge1(neighborVertex, newArrivalTime, currentVertex);
                    }
                }
            }

            if (isStop(oneTripTransferParent[currentVertex])) {
                shortcutCandidatesInQueue--;
            }

            if (shortcutCandidatesInQueue == 0) {
                shortcutCandidatesInQueue = -1;
                transferLimit = currentLabel->arrivalTime + witnessTransferLimit;
                if (transferLimit < currentLabel->arrivalTime) transferLimit = intMax;
                if constexpr (Debug) {
                    std::cout << "   Transfer limit in round 1: "
                              << String::secToString(transferLimit - sourceDepartureTime) << std::endl;
                }
            }

            if (isStop(currentVertex)) {
                stopsUpdatedByTransfer.insert(StopId(currentVertex));
            }

            if (currentLabel->arrivalTime > transferLimit) break;
        }

        stopsUpdatedByRoute.clear();
    }

    inline void finalDijkstra() noexcept {
        Assert(stopsUpdatedByTransfer.empty(), "stopsUpdatedByTransfer is not empty!");

        for (const StopId stop : stopsUpdatedByRoute) {
            twoTripsQueue.update(&twoTripsArrivalLabels[stop]);
            const StopId routeParent = twoTripsRouteParent[stop];
            if (isStop(routeParent)) {
                if (!shortcutDestinationCandidates.contains(routeParent)) {
                    shortcutDestinationCandidates.insert(routeParent);
                }
                shortcutDestinationCandidates[routeParent].insert(stop);
            }
        }

        while (!twoTripsQueue.empty()) {
            ArrivalLabel* currentLabel = twoTripsQueue.extractFront();
            const Vertex currentVertex = Vertex(currentLabel - &twoTripsArrivalLabels[0]);

            for (const Edge edge : graph.edgesFrom(currentVertex)) {
                const Vertex neighborVertex = graph.get(ToVertex, edge);
                const int walkTime = graph.getWalkArrivalFrom(edge, 0);
                if (walkTime == never) continue;

                const int newArrivalTime = currentLabel->arrivalTime + walkTime;
                if (newArrivalTime < twoTripsArrivalLabels[neighborVertex].arrivalTime) {
                    arrivalByEdge2(neighborVertex, newArrivalTime, currentVertex);
                }
            }

            if (isStop(currentVertex)) {
                const StopId routeParent = twoTripsRouteParent[currentVertex];
                if (isStop(routeParent)) {
                    const StopId transferParent = oneTripTransferParent[routeParent];
                    const int walkingDistance = oneTripArrivalLabels[routeParent].arrivalTime -
                                                oneTripArrivalLabels[transferParent].arrivalTime;

                    if constexpr (IgnoreIsolatedCandidates) {
                        if (directTransferArrivalLabels[currentVertex].arrivalTime < never) {
                            shortcuts.emplace_back(transferParent, routeParent, walkingDistance);
                        }
                    } else {
                        shortcuts.emplace_back(transferParent, routeParent, walkingDistance);
                    }

                    Assert(shortcutDestinationCandidates.contains(routeParent),
                           "Route parent not in shortcutDestinationCandidates!");

                    if constexpr (!CountOptimalCandidates) {
                        for (const StopId obsoleteCandidate : shortcutDestinationCandidates[routeParent]) {
                            twoTripsRouteParent[obsoleteCandidate] = noStop;
                        }
                        shortcutDestinationCandidates.remove(routeParent);
                    } else {
                        twoTripsRouteParent[currentVertex] = noStop;
                        shortcutDestinationCandidates[routeParent].erase(StopId(currentVertex));
                        if (shortcutDestinationCandidates[routeParent].empty()) {
                            shortcutDestinationCandidates.remove(routeParent);
                        }
                    }
                }
            }

            if (shortcutDestinationCandidates.empty()) break;
        }

        Assert(shortcutDestinationCandidates.empty(),
               "There are still shortcut destination candidates left!");
        stopsUpdatedByRoute.clear();
    }

    //==========================================================================
    // ARRIVAL FUNCTIONS
    //==========================================================================

    template<int ROUND>
    inline int arrivalTime(const Vertex vertex) const noexcept {
        static_assert((ROUND == 0) | (ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 0) {
            Assert(isStop(vertex), "Arrival time in round 0 only available for stops!");
            return zeroTripsArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 1) {
            return oneTripArrivalLabels[vertex].arrivalTime;
        } else {
            return twoTripsArrivalLabels[vertex].arrivalTime;
        }
    }

    template<int ROUND>
    inline void arrivalByRoute(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        static_assert((ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 1) {
            arrivalByRoute1(stop, arrivalTime, parent);
        } else {
            arrivalByRoute2(stop, arrivalTime, parent);
        }
    }

    inline void arrivalByRoute1(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        // Mark journey as candidate or witness
        if (stationOfStop[parent].representative == sourceStation.representative) {
            oneTripTransferParent[stop] = stop;
        } else {
            oneTripTransferParent[stop] = noStop;
        }

        updateArrival<1>(stop, arrivalTime, timestamp);

        if (oneTripArrivalLabels[stop].isOnHeap()) {
            oneTripQueue.remove(&oneTripArrivalLabels[stop]);
        }

        if (twoTripsArrivalLabels[stop].arrivalTime > arrivalTime) {
            updateArrival<2>(stop, arrivalTime, timestamp);
            if (twoTripsArrivalLabels[stop].isOnHeap()) {
                twoTripsQueue.remove(&twoTripsArrivalLabels[stop]);
            }
        }

        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByRoute2(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        // Mark journey as candidate or witness
        if (isStop(oneTripTransferParent[parent]) &&
            (oneTripTransferParent[parent] != parent) &&
            (!shortcutAlreadyExists(parent))) {
            twoTripsRouteParent[stop] = parent;
        } else {
            twoTripsRouteParent[stop] = noStop;
        }

        updateArrival<2>(stop, arrivalTime, timestamp);

        if (twoTripsArrivalLabels[stop].isOnHeap()) {
            twoTripsQueue.remove(&twoTripsArrivalLabels[stop]);
        }

        stopsUpdatedByRoute.insert(stop);
    }

    inline bool shortcutAlreadyExists(const StopId parent) const noexcept {
        if constexpr (!CountOptimalCandidates) {
            return shortcutGraph.hasEdge(oneTripTransferParent[parent], parent);
        } else {
            suppressUnusedParameterWarning(parent);
            return false;
        }
    }

    inline void arrivalByEdge0(const Vertex vertex, const int arrivalTime) noexcept {
        updateArrival<0>(vertex, arrivalTime, timestamp);

        if (oneTripArrivalLabels[vertex].arrivalTime > arrivalTime) {
            updateArrival<1>(vertex, arrivalTime, timestamp);
            if (oneTripArrivalLabels[vertex].isOnHeap()) {
                oneTripQueue.remove(&oneTripArrivalLabels[vertex]);
            }
            if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
                updateArrival<2>(vertex, arrivalTime, timestamp);
                if (twoTripsArrivalLabels[vertex].isOnHeap()) {
                    twoTripsQueue.remove(&twoTripsArrivalLabels[vertex]);
                }
            }
        }
    }

    inline void arrivalByEdge1(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        if (isShortcutCandidate(vertex)) shortcutCandidatesInQueue--;
        if (isStop(oneTripTransferParent[parent])) shortcutCandidatesInQueue++;

        oneTripTransferParent[vertex] = oneTripTransferParent[parent];
        updateArrival<1>(vertex, arrivalTime, oneTripTimestamps[parent]);

        if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
            updateArrival<2>(vertex, arrivalTime, oneTripTimestamps[parent]);
            if (twoTripsArrivalLabels[vertex].isOnHeap()) {
                twoTripsQueue.remove(&twoTripsArrivalLabels[vertex]);
            }
        }

        oneTripQueue.update(&oneTripArrivalLabels[vertex]);
    }

    inline void arrivalByEdge2(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        updateArrival<2>(vertex, arrivalTime, twoTripsTimestamps[parent]);
        twoTripsQueue.update(&twoTripsArrivalLabels[vertex]);

        if (!isStop(vertex)) return;

        const StopId routeParent = twoTripsRouteParent[vertex];
        if (isStop(routeParent)) {
            Assert(shortcutDestinationCandidates.contains(routeParent), "Invalid routeParent!");
            Assert(shortcutDestinationCandidates[routeParent].contains(StopId(vertex)), "Vertex not in list!");
            shortcutDestinationCandidates[routeParent].erase(StopId(vertex));
            if (shortcutDestinationCandidates[routeParent].empty()) {
                shortcutDestinationCandidates.remove(routeParent);
            }
        }
        twoTripsRouteParent[vertex] = noStop;
    }

    template<int ROUND>
    inline void updateArrival(const Vertex vertex, const int arrivalTime, const uint16_t labelTimestamp) noexcept {
        if constexpr (ROUND == 0) {
            zeroTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            suppressUnusedParameterWarning(labelTimestamp);
        } else if constexpr (ROUND == 1) {
            oneTripArrivalLabels[vertex].arrivalTime = arrivalTime;
            oneTripTimestamps[vertex] = labelTimestamp;
        } else {
            twoTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            twoTripsTimestamps[vertex] = labelTimestamp;
        }
    }

    inline bool isShortcutCandidate(const Vertex vertex) const noexcept {
        return oneTripArrivalLabels[vertex].isOnHeap() && isStop(oneTripTransferParent[vertex]);
    }

private:
    //==========================================================================
    // DATA
    //==========================================================================

    const Intermediate::Data& data;
    const TimeDependentGraph& graph;
    DynamicTransferGraph& shortcutGraph;

    const size_t numberOfStops;
    const size_t numberOfVertices;
    const int witnessTransferLimit;
    const int earliestDepartureTime;

    // Route structures
    std::vector<Route> routes;
    std::vector<std::vector<RouteSegment>> routesContainingStop;

    std::vector<Station> stationOfStop;
    Station sourceStation;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> directTransferQueue;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<ArrivalLabel> zeroTripsArrivalLabels;

    std::vector<ArrivalLabel> oneTripArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> oneTripQueue;
    std::vector<uint16_t> oneTripTimestamps;

    std::vector<ArrivalLabel> twoTripsArrivalLabels;
    ExternalKHeap<2, ArrivalLabel> twoTripsQueue;
    std::vector<uint16_t> twoTripsTimestamps;

    std::vector<StopId> oneTripTransferParent;
    std::vector<StopId> twoTripsRouteParent;

    int shortcutCandidatesInQueue;
    IndexedMap<std::set<StopId>, false, StopId> shortcutDestinationCandidates;
    std::vector<Shortcut> shortcuts;
    size_t optimalCandidates;

    IndexedMap<StopIndex, false, size_t> routesServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    uint16_t timestamp;
};

}  // namespace RAPTOR::ULTRA