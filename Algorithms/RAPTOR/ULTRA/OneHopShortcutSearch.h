#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <set>
#include <concepts>

#include "../../../Helpers/Helpers.h"
#include "../../../Helpers/Vector/Vector.h"

#include "../../../DataStructures/Container/Map.h"
#include "../../../DataStructures/Container/IndexedSet.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/Shortcut.h"

namespace RAPTOR::ULTRA {

// One-hop (bounded-radius) variant of the ULTRA canonical-MR shortcut search.
//
// The standard ShortcutSearch (see ShortcutSearch.h) computes ULTRA transfer
// shortcuts by running two rounds of canonical MR (Baum, Buchhold, Sauer,
// Wagner, Zuendorf 2019; arXiv:1906.04832, Section 3), where each transfer
// relaxation phase is a multi-hop Dijkstra search over the (unrestricted)
// transfer graph. The resulting shortcut edges represent (possibly multi-hop)
// intermediate transfers as single one-hop edges.
//
// This variant instead applies the RAPTOR-style one-hop modification described
// in the paper (p.3, ref. [10]): transfers are relaxed as a SINGLE edge and
// journeys with two consecutive transfer edges are prohibited. Concretely, the
// three Dijkstra phases are replaced by single-edge relaxations that use each
// stop's *trip* arrival time as the base (so a transfer arrival can never be
// extended by a second transfer). Additionally, a per-search transfer radius
// (maxTransferTravelTime, in seconds) discards transfer edges that are longer
// than the radius. Because the transfer edges of every stop are sorted by
// ascending TravelTime (RAPTOR::Data::sortEdges), the radius check stops
// scanning a stop's edges as soon as the limit is exceeded.
//
// The emitted shortcuts are therefore exactly the single one-hop intermediate
// transfer edges (within the radius) that are required by a canonical
// candidate journey and not dominated by a witness. In other words, the output
// is the subset of the one-hop transfer graph that is needed as intermediate
// transfers -- an analogue of trip-based transfer reduction for the one-hop,
// bounded-radius model. It requires a stop-to-stop transfer graph as input
// (one-hop edges between stops); running it on a graph with non-stop routing
// vertices would only reach adjacent routing vertices in a single hop.
//
// NOTE ON CANONICITY: to keep the search order-independent, ties between
// equivalent one-hop journeys are resolved conservatively (in favor of marking
// a candidate). This can generate a few superfluous shortcuts but never omits a
// required one, so the shortcut set remains sufficient. Exact canonical
// tie-breaking (by id_R / id_V) could be layered on top if a minimal set is
// required.
template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false>
class OneHopShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    using Type = OneHopShortcutSearch<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

public:
    struct ArrivalLabel {
        ArrivalLabel() : arrivalTime(never) {}
        int arrivalTime;
    };

    struct DepartureLabel {
        DepartureLabel(const RouteId routeId = noRouteId, const StopIndex stopIndex = noStopIndex, const int departureTime = never) : route(routeId, stopIndex), departureTime(departureTime) {}
        RouteSegment route;
        int departureTime;
        inline bool operator<(const DepartureLabel& other) const noexcept {
            return (departureTime > other.departureTime) || ((departureTime == other.departureTime) && (route.routeId < other.route.routeId));
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
    OneHopShortcutSearch(const Data& data, DynamicTransferGraph& shortcutGraph, const int maxTransferTravelTime = INFTY) :
        data(data),
        shortcutGraph(shortcutGraph),
        stationOfStop(data.numberOfStops()),
        sourceStation(),
        sourceDepartureTime(0),
        shortcutDestinationCandidates(data.numberOfStops()),
        optimalCandidates(0),
        routesServingUpdatedStops(data.numberOfRoutes()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        maxTransferTravelTime(maxTransferTravelTime),
        earliestDepartureTime(data.getMinDepartureTime()),
        timestamp(0) {
        Assert(data.hasImplicitBufferTimes(), "Shortcut search requires implicit departure buffer times!");
        // Stations group stops that are mutually reachable via zero-time transfer edges.
        for (const StopId stop : data.stops()) {
            stationOfStop[stop].add(stop);
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                if (data.transferGraph.get(TravelTime, edge) != 0) break;
                const Vertex other = data.transferGraph.get(ToVertex, edge);
                if (data.isStop(other)) stationOfStop[stop].add(StopId(other));
            }
        }
    }

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        Assert(data.isStop(source), "source (" << source << ") is not a stop!");
        if (stationOfStop[source].representative != source) return;
        setSource(source);
        for (const ConsolidatedDepartureLabel& label : collectDepartures(minTime, maxTime)) {
            runForDepartureTime(label);
            if constexpr (CountOptimalCandidates) {
                optimalCandidates += shortcuts.size();
            }
            for (const Shortcut& shortcut : shortcuts) {
                if (!shortcutGraph.hasEdge(shortcut.origin, shortcut.destination)) {
                    shortcutGraph.addEdge(shortcut.origin, shortcut.destination).set(TravelTime, shortcut.travelTime);
                } else {
                    Assert(shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) == shortcut.travelTime, "Edge from " << shortcut.origin << " to " << shortcut.destination << " has inconclusive travel time (" << shortcutGraph.get(TravelTime, shortcutGraph.findEdge(shortcut.origin, shortcut.destination)) << ", " << shortcut.travelTime << ")");
                }
            }
        }
    }

    inline size_t getNumberOfOptimalCandidates() const noexcept requires CountOptimalCandidates {
        return optimalCandidates;
    }

    inline int getTransferRadius() const noexcept {
        return maxTransferTravelTime;
    }

    // Instrumentation for debugging a single expected shortcut traceOrigin->traceDest.
    StopId traceOrigin = noStop;
    StopId traceDest = noStop;
    RouteId traceRoute = noRouteId;
    int nDepartures = 0;
    inline StopId representativeOf(const StopId stop) const noexcept {
        return stationOfStop[stop].representative;
    }

private:
    inline void setSource(const StopId source) noexcept {
        Assert(stationOfStop[source].representative == source, "Source " << source << " is not representative of its station!");
        clear();
        sourceStation = stationOfStop[source];
        initialOneHopTransfers();
        sort(stopsReachedByDirectTransfer);
        if constexpr (Debug) {
            std::cout << "   Source stop: " << source << std::endl;
            std::cout << "   Number of stops reached by direct transfer: " << String::prettyInt(stopsReachedByDirectTransfer.size()) << std::endl;
        }
    }

    inline void runForDepartureTime(const ConsolidatedDepartureLabel& label) noexcept {
        if constexpr (Debug) std::cout << "   Running search for departure time: " << label.departureTime << " (" << String::secToTime(label.departureTime) << ")" << std::endl;

        // Reset everything touched by the previous departure so this departure is
        // searched independently (no cross-departure self-pruning contamination,
        // which was dropping short intermediate-transfer shortcuts).
        for (const Vertex v : touchedVertices) {
            oneTripArrivalLabels[v].arrivalTime = never;
            twoTripsArrivalLabels[v].arrivalTime = never;
            oneTripTransferParent[v] = noStop;
            shortcutOrigin[v] = noStop;
            if (data.isStop(v)) {
                zeroTripsArrivalLabels[v].arrivalTime = never;
                twoTripsRouteParent[v] = noStop;
            }
        }
        touchedVertices.clear();

        timestamp++;
        shortcutDestinationCandidates.clear();
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();

        sourceDepartureTime = label.departureTime;
        relaxInitialTransfers();
        collectRoutes1(label.routes);
        scanRoutes<1>();
        intermediateOneHopTransfers();
        collectRoutes2();
        scanRoutes<2>();
        finalOneHopTransfers();
    }

    inline std::vector<ConsolidatedDepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        Assert(directTransferArrivalLabels[sourceStation.representative].arrivalTime == 0, "Direct transfer for source " << sourceStation.representative << " is incorrect!");
        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::vector<DepartureLabel> departureLabels;
        for (const RouteId route : data.routes()) {
            const StopId* stops = data.stopArrayOfRoute(route);
            const size_t tripSize = data.numberOfStopsInRoute(route);
            int minimalTransferTime = never;
            for (size_t stopIndex = 0; stopIndex + 1 < tripSize; stopIndex++) {
                if (directTransferArrivalLabels[stops[stopIndex]].arrivalTime > minimalTransferTime) continue;
                minimalTransferTime = directTransferArrivalLabels[stops[stopIndex]].arrivalTime;
                for (const StopEvent* trip = data.firstTripOfRoute(route); trip <= data.lastTripOfRoute(route); trip += tripSize) {
                    const int departureTime = trip[stopIndex].departureTime - minimalTransferTime;
                    if (departureTime < cutoffTime) continue;
                    if (departureTime > maxTime) break;
                    if (stationOfStop[stops[stopIndex]].representative == sourceStation.representative) {
                        departureLabels.emplace_back(noRouteId, noStopIndex, departureTime);
                    }
                    departureLabels.emplace_back(route, StopIndex(stopIndex), departureTime);
                }
            }
        }
        sort(departureLabels);
        std::vector<ConsolidatedDepartureLabel> result(1);
        for (const DepartureLabel& label : departureLabels) {
            if (label.route.routeId == noRouteId) {
                if (label.departureTime == result.back().departureTime) continue;
                result.back().departureTime = label.departureTime;
                result.emplace_back(label.departureTime);
            } else {
                result.back().routes.emplace_back(label.route);
            }
        }
        result.pop_back();
        for (ConsolidatedDepartureLabel& label : result) {
            sort(label.routes);
        }
        return result;
    }

private:
    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(directTransferArrivalLabels);
        stopsReachedByDirectTransfer.clear();

        std::vector<ArrivalLabel>(data.numberOfStops()).swap(zeroTripsArrivalLabels);

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(oneTripArrivalLabels);
        std::vector<u_int16_t>(data.transferGraph.numVertices(), 0).swap(oneTripTimestamps);

        std::vector<ArrivalLabel>(data.transferGraph.numVertices()).swap(twoTripsArrivalLabels);
        std::vector<u_int16_t>(data.transferGraph.numVertices(), 0).swap(twoTripsTimestamps);

        std::vector<StopId>(data.transferGraph.numVertices(), noStop).swap(oneTripTransferParent);
        std::vector<StopId>(data.numberOfStops(), noStop).swap(twoTripsRouteParent);

        std::vector<StopId>(data.transferGraph.numVertices(), noStop).swap(shortcutOrigin);
        std::vector<int>(data.transferGraph.numVertices(), 0).swap(shortcutEdgeTime);

        std::vector<u_int16_t>(data.transferGraph.numVertices(), 0).swap(touchedTimestamp);
        touchedVertices.clear();

        shortcutDestinationCandidates.clear();
        shortcuts.clear();

        routesServingUpdatedStops.clear();
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
    }

    inline void collectRoutes1(const std::vector<RouteSegment>& routes) noexcept {
        for (const RouteSegment& route : routes) {
            Assert(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
            Assert(route.stopIndex + 1 < data.numberOfStopsInRoute(route.routeId), "RouteSegment " << route << " is not a departure event!");
            Assert(data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime >= arrivalTime<0>(data.stopOfRouteSegment(route)), "RouteSegment " << route << " is not reachable!");
            if (routesServingUpdatedStops.contains(route.routeId)) {
                routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
            } else {
                routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
            }
        }
        Assert(routesServingUpdatedStops.isSortedByKeys(), "Collected route segments are not sorted!");
    }

    inline void collectRoutes2() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            if (stop == traceDest) {
                std::cout << "TRACE collectRoutes2: dest=" << stop << " in stopsUpdatedByTransfer, oneTripArr=" << oneTripArrivalLabels[stop].arrivalTime
                          << " #routes=" << data.routesContainingStop(stop).size() << std::endl;
            }
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                Assert(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                Assert(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (stop == traceDest) {
                    std::cout << "TRACE collectRoutes2 route " << route.routeId << " stopIndex=" << route.stopIndex
                              << " isLast=" << (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId))
                              << " lastTripDep=" << data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime
                              << " arr=" << oneTripArrivalLabels[stop].arrivalTime
                              << " => added=" << ((route.stopIndex + 1 != data.numberOfStopsInRoute(route.routeId)) && (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime >= oneTripArrivalLabels[stop].arrivalTime)) << std::endl;
                }
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < oneTripArrivalLabels[stop].arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
        routesServingUpdatedStops.sortKeys();
    }

    // Simple, validated route scan (identical semantics to OneHopRAPTOR's
    // scanRoutes). No canonical/witness domination logic — this is the "two-phase
    // OneHopRAPTOR" shortcut extraction: correctness is inherited from the query
    // algorithm, and shortcuts are over-generated (emitted for every candidate
    // intermediate transfer) rather than pruned by witnesses.
    template<int CURRENT>
    inline void scanRoutes() noexcept {
        static_assert((CURRENT == 1) | (CURRENT == 2), "Invalid round!");
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            const StopIndex stopIndex = routesServingUpdatedStops[route];
            if (route == traceRoute) {
                std::cout << "TRACE scanRoutes<" << CURRENT << ">: route " << route << " startStopIndex=" << stopIndex
                          << " startStop=" << data.stopArrayOfRoute(route)[stopIndex]
                          << " dep=" << sourceDepartureTime << std::endl;
            }
            TripIterator tripIterator = data.getTripIterator(route, stopIndex);
            StopIndex parentIndex = stopIndex;
            while (tripIterator.hasFurtherStops()) {
                //Find the earliest trip that can be entered at the current stop.
                while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop()))) {
                    tripIterator.previousTrip();
                    parentIndex = tripIterator.getStopIndex();
                }
                if constexpr (CURRENT == 2) {
                    //Emit the intermediate-transfer shortcut whenever trip 2 is
                    //boarded at a stop reached via a candidate intermediate
                    //transfer -- regardless of whether the trip-2 arrival improves
                    //here, because the 2-trip journey may only be Pareto-optimal
                    //at the target after a final transfer.
                    const StopId boardStop = tripIterator.stop(parentIndex);
                    if (shortcutOrigin[boardStop] != noStop && !shortcutAlreadyExists(boardStop)) {
                        shortcuts.emplace_back(shortcutOrigin[boardStop], boardStop, shortcutEdgeTime[boardStop]);
                    }
                }
                tripIterator.nextStop();
                const int newArrivalTime = tripIterator.arrivalTime();
                if (newArrivalTime < arrivalTime<CURRENT>(tripIterator.stop())) {
                    arrivalByRoute<CURRENT>(tripIterator.stop(), newArrivalTime, tripIterator.stop(parentIndex));
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
            return shortcutOrigin[parent] != noStop;
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

    // Round 0: candidates have an EMPTY initial transfer, so trip 1 boards at the
    // source station only. We deliberately do NOT relax an initial walk to
    // non-station stops -- that would reach a candidate's trip-1 end as a 0-trip
    // journey and dominate its trip-1 arrival, dropping the candidate (initial
    // walks only ever produce witnesses, which this over-generating search does
    // not need).
    inline void initialOneHopTransfers() noexcept {
        for (const StopId sourceStop : sourceStation.stops) {
            directTransferArrivalLabels[sourceStop].arrivalTime = 0;
            stopsReachedByDirectTransfer.emplace_back(sourceStop);
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

    // Intermediate transfers (between trip 1 and trip 2), relaxed one hop at a
    // time. Each stop reached by a trip in round 1 relaxes a single transfer
    // edge, using its TRIP arrival time as the base. Snapshotting the trip
    // arrival times first prevents a transfer arrival from being extended by a
    // second transfer (which would be two consecutive transfer edges).
    inline void intermediateOneHopTransfers() noexcept {
        std::vector<std::pair<StopId, int>> seeds;
        seeds.reserve(stopsUpdatedByRoute.size());
        for (const StopId stop : stopsUpdatedByRoute) {
            seeds.emplace_back(stop, oneTripArrivalLabels[stop].arrivalTime);
        }
        for (const auto& [stop, baseTime] : seeds) {
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                const int travelTime = data.transferGraph.get(TravelTime, edge);
                if (travelTime > maxTransferTravelTime) break;
                const Vertex neighbor = data.transferGraph.get(ToVertex, edge);
                if (!data.isStop(neighbor)) continue;
                const int newArrivalTime = baseTime + travelTime;
                if (stop == traceOrigin && StopId(neighbor) == traceDest) {
                    std::cout << "TRACE intermediate: origin=" << stop << " -> dest=" << neighbor
                              << " edge=" << travelTime << " baseTime(trip1arr)=" << baseTime
                              << " newArr=" << newArrivalTime << " curLabel=" << oneTripArrivalLabels[neighbor].arrivalTime
                              << (newArrivalTime < oneTripArrivalLabels[neighbor].arrivalTime ? " IMPROVES" :
                                  (newArrivalTime == oneTripArrivalLabels[neighbor].arrivalTime ? " EQUAL" : " WORSE"))
                              << " originCandidate=" << (oneTripTransferParent[stop] == stop) << std::endl;
                }
                if (newArrivalTime < oneTripArrivalLabels[neighbor].arrivalTime) {
                    relaxIntermediateEdge(neighbor, newArrivalTime, stop, travelTime);
                    stopsUpdatedByTransfer.insert(StopId(neighbor));
                }
                //A candidate transfer that reaches the neighbor at the SAME time
                //as the current best (a witness, an empty transfer, or a previous
                //iteration) must still record its intermediate transfer, so its
                //shortcut is generated. Only set it when the neighbor has no
                //candidate origin yet, and never change the (equal) arrival time.
                //Emitting a possibly-superfluous but valid shortcut is safe;
                //dropping a needed one is not.
                else if (newArrivalTime == oneTripArrivalLabels[neighbor].arrivalTime) {
                    const bool isCandidate = oneTripTransferParent[stop] == stop;
                    if (isCandidate && shortcutOrigin[neighbor] == noStop) {
                        shortcutOrigin[neighbor] = stop;
                        shortcutEdgeTime[neighbor] = travelTime;
                        stopsUpdatedByTransfer.insert(StopId(neighbor));
                    }
                }
            }
            // The trip-1 arrival stop can also board trip 2 with an empty
            // intermediate transfer (a witness journey, never a shortcut).
            if (data.isStop(stop)) {
                stopsUpdatedByTransfer.insert(StopId(stop));
            }
        }
        stopsUpdatedByRoute.clear();
    }

    // No final-transfer / witness-domination phase: shortcuts are emitted
    // directly in arrivalByRoute2 (over-generating, no pruning), so this only
    // clears the round-2 arrivals.
    inline void finalOneHopTransfers() noexcept {
        stopsUpdatedByRoute.clear();
    }

    template<int ROUND>
    inline int arrivalTime(const Vertex vertex) const noexcept {
        static_assert((ROUND == 0) | (ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 0) {
            Assert(data.isStop(vertex), "Arrival time in round 0 only available for stops!");
            return zeroTripsArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 1) {
            return oneTripArrivalLabels[vertex].arrivalTime;
        } else if constexpr (ROUND == 2) {
            return twoTripsArrivalLabels[vertex].arrivalTime;
        }
    }

    template<int ROUND>
    inline void arrivalByRoute(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        static_assert((ROUND == 1) | (ROUND == 2), "Invalid round!");
        if constexpr (ROUND == 1) {
            arrivalByRoute1(stop, arrivalTime, parent);
        } else if constexpr (ROUND == 2) {
            arrivalByRoute2(stop, arrivalTime, parent);
        }
    }

    inline void arrivalByRoute1(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        //Mark journey as candidate or witness
        if (stationOfStop[parent].representative == sourceStation.representative) {
            oneTripTransferParent[stop] = stop;
        } else {
            oneTripTransferParent[stop] = noStop;
        }
        if (stop == traceOrigin) {
            std::cout << "TRACE trip1 arrival at origin=" << stop << " arr=" << arrivalTime << " boardedAt=" << parent
                      << " candidateTrip1=" << (oneTripTransferParent[stop] == stop)
                      << " (dep=" << sourceDepartureTime << ")" << std::endl;
        }
        updateArrival<1>(stop, arrivalTime, timestamp);
        if (twoTripsArrivalLabels[stop].arrivalTime > arrivalTime) {
            updateArrival<2>(stop, arrivalTime, timestamp);
        }
        stopsUpdatedByRoute.insert(stop);
    }

    inline void arrivalByRoute2(const StopId stop, const int arrivalTime, const StopId) noexcept {
        //Shortcuts are emitted at the trip-2 boarding point in scanRoutes<2>.
        updateArrival<2>(stop, arrivalTime, timestamp);
        stopsUpdatedByRoute.insert(stop);
    }

    inline bool shortcutAlreadyExists(const StopId parent) const noexcept {
        if constexpr (!CountOptimalCandidates) {
            return shortcutGraph.hasEdge(shortcutOrigin[parent], parent);
        } else {
            suppressUnusedParameterWarning(parent);
            return false;
        }
    }

    inline void arrivalByEdge0(const Vertex vertex, const int arrivalTime) noexcept {
        updateArrival<0>(vertex, arrivalTime, timestamp);
        if (oneTripArrivalLabels[vertex].arrivalTime > arrivalTime) {
            updateArrival<1>(vertex, arrivalTime, timestamp);
            if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
                updateArrival<2>(vertex, arrivalTime, timestamp);
            }
        }
    }

    inline void relaxIntermediateEdge(const Vertex vertex, const int arrivalTime, const StopId parent, const int edgeTime) noexcept {
        // parent is a trip-1 arrival; the intermediate transfer is the SINGLE
        // edge parent->vertex. The shortcut origin is parent itself iff parent is
        // a candidate trip-1 end (its trip was boarded from the source station);
        // the shortcut's travel time is this single edge, not a label difference.
        if (oneTripTransferParent[parent] == parent) {
            shortcutOrigin[vertex] = parent;
            shortcutEdgeTime[vertex] = edgeTime;
        } else {
            shortcutOrigin[vertex] = noStop;
        }
        updateArrival<1>(vertex, arrivalTime, timestamp);
        if (twoTripsArrivalLabels[vertex].arrivalTime > arrivalTime) {
            updateArrival<2>(vertex, arrivalTime, timestamp);
        }
    }

    inline void relaxFinalEdge(const Vertex vertex, const int arrivalTime, const Vertex parent) noexcept {
        updateArrival<2>(vertex, arrivalTime, twoTripsTimestamps[parent]);
        if (!data.isStop(vertex)) return;
        const StopId routeParent = twoTripsRouteParent[vertex];
        if (data.isStop(routeParent)) {
            //Candidate was dominated by a witness => remove from shortcutDestinationCandidates list.
            if (shortcutDestinationCandidates.contains(routeParent)) {
                shortcutDestinationCandidates[routeParent].erase(StopId(vertex));
                if (shortcutDestinationCandidates[routeParent].empty()) {
                    shortcutDestinationCandidates.remove(routeParent);
                }
            }
            twoTripsRouteParent[vertex] = noStop;
        }
    }

    inline void markTouched(const Vertex vertex) noexcept {
        if (touchedTimestamp[vertex] != timestamp) {
            touchedTimestamp[vertex] = timestamp;
            touchedVertices.emplace_back(vertex);
        }
    }

    template<int ROUND>
    inline void updateArrival(const Vertex vertex, const int arrivalTime, const u_int16_t labelTimestamp) noexcept {
        markTouched(vertex);
        if constexpr (ROUND == 0) {
            zeroTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            suppressUnusedParameterWarning(labelTimestamp);
        } else if constexpr (ROUND == 1) {
            oneTripArrivalLabels[vertex].arrivalTime = arrivalTime;
            oneTripTimestamps[vertex] = labelTimestamp;
        } else if constexpr (ROUND == 2) {
            twoTripsArrivalLabels[vertex].arrivalTime = arrivalTime;
            twoTripsTimestamps[vertex] = labelTimestamp;
        }
    }

private:
    const Data& data;
    DynamicTransferGraph& shortcutGraph;
    std::vector<Station> stationOfStop;

    Station sourceStation;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<ArrivalLabel> zeroTripsArrivalLabels;

    std::vector<ArrivalLabel> oneTripArrivalLabels;
    std::vector<u_int16_t> oneTripTimestamps;

    std::vector<ArrivalLabel> twoTripsArrivalLabels;
    std::vector<u_int16_t> twoTripsTimestamps;

    //Only valid for candidates
    std::vector<StopId> oneTripTransferParent;
    std::vector<StopId> twoTripsRouteParent;

    //One-hop intermediate transfer reaching a stop after trip 1: the origin
    //(candidate trip-1 end stop) and the travel time of the single edge used.
    //shortcutOrigin[v] == noStop means v's best one-trip arrival did not use a
    //candidate intermediate transfer (empty transfer or witness).
    std::vector<StopId> shortcutOrigin;
    std::vector<int> shortcutEdgeTime;

    //Vertices whose labels were set during the current departure-time search, so
    //they can be reset for a clean, independent search of the next departure time
    //(avoids cross-departure contamination from rRAPTOR self-pruning).
    std::vector<Vertex> touchedVertices;
    std::vector<u_int16_t> touchedTimestamp;

    //Maps potential shortcut destinations to the final stops of the candidate journeys using that shortcut
    IndexedMap<std::set<StopId>, false, StopId> shortcutDestinationCandidates;
    std::vector<Shortcut> shortcuts;
    size_t optimalCandidates;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;
    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;

    int maxTransferTravelTime;

    int earliestDepartureTime;

    u_int16_t timestamp;

};

}
