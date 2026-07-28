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

namespace RAPTOR::FILTRA {

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
// EXPLORE_ENDPOINT_TRANSFERS selects between two variants:
//   true  -- full canonical MR: walk -> trip -> walk -> trip -> walk. The initial
//            walk lets trip 1 be boarded away from the source; the final walk
//            witness-prunes candidates. Smaller shortcut set, slower to build.
//   false -- endpoint-optimised: trip -> walk -> trip. Skips the initial and final
//            walks, relying on every stop being used as a source, so a journey
//            that walks before boarding is covered by the search from its boarding
//            stop. Larger (over-generated) set, much faster to build.
// Either way only intermediate (alight, board) shortcuts are emitted; first/last
// mile is answered at query time from the stop-to-stop graph.
template<bool DEBUG = false, bool COUNT_OPTIMAL_CANDIDATES = false, bool IGNORE_ISOLATED_CANDIDATES = false, bool EXPLORE_ENDPOINT_TRANSFERS = true>
class ShortcutSearch {

public:
    inline static constexpr bool Debug = DEBUG;
    inline static constexpr bool CountOptimalCandidates = COUNT_OPTIMAL_CANDIDATES;
    inline static constexpr bool IgnoreIsolatedCandidates = IGNORE_ISOLATED_CANDIDATES;
    inline static constexpr bool ExploreEndpointTransfers = EXPLORE_ENDPOINT_TRANSFERS;
    using Type = ShortcutSearch<Debug, CountOptimalCandidates, IgnoreIsolatedCandidates>;

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

public:
    ShortcutSearch(const Data& data, DynamicTransferGraph& shortcutGraph, const int maxTransferTravelTime = INFTY) :
        data(data),
        shortcutGraph(shortcutGraph),
        sourceStop(noStop),
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
    }

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        Assert(data.isStop(source), "source (" << source << ") is not a stop!");
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
        return stop;
    }

private:
    inline void setSource(const StopId source) noexcept {
        clear();
        sourceStop = source;
        initializeSource();
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
        Assert(directTransferArrivalLabels[sourceStop].arrivalTime == 0, "Direct transfer for source " << sourceStop << " is incorrect!");
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
                    if (stops[stopIndex] == sourceStop) {
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
        sourceStop = noStop;

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

    // Simple, validated route scan (identical semantics to the one-hop query
    // algorithm's scanRoutes, i.e. RAPTOR<..., TRANSITIVE=false, ...>). No
    // canonical/witness domination logic — this is the "two-phase one-hop RAPTOR"
    // shortcut extraction: correctness is inherited from the query algorithm, and
    // shortcuts are over-generated (emitted for every candidate intermediate
    // transfer) rather than pruned by witnesses.
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
                //Find earliest trip that can be entered
                if (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop()))) {
                    do {
                        tripIterator.previousTrip();
                    } while (tripIterator.hasEarlierTrip() && (tripIterator.previousDepartureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())));
                    if (!stopsUpdatedByTransfer.contains(tripIterator.stop())) {
                        //Trip was improved by an arrival that was found during a previous RAPTOR iteration.
                        //We already explored this trip during that iteration.
                        //Fast forward to the next stop that was updated in the current iteration and can enter the current trip.
                        do {
                            tripIterator.nextStop();
                        } while (tripIterator.hasFurtherStops() && ((!stopsUpdatedByTransfer.contains(tripIterator.stop())) || (tripIterator.departureTime() < arrivalTime<CURRENT - 1>(tripIterator.stop()))));
                        parentIndex = tripIterator.getStopIndex();
                        continue;
                    }
                    parentIndex = tripIterator.getStopIndex();
                }
                //Candidates may dominate equivalent labels from previous iterations
                else if (stopsUpdatedByTransfer.contains(tripIterator.stop()) && !isFromCurrentIteration<CURRENT - 1>(tripIterator.stop(parentIndex)) && isCandidate<CURRENT>(tripIterator.stop()) && tripIterator.departureTime() >= arrivalTime<CURRENT - 1>(tripIterator.stop())) {
                    parentIndex = tripIterator.getStopIndex();
                }
                if constexpr (CURRENT == 2 && !ExploreEndpointTransfers) {
                    //Endpoint-optimised variant: emit the intermediate shortcut at
                    //the trip-2 BOARDING point, unconditionally. The two-trip
                    //journey may only be Pareto-optimal after a later leg (e.g. a
                    //final walk), so its trip-2 arrival need not improve here.
                    //Gating emission on arrival improvement drops such shortcuts.
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
                //Candidates may dominate equivalent labels from previous iterations
                else if (newArrivalTime == arrivalTime<CURRENT>(tripIterator.stop()) && !isFromCurrentIteration<CURRENT>(tripIterator.stop()) && newArrivalTime < arrivalTime<CURRENT - 1>(tripIterator.stop())) {
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
            if constexpr (ExploreEndpointTransfers) {
                return directTransferArrivalLabels[parent].arrivalTime < never;
            } else {
                return parent == sourceStop;
            }
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
    // Seeds the search at the source stop. The input graph is stop-to-stop and
    // is used exactly as given -- no grouping, no first/last-mile preprocessing.
    inline void initializeSource() noexcept {
        directTransferArrivalLabels[sourceStop].arrivalTime = 0;
        stopsReachedByDirectTransfer.emplace_back(sourceStop);
        //Canonical MR explores the initial transfer (initialDijkstra). The input
        //graph is transitively closed, so one hop is the shortest walk; bound it
        //by the radius. Edges are sorted by ascending travel time.
        if constexpr (!ExploreEndpointTransfers) return;
        for (const Edge edge : data.transferGraph.edgesFrom(sourceStop)) {
            const int travelTime = data.transferGraph.get(TravelTime, edge);
            if (travelTime > maxTransferTravelTime) break;
            const Vertex neighbor = data.transferGraph.get(ToVertex, edge);
            if (!data.isStop(neighbor)) continue;
            if (travelTime < directTransferArrivalLabels[neighbor].arrivalTime) {
                if (directTransferArrivalLabels[neighbor].arrivalTime >= never) {
                    stopsReachedByDirectTransfer.emplace_back(StopId(neighbor));
                }
                directTransferArrivalLabels[neighbor].arrivalTime = travelTime;
            }
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
    // Canonical MR's finalDijkstra, as a one-hop phase: the input graph is
    // transitively closed, so a single hop is the shortest walk. A candidate is
    // witnessed (and discarded) if some 2-trip journey reaches its stop strictly
    // earlier by walking; survivors become shortcuts. The route-based arrivals are
    // snapshotted first so the test is independent of relaxation order.
    inline void finalOneHopTransfers() noexcept {
        if constexpr (!ExploreEndpointTransfers) {
            //Endpoint-optimised variant: shortcuts are emitted unconditionally at
            //the trip-2 boarding point in scanRoutes<2>, so nothing to do here.
            stopsUpdatedByRoute.clear();
            return;
        }
        finalRouteArrivals.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            finalRouteArrivals.emplace_back(stop, twoTripsArrivalLabels[stop].arrivalTime);
        }
        for (const auto& [from, baseTime] : finalRouteArrivals) {
            for (const Edge edge : data.transferGraph.edgesFrom(from)) {
                const int travelTime = data.transferGraph.get(TravelTime, edge);
                if (travelTime > maxTransferTravelTime) break;
                const Vertex to = data.transferGraph.get(ToVertex, edge);
                if (!data.isStop(to)) continue;
                const int newArrivalTime = baseTime + travelTime;
                if (newArrivalTime < twoTripsArrivalLabels[to].arrivalTime) {
                    updateArrival<2>(to, newArrivalTime, timestamp);
                    //A witness reached this stop earlier => its candidate dies.
                    twoTripsRouteParent[StopId(to)] = noStop;
                }
            }
        }
        for (const auto& [stop, baseTime] : finalRouteArrivals) {
            suppressUnusedParameterWarning(baseTime);
            const StopId boardStop = twoTripsRouteParent[stop];
            if (!data.isStop(boardStop)) continue;
            if (shortcutOrigin[boardStop] == noStop) continue;
            if (shortcutAlreadyExists(boardStop)) continue;
            shortcuts.emplace_back(shortcutOrigin[boardStop], boardStop, shortcutEdgeTime[boardStop]);
            twoTripsRouteParent[stop] = noStop;
        }
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
        if (ExploreEndpointTransfers ? (directTransferArrivalLabels[parent].arrivalTime < never)
                                     : (parent == sourceStop)) {
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

    inline void arrivalByRoute2(const StopId stop, const int arrivalTime, const StopId parent) noexcept {
        //Mark the journey as a candidate: the trip-2 boarding stop `parent` must
        //have been reached by a candidate intermediate transfer. Whether the
        //candidate survives is decided by the final one-hop phase.
        if ((shortcutOrigin[parent] != noStop) && !shortcutAlreadyExists(parent)) {
            twoTripsRouteParent[stop] = parent;
        } else {
            twoTripsRouteParent[stop] = noStop;
        }
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
    StopId sourceStop;
    int sourceDepartureTime;

    std::vector<ArrivalLabel> directTransferArrivalLabels;
    std::vector<StopId> stopsReachedByDirectTransfer;
    std::vector<std::pair<StopId, int>> finalRouteArrivals;

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
