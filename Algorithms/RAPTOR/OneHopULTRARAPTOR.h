#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/RAPTOR/Entities/EarliestArrivalTime.h"
#include "../../DataStructures/Container/IndexedSet.h"
#include "../../DataStructures/Container/Map.h"

#include "Profiler.h"

namespace RAPTOR {

// One-hop ULTRA-RAPTOR WITHOUT a Bucket-CH (apples-to-apples validation variant).
//
// Standard ULTRA-RAPTOR uses the shortcut graph for intermediate transfers and a
// Bucket-CH one-to-many search for the initial (source->first stop) and final
// (last stop->target) transfers. To compare the shortcut set against a plain
// one-hop RAPTOR on the full transfer graph on equal footing, this variant drops
// the CH and instead takes the initial and final transfers directly from the
// FULL transfer graph, while intermediate transfers use the shortcut graph
// (data.transferGraph). All transfers remain one-hop (a single edge) and are
// bounded by the same per-query radius.
//
// Consequently, the ONLY difference from a one-hop RAPTOR on the full graph is
// the source of intermediate transfers (shortcuts vs. full graph). If the
// shortcut set is a correct, sufficient set of intermediate transfers, this
// algorithm returns exactly the same Pareto set as one-hop RAPTOR on the full
// graph for every query.
template<typename PROFILER = NoProfiler, bool TARGET_PRUNING = true>
class OneHopULTRARAPTOR {

public:
    using Profiler = PROFILER;
    static constexpr bool TargetPruning = TARGET_PRUNING;
    static constexpr bool SeparateRouteAndTransferEntries = true;
    static constexpr int RoundFactor = 2;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = OneHopULTRARAPTOR<Profiler, TargetPruning>;
    using InitialTransferGraph = TransferGraph;
    using SourceType = StopId;

private:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel() : arrivalTime(never), parentDepartureTime(never), parent(noStop), usesRoute(false), routeId(noRouteId) {}
        int arrivalTime;
        int parentDepartureTime;
        StopId parent;
        bool usesRoute;
        union {
            RouteId routeId;
            Edge transferId;
        };
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    // data.transferGraph must hold the (intermediate) shortcut graph.
    // fullTransferGraph must hold the full transfer graph (e.g. transitive
    // closure) used for the initial and final transfers.
    OneHopULTRARAPTOR(const Data& data, const TransferGraph& fullTransferGraph, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        fullTransferGraph(fullTransferGraph),
        earliestArrival(data.numberOfStops()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        targetTransferTime(data.numberOfStops(), INFTY),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(never),
        maxTransferTravelTime(INFTY),
        profiler(profilerTemplate) {
        Assert(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        Assert(fullTransferGraph.numVertices() >= data.numberOfStops(), "Full transfer graph is too small!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const StopId source, const int departureTime, const StopId target = noStop, const int maxTransferTravelTime = INFTY, const size_t maxRounds = INFTY) noexcept {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear();
        this->maxTransferTravelTime = maxTransferTravelTime;
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime, target);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        relaxTransfers<true>();
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();

        for (size_t i = 0; i < maxRounds; i++) {
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
            if (stopsUpdatedByRoute.empty()) {
                profiler.doneRound();
                break;
            }
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            relaxTransfers<false>();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    inline std::vector<Journey> getJourneys() const noexcept {
        return getJourneys(targetStop);
    }

    inline std::vector<Journey> getJourneys(const StopId stop) const noexcept {
        std::vector<Journey> journeys;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getJourney(journeys, i, stop);
        }
        return journeys;
    }

    inline std::vector<ArrivalLabel> getArrivals() const noexcept {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(const StopId stop) const noexcept {
        Assert(data.isStop(stop), "The StopId " << stop << " does not correspond to any stop!");
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrival(labels, i, stop);
        }
        return labels;
    }

    inline bool reachable(const StopId stop) const noexcept {
        return earliestArrival[stop].getArrivalTime() < never;
    }

    inline int getEarliestArrivalTime(const StopId stop) const noexcept {
        return earliestArrival[stop].getArrivalTime();
    }

    template<bool RESET_CAPACITIES = false>
    inline void clear() noexcept {
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        // Reset the target transfer times that were set for the previous query.
        for (const StopId stop : targetTransferStops) {
            targetTransferTime[stop] = INFTY;
        }
        targetTransferStops.clear();
        targetStop = noStop;
        sourceDepartureTime = never;
        maxTransferTravelTime = INFTY;
        if constexpr (RESET_CAPACITIES) {
            std::vector<Round>().swap(rounds);
            std::vector<ArrivalTime>(earliestArrival.size()).swap(earliestArrival);
        } else {
            rounds.clear();
            Vector::fill(earliestArrival);
        }
    }

    inline void reset() noexcept {
        clear<true>();
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void initialize(const StopId source, const int departureTime, const StopId target) noexcept {
        sourceStop = source;
        targetStop = target;
        sourceDepartureTime = departureTime;
        // Precompute the final transfer: time from each stop to the target via
        // the full transfer graph (symmetric on the closed graph, so we read the
        // target's outgoing edges). targetTransferTime[v] = walk(v -> target).
        if (target != noStop) {
            targetTransferTime[target] = 0;
            targetTransferStops.emplace_back(target);
            for (const Edge edge : fullTransferGraph.edgesFrom(target)) {
                const Vertex neighbor = fullTransferGraph.get(ToVertex, edge);
                if (!data.isStop(neighbor)) continue;
                const int travelTime = fullTransferGraph.get(TravelTime, edge);
                if (travelTime < targetTransferTime[neighbor]) {
                    if (targetTransferTime[neighbor] >= INFTY) targetTransferStops.emplace_back(StopId(neighbor));
                    targetTransferTime[neighbor] = travelTime;
                }
            }
        }
        startNewRound();
        arrivalByRoute(source, sourceDepartureTime);
        currentRound()[source].parent = source;
        currentRound()[source].parentDepartureTime = sourceDepartureTime;
        currentRound()[source].usesRoute = false;
        startNewRound();
    }

    inline void collectRoutesServingUpdatedStops() noexcept {
        for (const StopId stop : stopsUpdatedByTransfer) {
            Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
            const int arrivalTime = previousRound()[stop].arrivalTime;
            Assert(arrivalTime < never, "Updated stop has arrival time = never!");
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                Assert(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                Assert(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId)) continue;
                if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < arrivalTime) continue;
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline void scanRoutes() noexcept {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            profiler.countMetric(METRIC_ROUTES);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            Assert(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* trip = data.lastTripOfRoute(route);
            StopId stop = stops[stopIndex];
            Assert(trip[stopIndex].departureTime >= previousRound()[stop].arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRound()[stop].arrivalTime << ", LastDeparture: " << trip[stopIndex].departureTime << ")!");

            StopIndex parentIndex = stopIndex;
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRound()[stop].arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                stopIndex++;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                if (arrivalByRoute(stop, trip[stopIndex].arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[stop];
                    label.parent = stops[parentIndex];
                    label.parentDepartureTime = trip[parentIndex].departureTime;
                    label.usesRoute = true;
                    label.routeId = route;
                }
            }
        }
    }

    // INITIAL_TRANSFERS: use the full transfer graph from the source (initial
    // transfers). Otherwise: use the shortcut graph for intermediate transfers,
    // plus a single final transfer to the target via the full graph.
    template<bool INITIAL_TRANSFERS>
    inline void relaxTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = previousRound()[stop].arrivalTime;
            if constexpr (INITIAL_TRANSFERS) {
                for (const Edge edge : fullTransferGraph.edgesFrom(stop)) {
                    const int travelTime = fullTransferGraph.get(TravelTime, edge);
                    if (travelTime > maxTransferTravelTime) break;
                    const Vertex toVertex = fullTransferGraph.get(ToVertex, edge);
                    if (!data.isStop(toVertex)) continue;
                    profiler.countMetric(METRIC_EDGES);
                    relaxTransferEdge(stop, StopId(toVertex), earliestArrivalTime + travelTime, edge);
                }
            } else {
                for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                    const int travelTime = data.transferGraph.get(TravelTime, edge);
                    if (travelTime > maxTransferTravelTime) break;
                    profiler.countMetric(METRIC_EDGES);
                    const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                    relaxTransferEdge(stop, toStop, earliestArrivalTime + travelTime, edge);
                }
                // Final transfer to the target via the full transfer graph.
                if (targetStop != noStop && targetTransferTime[stop] <= maxTransferTravelTime) {
                    relaxTransferEdge(stop, targetStop, earliestArrivalTime + targetTransferTime[stop], noEdge);
                }
            }
            // Carry the trip arrival into the transfer round so the stop can
            // serve routes in the next round.
            if (arrivalByTransfer(stop, earliestArrivalTime)) {
                EarliestArrivalLabel& label = currentRound()[stop];
                label.parent = stop;
                label.parentDepartureTime = earliestArrivalTime;
                label.usesRoute = false;
            }
        }
    }

    inline void relaxTransferEdge(const StopId fromStop, const StopId toStop, const int arrivalTime, const Edge edge) noexcept {
        Assert(data.isStop(toStop), "Transfer reaches non-stop vertex!");
        if (arrivalByTransfer(toStop, arrivalTime)) {
            EarliestArrivalLabel& label = currentRound()[toStop];
            label.parent = fromStop;
            label.parentDepartureTime = previousRound()[fromStop].arrivalTime;
            label.usesRoute = false;
            label.transferId = edge;
        }
    }

    inline Round& currentRound() noexcept {
        Assert(!rounds.empty(), "Cannot return current round, because no round exists!");
        return rounds.back();
    }

    inline Round& previousRound() noexcept {
        Assert(rounds.size() >= 2, "Cannot return previous round, because less than two rounds exist!");
        return rounds[rounds.size() - 2];
    }

    inline void startNewRound() noexcept {
        rounds.emplace_back(data.numberOfStops());
    }

    inline bool arrivalByRoute(const StopId stop, const int time) noexcept {
        Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (targetStop != noStop && earliestArrival[targetStop].getArrivalTimeByRoute() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByRoute() <= time) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline bool arrivalByTransfer(const StopId stop, const int time) noexcept {
        Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (targetStop != noStop && earliestArrival[targetStop].getArrivalTimeByTransfer() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByTransfer() <= time) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByTransfer(time);
        stopsUpdatedByTransfer.insert(stop);
        return true;
    }

    inline void getJourney(std::vector<Journey>& journeys, size_t round, StopId stop) const noexcept {
        if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        if (rounds[round][stop].arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime)) return;
        Journey journey;
        do {
            Assert(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = rounds[round][stop];
            journey.emplace_back(label.parent, stop, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);
            stop = label.parent;
            round--;
        } while (journey.back().from != sourceStop);
        journeys.emplace_back(Vector::reverse(journey));
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, size_t round, const StopId stop) const noexcept {
        if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        if (rounds[round][stop].arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime)) return;
        labels.emplace_back(rounds[round][stop].arrivalTime, round / RoundFactor);
    }

private:
    const Data& data;
    const TransferGraph& fullTransferGraph;

    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    std::vector<int> targetTransferTime;
    std::vector<StopId> targetTransferStops;

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    int maxTransferTravelTime;

    Profiler profiler;

};

}
