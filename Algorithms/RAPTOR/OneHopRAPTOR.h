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

// One-hop RAPTOR (Delling, Dibbelt, Pajor 2019).
//
// Instead of operating on a transitively closed transfer graph, this variant
// works directly on a (non-transitive) one-hop transfer graph: in every round
// only a SINGLE transfer edge is relaxed. Consequently, journeys with two or
// more consecutive transfer edges are prohibited. Among the journeys that
// remain, the algorithm still finds the Pareto-optimal ones (by arrival time
// and number of trips). This avoids materializing the transitive closure -
// which can blow up the size of the transfer graph - at the cost of possibly
// counterintuitive journeys that take a detour to avoid chaining two transfers.
//
// In addition, this variant supports a per-query transfer radius: transfer
// edges whose travel time exceeds `maxTransferTravelTime` (seconds) are ignored
// for that query. Because the transfer edges of every stop are sorted by
// ascending TravelTime (see RAPTOR::Data::sortEdges), the radius check can stop
// scanning a stop's edges as soon as the limit is exceeded.
//
// The radius does NOT apply to the implicit "stay at the current stop" self
// loop, which merely carries a trip arrival into the transfer round so that the
// stop can serve routes in the following round.
template<typename PROFILER = NoProfiler, bool TARGET_PRUNING = true>
class OneHopRAPTOR {

public:
    using Profiler = PROFILER;
    static constexpr bool TargetPruning = TARGET_PRUNING;
    // One-hop transfers require route and transfer arrivals to be kept in
    // separate rounds, so that a stop reached by a trip cannot immediately be
    // used as the source of another transfer within the same round.
    static constexpr bool SeparateRouteAndTransferEntries = true;
    static constexpr int RoundFactor = 2;
    using ArrivalTime = EarliestArrivalTime<SeparateRouteAndTransferEntries>;
    using Type = OneHopRAPTOR<Profiler, TargetPruning>;
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
    OneHopRAPTOR(const Data& data, const Profiler& profilerTemplate = Profiler()) :
        data(data),
        earliestArrival(data.numberOfStops()),
        stopsUpdatedByRoute(data.numberOfStops()),
        stopsUpdatedByTransfer(data.numberOfStops()),
        routesServingUpdatedStops(data.numberOfRoutes()),
        sourceStop(noStop),
        targetStop(noStop),
        sourceDepartureTime(never),
        maxTransferTravelTime(INFTY),
        profiler(profilerTemplate) {
        Assert(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        profiler.registerExtraRounds({EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION});
        profiler.registerPhases({PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS});
        profiler.registerMetrics({METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    template<typename ATTRIBUTE>
    OneHopRAPTOR(const Data& data, const InitialTransferGraph&, const InitialTransferGraph&, const ATTRIBUTE, const Profiler& profilerTemplate = Profiler()) :
        OneHopRAPTOR(data, profilerTemplate) {
    }

    // Runs a query with an optional per-query transfer radius (in seconds of
    // transfer travel time). Passing INFTY (the default) disables the radius.
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
        relaxTransfers();
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
            relaxTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
        }
        profiler.done();
    }

    inline int getTransferRadius() const noexcept {
        return maxTransferTravelTime;
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

    inline Journey getEarliestJourney(const StopId stop) const noexcept {
        std::vector<Journey> journeys = getJourneys(stop);
        return journeys.empty() ? Journey() : journeys.back();
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

    inline std::vector<int> getArrivalTimes() const noexcept {
        return getArrivalTimes(targetStop);
    }

    inline std::vector<int> getArrivalTimes(const StopId stop) const noexcept {
        std::vector<int> arrivalTimes;
        for (size_t i = 0; i < rounds.size(); i += RoundFactor) {
            getArrivalTime(arrivalTimes, i, stop);
        }
        return arrivalTimes;
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

    inline int getArrivalTime(const StopId stop, const size_t numberOfTrips) const noexcept {
        size_t round = numberOfTrips * RoundFactor;
        if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        Assert(rounds[round][stop].arrivalTime < never, "No label found for stop " << stop << " in round " << round << "!");
        return rounds[round][stop].arrivalTime;
    }

private:
    inline void initialize(const StopId source, const int departureTime, const StopId target) noexcept {
        sourceStop = source;
        targetStop = target;
        sourceDepartureTime = departureTime;
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

    inline void relaxTransfers() noexcept {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = previousRound()[stop].arrivalTime;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                const int transferTravelTime = data.transferGraph.get(TravelTime, edge);
                // Transfer edges are sorted by ascending TravelTime, so once the
                // per-query radius is exceeded no later edge can satisfy it.
                if (transferTravelTime > maxTransferTravelTime) break;
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + transferTravelTime;
                Assert(data.isStop(data.transferGraph.get(ToVertex, edge)), "Graph contains edges to non stop vertices!");
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                if (arrivalByTransfer(toStop, arrivalTime)) {
                    EarliestArrivalLabel& label = currentRound()[toStop];
                    label.parent = stop;
                    label.parentDepartureTime = earliestArrivalTime;
                    label.usesRoute = false;
                    label.transferId = edge;
                }
            }
            // Carry the trip arrival into the transfer round so that the stop
            // itself can serve routes in the next round. Staying at the stop is
            // not a transfer edge and is therefore never subject to the radius.
            if (arrivalByTransfer(stop, earliestArrivalTime)) {
                EarliestArrivalLabel& label = currentRound()[stop];
                label.parent = stop;
                label.parentDepartureTime = earliestArrivalTime;
                label.usesRoute = false;
            }
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
        if constexpr (TargetPruning) if (earliestArrival[targetStop].getArrivalTimeByRoute() <= time) return false;
        if (earliestArrival[stop].getArrivalTimeByRoute() <= time) return false;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        currentRound()[stop].arrivalTime = time;
        earliestArrival[stop].setArrivalTimeByRoute(time);
        stopsUpdatedByRoute.insert(stop);
        return true;
    }

    inline bool arrivalByTransfer(const StopId stop, const int time) noexcept {
        Assert(data.isStop(stop), "Stop " << stop << " is out of range!");
        if constexpr (TargetPruning) if (earliestArrival[targetStop].getArrivalTimeByTransfer() <= time) return false;
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

    inline void getArrivalTime(std::vector<int>& labels, size_t round, const StopId stop) const noexcept {
        if ((round + 1 < rounds.size()) && (rounds[round + 1][stop].arrivalTime < rounds[round][stop].arrivalTime)) round++;
        labels.emplace_back(std::min(rounds[round][stop].arrivalTime, (labels.empty()) ? (never) : (labels.back())));
    }

private:
    const Data& data;

    std::vector<Round> rounds;

    std::vector<ArrivalTime> earliestArrival;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    StopId sourceStop;
    StopId targetStop;
    int sourceDepartureTime;
    int maxTransferTravelTime;

    Profiler profiler;

};

}
