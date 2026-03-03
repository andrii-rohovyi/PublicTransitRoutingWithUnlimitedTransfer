#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <type_traits>

#include "../CH/CH.h"
#include "../RAPTOR/InitialTransfers.h"

#include "../../Helpers/Assert.h"
#include "../../Helpers/Timer.h"
#include "../../Helpers/Types.h"
#include "../../Helpers/Vector/Vector.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../DataStructures/CSA/Entities/Journey.h"
#include "../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../DataStructures/TripBased/DelayData.h"
#include "Profiler.h"

namespace CSA {

template<bool PATH_RETRIEVAL = true, typename PROFILER = NoProfiler>
class DelayULTRACSA {

public:
    using InitialTransferGraph = CHGraph;
    constexpr static bool PathRetrieval = PATH_RETRIEVAL;
    using Profiler = PROFILER;
    using Type = DelayULTRACSA<PathRetrieval, Profiler>;
    using TripFlag = std::conditional_t<PathRetrieval, ConnectionId, bool>;

private:
    struct ParentLabel {
        ParentLabel(const Vertex parent = noVertex,
                    const bool reachedByTransfer = false,
                    const TripId tripId = noTripId) :
            parent(parent),
            reachedByTransfer(reachedByTransfer),
            tripId(tripId) {
        }
        Vertex parent;
        bool reachedByTransfer;
        union {
            TripId tripId;
            Edge transferId;
        };
    };

public:
    DelayULTRACSA(const Data& data,
                  const CH::CH& chData,
                  const std::vector<int>& arrivalDelay = {},
                  const int maxArrivalDelay = 0,
                  const Profiler& profilerTemplate = Profiler()) :
        data(data),
        initialTransfers(chData, FORWARD, data.numberOfStops()),
        arrivalDelay(arrivalDelay),
        maxArrivalDelay(maxArrivalDelay),
        hasDelayInfo(!arrivalDelay.empty()),
        sourceVertex(noVertex),
        sourceDepartureTime(never),
        targetVertex(noVertex),
        tripReached(data.numberOfTrips(), TripFlag()),
        arrivalTime(data.numberOfStops() + 1, never),
        parentLabel(PathRetrieval ? data.numberOfStops() + 1 : 0),
        tripBoardingEvent(data.numberOfTrips(), noStopEvent),
        profiler(profilerTemplate) {
        Assert(!Graph::hasLoops(data.transferGraph),
               "Shortcut graph may not have loops!");
        Assert(Vector::isSorted(data.connections),
               "Connections must be sorted in ascending order!");
        profiler.registerPhases({PHASE_CLEAR, PHASE_INITIALIZATION,
                                 PHASE_CONNECTION_SCAN});
        profiler.registerMetrics({METRIC_CONNECTIONS, METRIC_EDGES,
                                  METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER});
        profiler.initialize();
    }

    inline void run(const Vertex source, const int departureTime,
                    const Vertex target = noVertex) noexcept {
        profiler.start();

        profiler.startPhase();
        clear();
        profiler.donePhase(PHASE_CLEAR);

        profiler.startPhase();
        sourceVertex = source;
        sourceDepartureTime = departureTime;
        targetVertex = target;
        if (target == noVertex) {
            targetStop = noStop;
        } else {
            targetStop = data.isStop(target) ? StopId(target)
                                             : StopId(data.numberOfStops());
        }
        if (data.isStop(sourceVertex)) {
            arrivalTime[sourceVertex] = departureTime;
        }
        runInitialTransfers();
        const ConnectionId firstConnection = firstReachableConnection(departureTime);
        profiler.donePhase(PHASE_INITIALIZATION);

        profiler.startPhase();
        scanConnections(firstConnection, ConnectionId(data.connections.size()));
        profiler.donePhase(PHASE_CONNECTION_SCAN);

        profiler.done();
    }

    inline bool reachable(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? targetStop : StopId(vertex);
        return arrivalTime[stop] < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) const noexcept {
        const StopId stop = (vertex == targetVertex) ? targetStop : StopId(vertex);
        return arrivalTime[stop];
    }

    inline Journey getJourney() noexcept requires (PathRetrieval) {
        return getJourney(targetStop);
    }

    inline Journey getJourney(const Vertex vertex) noexcept requires (PathRetrieval) {
        StopId stop = (vertex == targetVertex) ? targetStop : StopId(vertex);
        Journey journey;
        if (!reachable(stop)) return journey;
        while (stop != sourceVertex) {
            const ParentLabel& label = parentLabel[stop];
            if (label.reachedByTransfer) {
                const int parentDep = (label.parent == sourceVertex)
                    ? sourceDepartureTime : arrivalTime[label.parent];
                journey.emplace_back(label.parent, stop, parentDep,
                                     arrivalTime[stop], label.transferId);
            } else {
                journey.emplace_back(
                    label.parent, stop,
                    data.connections[tripReached[label.tripId]].departureTime,
                    arrivalTime[stop], label.tripId);
            }
            stop = StopId(label.parent);
        }
        Vector::reverse(journey);
        return journey;
    }

    inline std::vector<RAPTOR::ArrivalLabel> getArrivals() const noexcept {
        std::vector<RAPTOR::ArrivalLabel> result;
        if (targetStop != noStop && arrivalTime[targetStop] < never) {
            result.emplace_back(arrivalTime[targetStop], 1);
        }
        return result;
    }

    inline const Profiler& getProfiler() const noexcept {
        return profiler;
    }

private:
    inline void clear() {
        sourceVertex = noVertex;
        sourceDepartureTime = never;
        targetVertex = noVertex;
        targetStop = noStop;
        Vector::fill(arrivalTime, never);
        Vector::fill(tripReached, TripFlag());
        Vector::fill(tripBoardingEvent, noStopEvent);
        if constexpr (PathRetrieval) {
            Vector::fill(parentLabel, ParentLabel());
        }
    }

    inline ConnectionId firstReachableConnection(const int departureTime) const noexcept {
        return ConnectionId(Vector::lowerBound(data.connections, departureTime,
            [](const Connection& connection, const int time) {
                return connection.departureTime < time;
            }));
    }

    inline void scanConnections(const ConnectionId begin,
                                const ConnectionId end) noexcept {
        for (ConnectionId i = begin; i < end; i++) {
            const Connection& connection = data.connections[i];
            if (targetStop != noStop &&
                connection.departureTime > arrivalTime[targetStop])
                break;

            if (connectionIsReachable(connection, i)) {
                profiler.countMetric(METRIC_CONNECTIONS);
                arrivalByTrip(connection.arrivalStopId, connection.arrivalTime,
                              connection.tripId, i);
            }
        }
    }

    inline bool connectionIsReachableFromStop(const Connection& connection) const noexcept {
        return arrivalTime[connection.departureStopId] <=
               connection.departureTime -
               data.minTransferTime(connection.departureStopId);
    }

    inline bool connectionIsReachableFromTrip(const Connection& connection) const noexcept {
        return tripReached[connection.tripId] != TripFlag();
    }

    inline bool connectionIsReachable(const Connection& connection,
                                      const ConnectionId id) noexcept {
        if (connectionIsReachableFromTrip(connection)) return true;
        if (connectionIsReachableFromStop(connection)) {
            if constexpr (PathRetrieval) {
                tripReached[connection.tripId] = id;
            } else {
                suppressUnusedParameterWarning(id);
                tripReached[connection.tripId] = true;
            }
            tripBoardingEvent[connection.tripId] =
                StopEventId(connection.departureStopId);
            return true;
        }
        return false;
    }

    inline void arrivalByTrip(const StopId stop, const int time,
                              const TripId trip, const ConnectionId connId) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        arrivalTime[stop] = time;
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = data.connections[tripReached[trip]].departureStopId;
            parentLabel[stop].reachedByTransfer = false;
            parentLabel[stop].tripId = trip;
        }

        const StopEventId fromStopEvent = StopEventId(connId);

        for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
            profiler.countMetric(METRIC_EDGES);
            const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
            const int travelTime = data.transferGraph.get(TravelTime, edge);
            const int newArrivalTime = time + travelTime;

            if (hasDelayInfo) {
                if (!isShortcutValidUnderDelay(edge, fromStopEvent)) continue;
            }

            arrivalByTransfer(toStop, newArrivalTime, stop, edge);
        }

        if (initialTransfers.getBackwardDistance(stop) != INFTY) {
            profiler.countMetric(METRIC_EDGES);
            const int newArrivalTime = time + initialTransfers.getBackwardDistance(stop);
            arrivalByTransfer(targetStop, newArrivalTime, stop, noEdge);
        }
    }

    inline bool isShortcutValidUnderDelay([[maybe_unused]] const Edge edge,
                                          [[maybe_unused]] const StopEventId fromEvent) const noexcept {
        return true;
    }

    inline void runInitialTransfers() noexcept {
        initialTransfers.run(sourceVertex, targetVertex);
        for (const Vertex stop : initialTransfers.getForwardPOIs()) {
            Assert(data.isStop(stop), "Reached POI " << stop << " is not a stop!");
            Assert(initialTransfers.getForwardDistance(stop) != INFTY,
                   "Vertex " << stop << " was not reached!");
            profiler.countMetric(METRIC_EDGES);
            const int newArrivalTime =
                sourceDepartureTime + initialTransfers.getForwardDistance(stop);
            arrivalByTransfer(StopId(stop), newArrivalTime, sourceVertex, noEdge);
        }
        if (initialTransfers.getDistance() != INFTY) {
            const int newArrivalTime =
                sourceDepartureTime + initialTransfers.getDistance();
            profiler.countMetric(METRIC_EDGES);
            arrivalByTransfer(targetStop, newArrivalTime, sourceVertex, noEdge);
        }
    }

    inline void arrivalByTransfer(const StopId stop, const int time,
                                  const Vertex parent, const Edge edge) noexcept {
        if (arrivalTime[stop] <= time) return;
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        arrivalTime[stop] = time;
        if constexpr (PathRetrieval) {
            parentLabel[stop].parent = parent;
            parentLabel[stop].reachedByTransfer = true;
            parentLabel[stop].transferId = edge;
        }
    }

private:
    const Data& data;
    RAPTOR::BucketCHInitialTransfers initialTransfers;

    const std::vector<int>& arrivalDelay;
    const int maxArrivalDelay;
    const bool hasDelayInfo;

    Vertex sourceVertex;
    int sourceDepartureTime;
    Vertex targetVertex;
    StopId targetStop;

    std::vector<TripFlag> tripReached;
    std::vector<int> arrivalTime;
    std::vector<ParentLabel> parentLabel;
    std::vector<StopEventId> tripBoardingEvent;

    Profiler profiler;
};

} // namespace CSA