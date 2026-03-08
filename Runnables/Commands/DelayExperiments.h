#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <random>

#include "../../Shell/Shell.h"

#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../DataStructures/TripBased/Delay.h"
#include "../../DataStructures/TripBased/DelayData.h"
#include "../../DataStructures/TripBased/DelayInfo.h"
#include "../../DataStructures/TripBased/DelayUpdateData.h"
#include "../../Algorithms/CSA/DelayULTRACSA.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../Algorithms/Dijkstra/TransferAwareDijkstra.h"
#include "../../DataStructures/Graph/TimeDependentGraph.h"
#include "../../DataStructures/Intermediate/Data.h"
#include "../../Algorithms/RAPTOR/ULTRARAPTOR.h"

// CSA delay support
#include "../../DataStructures/CSA/DelayData.h"
#include "../../Algorithms/CSA/DelayULTRACSA.h"
#include "../../DataStructures/CSA/Data.h"

#include "../../Algorithms/RAPTOR/DijkstraRAPTOR.h"
#include "../../Algorithms/RAPTOR/InitialTransfers.h"
#include "../../Algorithms/TripBased/Preprocessing/DelayUpdater.h"
#include "../../Algorithms/TripBased/Query/Query.h"

#include "../../Helpers/MultiThreading.h"
#include "../../DataStructures/CSA/Data.h"
#include "../../Algorithms/CSA/ULTRACSA.h"

#include "../../Algorithms/Dijkstra/TransferAwareDijkstraBucketCH.h"
#include "../../Algorithms/Dijkstra/TimeDependentDijkstraBucketCH.h"

using namespace Shell;

// ═══════════════════════════════════════════════════════════════════════════
//  STATISTICS & DATA STRUCTURES
// ═══════════════════════════════════════════════════════════════════════════

struct QueryStatistics {
    QueryStatistics(const std::vector<VertexQuery>& queries, const std::vector<std::vector<RAPTOR::ArrivalLabel>>& exactResults, const std::vector<std::vector<RAPTOR::ArrivalLabel>>& algorithmResults) :
        totalQueries(0),
        totalJourneys(0),
        failedQueries(0),
        failedJourneys(0) {
        for (size_t i = 0; i < queries.size(); i++) {
            addQuery(queries[i], exactResults[i], algorithmResults[i]);
        }
        finalize();
    }

    inline void addQuery(const VertexQuery& query, const std::vector<RAPTOR::ArrivalLabel>& exactResults, const std::vector<RAPTOR::ArrivalLabel>& algorithmResults) noexcept {
        totalQueries++;
        totalJourneys += exactResults.size();
        if (Vector::equals(exactResults, algorithmResults)) return;
        failedQueries++;
        for (const RAPTOR::ArrivalLabel& exactResult : exactResults) {
            int bestArrivalTime = INFTY;
            for (const RAPTOR::ArrivalLabel& algorithmResult : algorithmResults) {
                if (algorithmResult.numberOfTrips > exactResult.numberOfTrips) break;
                bestArrivalTime = algorithmResult.arrivalTime;
            }
            if (bestArrivalTime != exactResult.arrivalTime) {
                failedJourneys++;
                if (bestArrivalTime != INFTY) {
                    const int exactTravelTime = exactResult.arrivalTime - query.departureTime;
                    detours.emplace_back((bestArrivalTime - exactResult.arrivalTime)/static_cast<double>(exactTravelTime));
                }
            }
        }
    }

    inline void finalize() noexcept {
        std::sort(detours.begin(), detours.end());
    }

    inline friend std::ostream& operator<<(std::ostream& out, const QueryStatistics& statistics) noexcept {
        out << "Queries: " << statistics.totalQueries << std::endl;
        out << "Exact journeys: " << statistics.totalJourneys << std::endl;
        out << "Failed queries: " << statistics.failedQueries << " (" << String::percent(statistics.failedQueries/static_cast<double>(statistics.totalQueries)) << ")" << std::endl;
        out << "Failed journeys: " << statistics.failedJourneys << " (" << String::percent(statistics.failedJourneys/static_cast<double>(statistics.totalJourneys)) << ")" << std::endl;
        if (!statistics.detours.empty()) {
            out << "Mean detour: " << String::percent(Vector::mean(statistics.detours)) << std::endl;
            out << "Median detour: " << String::percent(Vector::median(statistics.detours)) << std::endl;
            out << "95th perc. detour: " << String::percent(Vector::percentile(statistics.detours, 0.95)) << std::endl;
        }
        return out;
    }

    size_t totalQueries;
    size_t totalJourneys;
    size_t failedQueries;
    size_t failedJourneys;
    std::vector<double> detours;
};

struct Transfer {
    Transfer(const StopEventId from, const StopEventId to, const int travelTime) :
        from(from),
        to(to),
        travelTime(travelTime) {
    }

    StopEventId from;
    StopEventId to;
    int travelTime;

    inline friend std::ostream& operator<<(std::ostream& out, const Transfer& transfer) noexcept {
        return out << transfer.from << " -> " << transfer.to << " @ " << transfer.travelTime;
    }
};

struct JourneyData {
    JourneyData(const TripBased::Data& data, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) :
        transfers(getTransfers(data, internalToOriginal, journey)),
        finalStopEvent(getFinalStopEvent(data, internalToOriginal, journey)),
        arrivalLabel(journey.back().arrivalTime, countTrips(journey)) {
    }

    JourneyData(const TripBased::DelayData& data, const TripBased::Data& delayedData, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) :
        JourneyData(delayedData, internalToOriginal, journey) {
        if (finalStopEvent != noStopEvent) {
            arrivalLabel.arrivalTime -= data.arrivalDelay[finalStopEvent];
        }
    }

    JourneyData(const TripBased::Data& data, const RAPTOR::Journey& journey) :
        JourneyData(data, Permutation(Construct::Id, data.numberOfStopEvents()), journey) {
    }

    JourneyData(const TripBased::DelayData& data, const RAPTOR::Journey& journey) :
        JourneyData(data, data.data, Permutation(Construct::Id, data.data.numberOfStopEvents()), journey) {
    }

    inline bool isFeasible(const TripBased::DelayData& delayData) const noexcept {
        for (const Transfer& transfer : transfers) {
            if (!delayData.isTransferFeasible(transfer.from, transfer.to, transfer.travelTime)) return false;
        }
        return true;
    }

    inline std::vector<int> getTransferSlacks(const TripBased::Data& data) const noexcept {
        std::vector<int> slacks;
        for (const Transfer& transfer : transfers) {
            slacks.emplace_back(data.getTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return slacks;
    }

    inline int getMinTransferSlack(const TripBased::Data& data) const noexcept {
        if (transfers.empty()) return 0;
        int minTransferSlack = INFTY;
        for (const Transfer& transfer : transfers) {
            minTransferSlack = std::min(minTransferSlack, data.getTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return minTransferSlack;
    }

    inline int getMaxTransferSlack(const TripBased::Data& data) const noexcept {
        int maxTransferSlack = 0;
        for (const Transfer& transfer : transfers) {
            maxTransferSlack = std::max(maxTransferSlack, data.getTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return maxTransferSlack;
    }

    inline int getMaxFailedTransferSlack(const TripBased::DelayData& delayData) const noexcept {
        int maxTransferSlack = 0;
        for (const Transfer& transfer : transfers) {
            if (delayData.isTransferFeasible(transfer.from, transfer.to, transfer.travelTime)) continue;
            maxTransferSlack = std::max(maxTransferSlack, delayData.getUndelayedTransferSlack(transfer.from, transfer.to, transfer.travelTime));
        }
        return maxTransferSlack;
    }

    inline RAPTOR::ArrivalLabel getDelayedArrivalLabel(const TripBased::DelayData& delayData) const noexcept {
        RAPTOR::ArrivalLabel result = arrivalLabel;
        if (finalStopEvent != noStopEvent) {
            result.arrivalTime += delayData.arrivalDelay[finalStopEvent];
        }
        return result;
    }

    inline static std::vector<Transfer> getTransfers(const TripBased::Data& data, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) noexcept {
        std::vector<Transfer> transfers;
        for (size_t i = 0; i < journey.size() - 1; i++) {
            if (!journey[i].usesRoute) continue;
            size_t next = i+1;
            int travelTime = 0;
            if (i < journey.size() - 2 && !journey[i+1].usesRoute && journey[i+2].usesRoute) {
                next = i+2;
                travelTime = journey[i+1].arrivalTime - journey[i].arrivalTime;
            } else if (!journey[i+1].usesRoute) {
                continue;
            }
            const StopId fromStop = StopId(journey[i].to);
            const RouteId fromRoute = journey[i].routeId;
            const int fromTime = journey[i].arrivalTime;
            const StopId toStop = StopId(journey[next].from);
            const RouteId toRoute = journey[next].routeId;
            const int toTime = journey[next].departureTime;
            const StopEventId fromEvent = findStopEvent(data, internalToOriginal, fromStop, fromRoute, fromTime, false);
            const StopEventId toEvent = findStopEvent(data, internalToOriginal, toStop, toRoute, toTime, true);
            transfers.emplace_back(fromEvent, toEvent, travelTime);
        }
        return transfers;
    }

    inline static StopEventId getFinalStopEvent(const TripBased::Data& data, const Permutation& internalToOriginal, const RAPTOR::Journey& journey) noexcept {
        for (size_t i = journey.size() - 1; i != size_t(-1); i--) {
            if (!journey[i].usesRoute) continue;
            const StopId stop = StopId(journey[i].to);
            const RouteId route = journey[i].routeId;
            const int arrivalTime = journey[i].arrivalTime;
            return findStopEvent(data, internalToOriginal, stop, route, arrivalTime, false);
        }
        return noStopEvent;
    }

    inline static StopEventId findStopEvent(const TripBased::Data& data, const Permutation& internalToOriginal, const StopId stop, const RouteId route, const int time, const bool departure) noexcept {
        for (StopIndex stopIndex(0); stopIndex < data.numberOfStopsInRoute(route); stopIndex++) {
            if (stop != data.raptorData.stopArrayOfRoute(route)[stopIndex]) continue;
            for (const TripId trip : data.tripsOfRoute(route)) {
                const StopEventId stopEvent = data.getStopEventId(trip, stopIndex);
                if (departure) {
                    if (data.departureTime(stopEvent) == time) return StopEventId(internalToOriginal[stopEvent]);
                } else {
                    if (data.arrivalTime(stopEvent) == time) return StopEventId(internalToOriginal[stopEvent]);
                }
            }
        }
        return noStopEvent;
    }

    std::vector<Transfer> transfers;
    StopEventId finalStopEvent;
    RAPTOR::ArrivalLabel arrivalLabel;
};

inline std::vector<JourneyData> getJourneyData(const TripBased::DelayData& data, const TripBased::Data& delayedData, const Permutation& internalToOriginal, const std::vector<RAPTOR::Journey>& journeys) noexcept {
    std::vector<JourneyData> journeyData;
    for (const RAPTOR::Journey& journey : journeys) {
        journeyData.emplace_back(data, delayedData, internalToOriginal, journey);
    }
    return journeyData;
}

inline std::vector<JourneyData> getJourneyData(const TripBased::DelayData& data, const std::vector<RAPTOR::Journey>& journeys) noexcept {
    std::vector<JourneyData> journeyData;
    for (const RAPTOR::Journey& journey : journeys) {
        journeyData.emplace_back(data, journey);
    }
    return journeyData;
}

inline std::vector<JourneyData> getJourneyData(const TripBased::Data& data, const std::vector<RAPTOR::Journey>& journeys) noexcept {
    std::vector<JourneyData> journeyData;
    for (const RAPTOR::Journey& journey : journeys) {
        journeyData.emplace_back(data, journey);
    }
    return journeyData;
}

struct QueryFeasibilityStatistics {
    QueryFeasibilityStatistics() :
        infeasibleQueries(0),
        infeasibleJourneys(0),
        totalJourneys(0) {
    }

    inline void addQuery(const std::vector<JourneyData>& journeyData, const TripBased::DelayData& delayData) noexcept {
        algorithmResults.emplace_back();
        totalJourneys += journeyData.size();
        bool infeasible = false;
        for (const JourneyData& journey : journeyData) {
            if (journey.isFeasible(delayData)) {
                algorithmResults.back().emplace_back(journey.getDelayedArrivalLabel(delayData));
            } else {
                infeasibleJourneys++;
                maxTransferSlacks.emplace_back(journey.getMaxFailedTransferSlack(delayData));
                infeasible = true;
            }
        }
        if (infeasible) infeasibleQueries++;
    }

    inline void finalize() noexcept {
        std::sort(maxTransferSlacks.begin(), maxTransferSlacks.end());
    }

    inline friend std::ostream& operator<<(std::ostream& out, const QueryFeasibilityStatistics& statistics) noexcept {
        out << "Infeasible queries: " << statistics.infeasibleQueries << " (" << String::percent(statistics.infeasibleQueries/static_cast<double>(statistics.algorithmResults.size())) << ")" << std::endl;
        out << "Infeasible journeys: " << statistics.infeasibleJourneys << " (" << String::percent(statistics.infeasibleJourneys/static_cast<double>(statistics.totalJourneys)) << ")" << std::endl;
        if (statistics.infeasibleJourneys > 0) {
            out << "Mean max transfer slack: " << Vector::mean(statistics.maxTransferSlacks) << std::endl;
            out << "Median max transfer slack: " << Vector::median(statistics.maxTransferSlacks) << std::endl;
            out << "95th perc. max transfer slack: " << Vector::percentile(statistics.maxTransferSlacks, 0.95) << std::endl;
        }
        return out;
    }

    size_t infeasibleQueries;
    size_t infeasibleJourneys;
    size_t totalJourneys;
    std::vector<size_t> maxTransferSlacks;
    std::vector<std::vector<RAPTOR::ArrivalLabel>> algorithmResults;
};

struct QueryInputData {
    QueryInputData() {}

    QueryInputData(const std::string& fileName) {
        deserialize(fileName);
    }

    inline void sortQueries() noexcept {
        Order queryOrder(Construct::Id, failedQueries.size());
        std::sort(queryOrder.begin(), queryOrder.end(), [&](const size_t a, const size_t b) {
            return failedQueries[a].departureTime < failedQueries[b].departureTime;
        });
        queryOrder.order(failedQueries);
        queryOrder.order(failedQueryResults);
    }

    inline void serialize(const std::string& fileName) const noexcept {
        IO::serialize(fileName, allowDepartureDelays, failedQueries, failedQueryResults, groupedDelaysByQuery);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        IO::deserialize(fileName, allowDepartureDelays, failedQueries, failedQueryResults, groupedDelaysByQuery);
    }

    bool allowDepartureDelays;
    std::vector<VertexQuery> failedQueries;
    std::vector<std::vector<RAPTOR::ArrivalLabel>> failedQueryResults;
    std::vector<std::vector<TripBased::DelayIncident>> groupedDelaysByQuery;
};

// ═══════════════════════════════════════════════════════════════════════════
//  SHARED HELPERS (used by MeasureDelayULTRACSAQueryPerformance and
//  MeasureDelayQueryCoverage)
// ═══════════════════════════════════════════════════════════════════════════

namespace DelayHelpers {

inline StopId stopOfEvent(const TripBased::Data& tbData,
                          const StopEventId event) noexcept {
    const TripId trip = tbData.tripOfStopEvent[event];
    const StopIndex idx = tbData.indexOfStopEvent[event];
    return tbData.stopArrayOfTrip(trip)[idx];
}

inline Intermediate::TransferGraph convertShortcutsToStopGraph(
        const RAPTOR::Data& raptorData,
        const TripBased::Data& tbData) noexcept {
    const size_t numStops = raptorData.numberOfStops();

    Intermediate::TransferGraph graph;
    graph.addVertices(numStops);

    for (size_t i = 0; i < numStops; i++) {
        graph.set(Coordinates, Vertex(i),
            raptorData.transferGraph.get(Coordinates, Vertex(i)));
    }

    const auto& seg = tbData.stopEventGraph;
    for (Vertex from(0); from < seg.numVertices(); from++) {
        if (static_cast<size_t>(from) >= tbData.numberOfStopEvents()) continue;
        const StopId fromStop = stopOfEvent(tbData, StopEventId(from));
        if (static_cast<size_t>(fromStop) >= numStops) continue;

        for (const Edge edge : seg.edgesFrom(from)) {
            const Vertex to = seg.get(ToVertex, edge);
            if (static_cast<size_t>(to) >= tbData.numberOfStopEvents()) continue;
            const StopId toStop = stopOfEvent(tbData, StopEventId(to));
            if (static_cast<size_t>(toStop) >= numStops) continue;
            if (fromStop == toStop) continue;

            const int travelTime = seg.get(TravelTime, edge);

            const size_t prevEdgeCount = graph.numEdges();
            const Edge newEdge = graph.findOrAddEdge(
                Vertex(fromStop), Vertex(toStop));
            if (graph.numEdges() != prevEdgeCount) {
                graph.set(TravelTime, newEdge, travelTime);
            } else {
                graph.set(TravelTime, newEdge,
                    std::min(graph.get(TravelTime, newEdge), travelTime));
            }
        }
    }

    graph.packEdges();
    return graph;
}

inline RAPTOR::Data buildShortcutRaptorData(
        const RAPTOR::Data& raptorData,
        const Intermediate::TransferGraph& shortcutGraph) noexcept {
    RAPTOR::Data result = raptorData;

    const size_t numVertices =
        raptorData.transferGraph.numVertices();

    Intermediate::TransferGraph dynGraph;
    dynGraph.addVertices(numVertices);

    for (Vertex v(0); v < numVertices; v++) {
        dynGraph.set(Coordinates, v,
            raptorData.transferGraph.get(Coordinates, v));
    }

    for (Vertex from(0); from < shortcutGraph.numVertices(); from++) {
        for (const Edge edge : shortcutGraph.edgesFrom(from)) {
            const Edge ne = dynGraph.addEdge(
                from, shortcutGraph.get(ToVertex, edge));
            dynGraph.set(TravelTime, ne,
                shortcutGraph.get(TravelTime, edge));
        }
    }

    Graph::move(std::move(dynGraph), result.transferGraph);

    if (!result.hasImplicitBufferTimes()) {
        result.useImplicitDepartureBufferTimes();
    }

    return result;
}

inline CSA::Data buildCSADataWithShortcuts(
        const RAPTOR::Data& raptorData,
        Intermediate::TransferGraph shortcutGraph) noexcept {

    std::vector<CSA::Stop> stops;
    stops.reserve(raptorData.numberOfStops());
    for (const StopId stop : raptorData.stops()) {
        stops.emplace_back(raptorData.stopData[stop]);
    }

    std::vector<CSA::Connection> connections;
    TripId nextTripId(0);
    for (const RouteId route : raptorData.routes()) {
        const StopId* routeStops = raptorData.stopArrayOfRoute(route);
        const size_t numStops = raptorData.numberOfStopsInRoute(route);
        const size_t numTrips = raptorData.numberOfTripsInRoute(route);
        for (size_t t = 0; t < numTrips; t++) {
            const RAPTOR::StopEvent* events =
                raptorData.tripOfRoute(route, t);
            for (size_t i = 0; i + 1 < numStops; i++) {
                const int explicitDepartureTime =
                    events[i].departureTime +
                    raptorData.stopData[routeStops[i]].minTransferTime;
                connections.emplace_back(
                    routeStops[i], routeStops[i + 1],
                    explicitDepartureTime,
                    events[i + 1].arrivalTime,
                    nextTripId);
            }
            nextTripId++;
        }
    }

    std::sort(connections.begin(), connections.end(),
        [](const CSA::Connection& a, const CSA::Connection& b) {
            return a.departureTime < b.departureTime;
        });

    std::vector<CSA::Trip> trips(static_cast<size_t>(nextTripId));

    return CSA::Data::FromInput<false>(stops, connections, trips,
                                       std::move(shortcutGraph));
}

inline Intermediate::Data buildIntermediateFromRAPTOR(
        const RAPTOR::Data& raptorData) noexcept {
    Intermediate::Data result;

    result.transferGraph.addVertices(
        raptorData.transferGraph.numVertices());
    for (Vertex v(0);
         v < raptorData.transferGraph.numVertices(); v++) {
        result.transferGraph.set(Coordinates, v,
            raptorData.transferGraph.get(Coordinates, v));
        for (const Edge e : raptorData.transferGraph.edgesFrom(v)) {
            const Vertex to =
                raptorData.transferGraph.get(ToVertex, e);
            const int tt =
                raptorData.transferGraph.get(TravelTime, e);
            const Edge ne = result.transferGraph.addEdge(v, to);
            result.transferGraph.set(TravelTime, ne, tt);
        }
    }

    for (const StopId stop : raptorData.stops()) {
        result.stops.emplace_back(raptorData.stopData[stop]);
    }

    for (const RouteId route : raptorData.routes()) {
        const StopId* routeStops =
            raptorData.stopArrayOfRoute(route);
        const size_t numStops =
            raptorData.numberOfStopsInRoute(route);
        const size_t numTrips =
            raptorData.numberOfTripsInRoute(route);

        for (size_t t = 0; t < numTrips; t++) {
            result.trips.emplace_back();
            Intermediate::Trip& trip = result.trips.back();

            const RAPTOR::StopEvent* events =
                raptorData.tripOfRoute(route, t);

            for (size_t s = 0; s < numStops; s++) {
                const int explicitDep =
                    events[s].departureTime +
                    raptorData.stopData[routeStops[s]].minTransferTime;
                trip.stopEvents.emplace_back(
                    routeStops[s],
                    events[s].arrivalTime,
                    explicitDep);
            }
        }
    }

    return result;
}

inline void printCSAQuality(
        const std::vector<VertexQuery>& queries,
        const std::vector<std::vector<RAPTOR::ArrivalLabel>>& exactResults,
        const std::vector<std::vector<RAPTOR::ArrivalLabel>>& csaResults
        ) noexcept {
    const size_t total = queries.size();
    size_t correct = 0, missed = 0, suboptimal = 0;
    std::vector<double> detours;

    for (size_t i = 0; i < total; i++) {
        const bool exactReachable = !exactResults[i].empty();
        const bool csaReachable   = !csaResults[i].empty();

        if (!exactReachable && !csaReachable) { correct++; continue; }
        if (exactReachable && !csaReachable)  { missed++;  continue; }
        if (!exactReachable && csaReachable)   { correct++; continue; }

        int exactEarliest = INFTY;
        for (const auto& label : exactResults[i])
            exactEarliest = std::min(exactEarliest, label.arrivalTime);
        const int csaArrival = csaResults[i].front().arrivalTime;

        if (csaArrival <= exactEarliest) {
            correct++;
        } else {
            suboptimal++;
            const int tt = exactEarliest - queries[i].departureTime;
            if (tt > 0) detours.emplace_back(
                (csaArrival - exactEarliest) / static_cast<double>(tt));
        }
    }

    std::cout << "  Total queries:    " << total << std::endl;
    std::cout << "  Correct:          " << correct << " ("
              << String::percent(correct / static_cast<double>(total))
              << ")" << std::endl;
    std::cout << "  Missed:           " << missed << " ("
              << String::percent(missed / static_cast<double>(total))
              << ")" << std::endl;
    std::cout << "  Suboptimal:       " << suboptimal << " ("
              << String::percent(suboptimal / static_cast<double>(total))
              << ")" << std::endl;
    if (!detours.empty()) {
        std::sort(detours.begin(), detours.end());
        std::cout << "  Mean detour:      "
                  << String::percent(Vector::mean(detours)) << std::endl;
        std::cout << "  Median detour:    "
                  << String::percent(Vector::median(detours)) << std::endl;
        std::cout << "  95th percentile:  "
                  << String::percent(Vector::percentile(detours, 0.95))
                  << std::endl;
    }
}

inline void printTDQuality(
        const std::vector<VertexQuery>& queries,
        const std::vector<std::vector<RAPTOR::ArrivalLabel>>& exactResults,
        const std::vector<int>& tdResults
        ) noexcept {
    const size_t total = queries.size();
    size_t correct = 0, missed = 0, suboptimal = 0, mismatchCount = 0;
    int maxDiff = 0;
    double totalDiff = 0;

    for (size_t i = 0; i < total; i++) {
        int mrEarliest = INFTY;
        for (const auto& label : exactResults[i])
            mrEarliest = std::min(mrEarliest, label.arrivalTime);
        const bool mrReachable = (mrEarliest < INFTY);
        const bool tdReachable = (tdResults[i] != never && tdResults[i] != intMax);

        if (!mrReachable && !tdReachable) { correct++; continue; }
        if (mrReachable && !tdReachable)  { missed++;  continue; }
        if (!mrReachable && tdReachable)   { correct++; continue; }

        if (tdResults[i] == mrEarliest) {
            correct++;
        } else {
            suboptimal++;
            int diff = tdResults[i] - mrEarliest;
            if (diff > maxDiff) maxDiff = diff;
            totalDiff += std::abs(diff);
            mismatchCount++;
            if (mismatchCount <= 5) {
                std::cout << "  Mismatch query " << i
                          << ": MR=" << mrEarliest
                          << ", alg=" << tdResults[i]
                          << " (diff=" << diff << "s)" << std::endl;
            }
        }
    }

    std::cout << "  Total queries:    " << total << std::endl;
    std::cout << "  Correct:          " << correct << " ("
              << String::percent(correct / static_cast<double>(total))
              << ")" << std::endl;
    std::cout << "  Missed:           " << missed << " ("
              << String::percent(missed / static_cast<double>(total))
              << ")" << std::endl;
    std::cout << "  Mismatches:       " << suboptimal << " ("
              << String::percent(suboptimal / static_cast<double>(total))
              << ")" << std::endl;
    if (mismatchCount > 0) {
        std::cout << "  Max difference:   " << maxDiff << "s" << std::endl;
        std::cout << "  Avg difference:   "
                  << (totalDiff / mismatchCount) << "s" << std::endl;
    }
}

} // namespace DelayHelpers

// ═══════════════════════════════════════════════════════════════════════════
//  ANALYSIS COMMANDS
// ═══════════════════════════════════════════════════════════════════════════

class AnalyzeHeadwayDistribution : public ParameterizedCommand {

public:
    AnalyzeHeadwayDistribution(BasicShell& shell) :
        ParameterizedCommand(shell, "analyzeHeadwayDistribution", "Analyzes the headway distribution of routes in the given network.") {
        addParameter("RAPTOR data");
    }

    virtual void execute() noexcept {
        const RAPTOR::Data data(getParameter("RAPTOR data"));
        data.printInfo();

        size_t numRoutes = 0;
        std::vector<size_t> headwayLimits { 5, 10, 20, 30, 60, 120, 240, INFTY };
        std::vector<size_t> numRoutesPerHeadway(headwayLimits.size(), 0);
        for (const RouteId route : data.routes()) {
            size_t numTrips = 0;
            const int firstDepartureTime = data.firstTripOfRoute(route)->departureTime;
            int lastDepartureTime = firstDepartureTime;
            for (size_t trip = 0; trip < data.numberOfTripsInRoute(route); trip++) {
                const int tripDepartureTime = data.tripOfRoute(route, trip)->departureTime;
                if (tripDepartureTime > 24 * 60 * 60) break;
                lastDepartureTime = tripDepartureTime;
                numTrips++;
            }
            const int timespan = lastDepartureTime - firstDepartureTime;
            if (numTrips <= 1) continue;
            numRoutes++;
            const double headway =  static_cast<double>(timespan) / (numTrips - 1);
            for (size_t i = 0; i < headwayLimits.size(); i++) {
                if (headway <= headwayLimits[i] * 60) {
                    numRoutesPerHeadway[i]++;
                    break;
                }
            }
        }
        std::cout << "Routes with <= 1 trips: " << String::percent((data.numberOfRoutes() - numRoutes)/static_cast<double>(data.numberOfRoutes())) << std::endl;
        for (size_t i = 0; i < headwayLimits.size(); i++) {
            std::cout << "Headway <= " << headwayLimits[i] << " min: " << String::percent(numRoutesPerHeadway[i]/static_cast<double>(numRoutes)) << std::endl;
        }
    }
};

class AnalyzeTransferSlacks : public ParameterizedCommand {

public:
    AnalyzeTransferSlacks(BasicShell& shell) :
        ParameterizedCommand(shell, "analyzeTransferSlacks", "Analyzes the transfer slacks of optimal journeys for random queries.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept {
        const TripBased::Data data(getParameter("Trip-Based data"));
        data.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        TripBased::Query<TripBased::AggregateProfiler> tripBased(data, bucketCH);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(bucketCH.numVertices(), n);

        std::vector<int> allTransferSlacks;
        std::vector<int> minTransferSlacks;
        std::vector<int> maxTransferSlacks;
        Progress progress(n);
        for (const VertexQuery& query : queries) {
            tripBased.run(query.source, query.departureTime, query.target);
            const std::vector<JourneyData> journeyData = getJourneyData(data, tripBased.getJourneys());
            for (const JourneyData& journey : journeyData) {
                allTransferSlacks += journey.getTransferSlacks(data);
                if (journey.transfers.empty()) continue;
                minTransferSlacks.emplace_back(journey.getMinTransferSlack(data));
                maxTransferSlacks.emplace_back(journey.getMaxTransferSlack(data));
            }
            progress++;
        }

        std::sort(allTransferSlacks.begin(), allTransferSlacks.end());
        std::sort(minTransferSlacks.begin(), minTransferSlacks.end());
        std::sort(maxTransferSlacks.begin(), maxTransferSlacks.end());
        std::cout << "All transfer slacks:" << std::endl;
        printSlacks(allTransferSlacks);
        std::cout << "Min transfer slacks:" << std::endl;
        printSlacks(minTransferSlacks);
        std::cout << "Max transfer slacks:" << std::endl;
        printSlacks(maxTransferSlacks);
    }

private:
    inline void printSlacks(std::vector<int>& slacks) const noexcept {
        std::sort(slacks.begin(), slacks.end());
        std::cout << "\tMin: " << Vector::min(slacks) << std::endl;
        std::cout << "\tMax: " << Vector::max(slacks) << std::endl;
        std::cout << "\tMean: " << Vector::mean(slacks) << std::endl;
        std::cout << "\tPercentiles:" << std::endl;
        for (double p = 0.05; p < 1; p += 0.05) {
            std::cout << "\t\t" << p << ": " << Vector::percentile(slacks, p) << std::endl;
        }
    }
};

class BuildFakeDelayData : public ParameterizedCommand {
public:
    BuildFakeDelayData(BasicShell& shell) :
        ParameterizedCommand(shell, "buildFakeDelayData", "Builds fake TripBased::DelayData with delay buffer 0 from TripBased::Data.") {
        addParameter("Trip-Based input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        TripBased::Data data(getParameter("Trip-Based input file"));
        TripBased::DelayData delayData(data);
        delayData.printInfo();
        delayData.serialize(getParameter("Output file"));
    }
};

class GenerateDelayScenario : public ParameterizedCommand {
public:
    GenerateDelayScenario(BasicShell& shell) :
        ParameterizedCommand(shell, "generateDelayScenario", "Generates a random delay scenario for the given network.") {
        addParameter("Trip-Based data");
        addParameter("Output file");
        addParameter("Start time", "00:00:00");
        addParameter("End time", "24:00:00");
        addParameter("Seed", "42");
    }

    virtual void execute() noexcept {
        const TripBased::Data data(getParameter("Trip-Based data"));
        data.printInfo();
        const int startTime = String::parseSeconds(getParameter("Start time"));
        const int endTime = String::parseSeconds(getParameter("End time"));
        const int seed = getParameter<int>("Seed");
        const TripBased::DelayScenario delayScenario(data, startTime, endTime, seed);
        delayScenario.serialize(getParameter("Output file"));
    }
};

class ValidateDelayULTRATripBased : public ParameterizedCommand {

public:
    ValidateDelayULTRATripBased(BasicShell& shell) :
        ParameterizedCommand(shell, "validateDelayULTRATripBased", "Validates journeys computed by ULTRA-TripBased with delay-tolerant shortcuts by comparing to Dijkstra-RAPTOR on random queries.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Trip-Based data");
        addParameter("Core CH data");
        addParameter("Bucket CH data");
        addParameter("Number of queries");
        addParameter("Delay scenario file");
        addParameter("Ignore max delay?");
        addParameter("Allow departure delays?");
        addParameter("Allow overtaking?");
    }

    virtual void execute() noexcept {
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        CH::CH coreCH(getParameter("Core CH data"));
        CH::CH bucketCH(getParameter("Bucket CH data"));

        const TripBased::DelayScenario delayScenario(delayData.data, getParameter("Delay scenario file"));
        const bool ignoreMaxDelay = getParameter<bool>("Ignore max delay?");
        const bool allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        const bool allowOvertaking = getParameter<bool>("Allow overtaking?");
        const TripBased::DelayQueryData queryData = delayData.applyDelayScenario(delayScenario.getAllIncidents(), allowDepartureDelays, ignoreMaxDelay, allowOvertaking);

        RAPTOR::Data delayedRaptorData = queryData.tripData.raptorData;
        Graph::move(std::move(dijkstraRaptorData.transferGraph), delayedRaptorData.transferGraph);
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> dijkstraRaptor(delayedRaptorData, coreCH);
        TripBased::Query<TripBased::AggregateProfiler> ultraTripBased(queryData.tripData, bucketCH);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(bucketCH.numVertices(), n);

        Progress progress(n);
        for (size_t i = 0; i < n; i++) {
            dijkstraRaptor.run(queries[i].source, queries[i].departureTime, queries[i].target);
            ultraTripBased.run(queries[i].source, queries[i].departureTime, queries[i].target);
            const std::vector<RAPTOR::ArrivalLabel> dijkstraArrivals = dijkstraRaptor.getArrivals();
            const std::vector<RAPTOR::ArrivalLabel> ultraArrivals = ultraTripBased.getArrivals();
            if (!Vector::equals(dijkstraArrivals, ultraArrivals)) {
                std::cout << "Query " << i << ": " << queries[i] << std::endl;
                std::cout << "Dijkstra arrivals:" << std::endl;
                std::cout << dijkstraArrivals << std::endl;
                std::cout << "ULTRA arrivals:" << std::endl;
                std::cout << ultraArrivals << std::endl;
                std::cout << "Dijkstra journeys:" << std::endl;
                for (const RAPTOR::Journey& journey : dijkstraRaptor.getJourneys()) {
                    std::cout << journey << std::endl;
                    std::cout << "Transfers:" << std::endl;
                    const JourneyData journeyData(queryData.tripData, journey);
                    for (const Transfer& transfer : journeyData.transfers) {
                        std::cout << transfer;
                        std::cout << ". In data? " << queryData.tripData.stopEventGraph.hasEdge(Vertex(transfer.from), Vertex(transfer.to));
                        const StopEventId originalFrom = StopEventId(queryData.internalToOriginal[transfer.from]);
                        const StopEventId originalTo = StopEventId(queryData.internalToOriginal[transfer.to]);
                        std::cout << ". Original: " << Transfer(originalFrom, originalTo, transfer.travelTime);
                        const Edge originalEdge = delayData.stopEventGraph.findEdge(Vertex(originalFrom), Vertex(originalTo));
                        std::cout << ". In original data? " << (originalEdge != noEdge);
                        if (originalEdge != noEdge) {
                            std::cout << ". Min origin delay: " << delayData.stopEventGraph.get(MinOriginDelay, originalEdge);
                            std::cout << ". Max origin delay: " << delayData.stopEventGraph.get(MaxOriginDelay, originalEdge);
                        }
                        std::cout << ". From delay: " << delayData.arrivalDelay[originalFrom];
                        std::cout << ". To delay: " << delayData.departureDelay[originalTo];
                        std::cout << std::endl;
                    }
                    std::cout << std::endl;
                }
                std::cout << "ULTRA journeys:" << std::endl;
                std::cout << ultraTripBased.getJourneys() << std::endl;
                return;
            }
            progress++;
        }
        dijkstraRaptor.getProfiler().printStatistics();
        ultraTripBased.getProfiler().printStatistics();
    }
};

class RunDelayUpdatesWithReplacement : public ParameterizedCommand {

public:
    RunDelayUpdatesWithReplacement(BasicShell& shell) :
        ParameterizedCommand(shell, "runDelayUpdatesWithReplacement", "Runs Delay-ULTRA update phase with replacement search for the given delay scenario.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Delay scenario file");
        addParameter("Allow departure delays?");
        addParameter("Replace delayed targets?");
        addParameter("Statistics file");
        addParameter("Log file");
        addParameter("Target slack");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }
    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(bucketCH.forward, bucketCH.backward, delayData.data.numberOfStops(), Weight);

        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");
        const ThreadPinning threadPinning(numberOfThreads, pinMultiplier);
        const int targetSlack = getParameter<int>("Target slack");
        TripBased::DelayUpdaterWithReplacement delayUpdater(delayData, initialTransfers, threadPinning, targetSlack);

        const TripBased::DelayScenario delayScenario(delayData.data, getParameter("Delay scenario file"));
        const bool allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        const bool replaceDelayedTargets = getParameter<bool>("Replace delayed targets?");
        delayUpdater.preRunUpdate(delayScenario.preStartUpdates, allowDepartureDelays, replaceDelayedTargets);
        delayUpdater.runUpdates(delayScenario.updates, allowDepartureDelays, replaceDelayedTargets);
        delayUpdater.writeStatistics(getParameter("Statistics file"));
        delayUpdater.writeLog(getParameter("Log file"));
    }

private:
    inline int getNumberOfThreads() const noexcept {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }
};

class RunDelayUpdatesWithoutReplacement : public ParameterizedCommand {

public:
    RunDelayUpdatesWithoutReplacement(BasicShell& shell) :
        ParameterizedCommand(shell, "runDelayUpdatesWithoutReplacement", "Runs Delay-ULTRA update phase without replacement search for the given delay scenario.") {
        addParameter("Trip-Based data");
        addParameter("Delay scenario file");
        addParameter("Allow departure delays?");
        addParameter("Statistics file");
        addParameter("Log file");
    }

    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        TripBased::DelayUpdaterWithoutReplacement delayUpdater(delayData);
        const TripBased::DelayScenario delayScenario(delayData.data, getParameter("Delay scenario file"));
        const bool allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        delayUpdater.preRunUpdate(delayScenario.preStartUpdates, allowDepartureDelays);
        delayUpdater.runUpdates(delayScenario.updates, allowDepartureDelays);
        delayUpdater.writeStatistics(getParameter("Statistics file"));
        delayUpdater.writeLog(getParameter("Log file"));
    }
};

class GenerateDelayQueries : public ParameterizedCommand {

public:
    GenerateDelayQueries(BasicShell& shell) :
        ParameterizedCommand(shell, "generateDelayQueries", "Generates queries for which an algorithm without delay information returns incorrect results.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Core CH data");
        addParameter("Number of queries");
        addParameter("Delay scenario file");
        addParameter("Allow departure delays?");
        addParameter("Output file");
    }

    virtual void execute() noexcept {
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();
        TripBased::DelayData originalDelayData(dijkstraRaptorData, INFTY, INFTY);
        const CH::CH coreCH(getParameter("Core CH data"));
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> undelayedRaptor(dijkstraRaptorData, coreCH);

        const TripBased::DelayScenario delayScenario(originalDelayData.data, getParameter("Delay scenario file"));
        QueryInputData queryInputData;
        queryInputData.allowDepartureDelays = getParameter<bool>("Allow departure delays?");
        std::vector<VertexQuery> allQueries;
        std::vector<std::vector<RAPTOR::ArrivalLabel>> delayedResults;
        QueryFeasibilityStatistics feasibilityStatistics;

        std::mt19937 randomGenerator(42);
        std::uniform_int_distribution<> vertexDistribution(0, coreCH.numVertices() - 1);
        std::uniform_int_distribution<> timeDistribution(delayScenario.startTime, delayScenario.endTime - 1);

        const size_t n = getParameter<size_t>("Number of queries");
        Progress progress(n);
        while (queryInputData.failedQueries.size() < n) {
            const VertexQuery query(Vertex(vertexDistribution(randomGenerator)), Vertex(vertexDistribution(randomGenerator)), timeDistribution(randomGenerator));
            allQueries.emplace_back(query);
            undelayedRaptor.run(query.source, query.departureTime, query.target);
            const std::vector<JourneyData> undelayedData = getJourneyData(originalDelayData, undelayedRaptor.getJourneys());

            TripBased::DelayData delayData = originalDelayData;
            const std::vector<TripBased::DelayIncident> queryDelayScenario = delayScenario.getDelayIncidentsUntil(query);
            delayData.applyDelays(queryDelayScenario, true, queryInputData.allowDepartureDelays);
            feasibilityStatistics.addQuery(undelayedData, delayData);

            const RAPTOR::Data delayedRaptorData = getDelayedRaptorData(delayData, dijkstraRaptorData);
            RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> delayedRaptor(delayedRaptorData, coreCH);
            delayedRaptor.run(query.source, query.departureTime, query.target);
            delayedResults.emplace_back(delayedRaptor.getArrivals());

            if (!Vector::equals(delayedResults.back(), feasibilityStatistics.algorithmResults.back())) {
                queryInputData.failedQueries.emplace_back(query);
                queryInputData.failedQueryResults.emplace_back(delayedResults.back());
                progress++;
            }
        }
        progress.finished();

        feasibilityStatistics.finalize();
        const QueryStatistics statistics(allQueries, delayedResults, feasibilityStatistics.algorithmResults);
        std::cout << statistics << feasibilityStatistics << std::endl;

        queryInputData.sortQueries();
        queryInputData.groupedDelaysByQuery = delayScenario.groupDelayIncidentsByQuery(queryInputData.failedQueries);
        queryInputData.serialize(getParameter("Output file"));
    }

private:
    inline RAPTOR::Data getDelayedRaptorData(const TripBased::DelayData& delayData, const RAPTOR::Data& raptorData) const noexcept {
        RAPTOR::Data delayedRaptorData = raptorData;
        for (StopEventId event(0); event < delayedRaptorData.numberOfStopEvents(); event++) {
            delayedRaptorData.stopEvents[event].arrivalTime += delayData.arrivalDelay[event];
            delayedRaptorData.stopEvents[event].departureTime += delayData.departureDelay[event];
        }
        delayedRaptorData.rebuildRoutes();
        return delayedRaptorData;
    }
};

class MeasureDelayULTRAQueryCoverage : public ParameterizedCommand {

public:
    MeasureDelayULTRAQueryCoverage(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayULTRAQueryCoverage", "Evaluates query coverage of Delay-ULTRA-TB.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Query input data");
        addParameter("Update log file");
    }

    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(bucketCH.forward, bucketCH.backward, delayData.data.numberOfStops(), Weight);

        const QueryInputData queryInputData(getParameter("Query input data"));
        const std::vector<VertexQuery>& queries = queryInputData.failedQueries;

        TripBased::DelayData updateDelayData = delayData;
        TripBased::DelayUpdateSimulator delayUpdater(updateDelayData, getParameter("Update log file"));
        QueryFeasibilityStatistics feasibilityStatistics;
        Progress progress(queries.size());
        for (size_t i = 0; i < queries.size(); i++) {
            delayUpdater.applyUpdatesUntil(queries[i].departureTime, queryInputData.allowDepartureDelays);
            delayData.applyDelays(queryInputData.groupedDelaysByQuery[i], true, queryInputData.allowDepartureDelays);
            const TripBased::DelayQueryData& queryData = delayUpdater.getQueryData();
            TripBased::Query<TripBased::AggregateProfiler> algorithm(queryData.tripData, initialTransfers);
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target);
            const std::vector<JourneyData> journeyData = getJourneyData(delayUpdater.getDelayData(), queryData.tripData, queryData.internalToOriginal, algorithm.getJourneys());
            feasibilityStatistics.addQuery(journeyData, delayData);
            progress++;
        }

        feasibilityStatistics.finalize();
        const QueryStatistics statistics(queries, queryInputData.failedQueryResults, feasibilityStatistics.algorithmResults);
        std::cout << statistics << feasibilityStatistics << std::endl;
    }
};

class MeasureHypotheticalDelayULTRAQueryCoverage : public ParameterizedCommand {

public:
    MeasureHypotheticalDelayULTRAQueryCoverage(BasicShell& shell) :
        ParameterizedCommand(shell, "measureHypotheticalDelayULTRAQueryCoverage", "Evaluates query coverage of Delay-ULTRA-TB, assuming that replacement shortcuts are known in advance and updates are incorporated instantly.") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Query input data");
        addParameter("Update log file");
    }

    virtual void execute() noexcept {
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(bucketCH.forward, bucketCH.backward, delayData.data.numberOfStops(), Weight);

        const QueryInputData queryInputData(getParameter("Query input data"));
        const std::vector<VertexQuery>& queries = queryInputData.failedQueries;

        TripBased::applyAllReplacementShortcutsFromUpdateLog(delayData, getParameter("Update log file"));
        TripBased::DelayQueryData queryData = delayData.createQueryData();

        QueryFeasibilityStatistics feasibilityStatistics;
        Progress progress(queries.size());
        for (size_t i = 0; i < queries.size(); i++) {
            delayData.applyDelays(queryInputData.groupedDelaysByQuery[i], true, queryInputData.allowDepartureDelays);
            delayData.refreshQueryData(queryData);
            TripBased::Query<TripBased::AggregateProfiler> algorithm(queryData.tripData, initialTransfers);
            algorithm.run(queries[i].source, queries[i].departureTime, queries[i].target);
            const std::vector<JourneyData> journeyData = getJourneyData(delayData, queryData.tripData, queryData.internalToOriginal, algorithm.getJourneys());
            feasibilityStatistics.addQuery(journeyData, delayData);
            progress++;
        }

        feasibilityStatistics.finalize();
        const QueryStatistics statistics(queries, queryInputData.failedQueryResults, feasibilityStatistics.algorithmResults);
        std::cout << statistics << feasibilityStatistics << std::endl;
    }
};

class MeasureDelayULTRAQueryPerformance : public ParameterizedCommand {

public:
    MeasureDelayULTRAQueryPerformance(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayULTRAQueryPerformance", "Measures query performance of ULTRA-TB with delay shortcuts and replacement search.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Trip-Based data");
        addParameter("Core CH data");
        addParameter("Bucket CH data");
        addParameter("Update log file");
        addParameter("Number of queries");
        addParameter("Start time", "14:00:00");
        addParameter("End time", "15:00:00");
    }

    virtual void execute() noexcept {
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        const CH::CH coreCH(getParameter("Core CH data"));
        const CH::CH bucketCH(getParameter("Bucket CH data"));

        const int startTime = String::parseSeconds(getParameter("Start time"));
        const int endTime = String::parseSeconds(getParameter("End time"));
        TripBased::DelayUpdateSimulator delayUpdater(delayData, getParameter("Update log file"));
        delayUpdater.applyUpdatesUntil(startTime, true);
        const TripBased::DelayQueryData& queryData = delayUpdater.getQueryData();

        RAPTOR::Data delayedRaptorData = queryData.tripData.raptorData;
        Graph::move(std::move(dijkstraRaptorData.transferGraph), delayedRaptorData.transferGraph);
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers, RAPTOR::AggregateProfiler> dijkstraRaptor(delayedRaptorData, coreCH);
        TripBased::Query<TripBased::AggregateProfiler> ultraTripBased(queryData.tripData, bucketCH);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries = generateRandomVertexQueries(bucketCH.numVertices(), n, startTime, endTime);
        const std::vector<std::vector<RAPTOR::ArrivalLabel>> dijkstraResults = runQueries(queries, dijkstraRaptor);
        const std::vector<std::vector<RAPTOR::ArrivalLabel>> ultraResults = runQueries(queries, ultraTripBased);
        const QueryStatistics statistics(queries, dijkstraResults, ultraResults);
        std::cout << statistics << std::endl;
    }

private:
    template<typename ALGORITHM>
    inline std::vector<std::vector<RAPTOR::ArrivalLabel>> runQueries(const std::vector<VertexQuery>& queries, ALGORITHM& algorithm) noexcept {
        Progress progress(queries.size());
        std::vector<std::vector<RAPTOR::ArrivalLabel>> results;
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            results.emplace_back(algorithm.getArrivals());
            progress++;
        }
        algorithm.getProfiler().printStatistics();
        return results;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
//  PER-QUERY COVERAGE (hypothetical mode, all algorithms)
// ═══════════════════════════════════════════════════════════════════════════

class MeasureDelayQueryCoverage : public ParameterizedCommand {

public:
    MeasureDelayQueryCoverage(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayQueryCoverage",
            "Evaluates per-query coverage of TB, RAPTOR, RAPTOR-EP, and CSA "
            "using hypothetical mode (all replacement shortcuts known in advance).") {
        addParameter("Trip-Based data");
        addParameter("Bucket CH data");
        addParameter("Query input data");
        addParameter("Update log file");
        addParameter("Number of queries", "0");
    }

    virtual void execute() noexcept {
        // =================================================================
        //  1. Load delay data and apply all replacement shortcuts upfront
        // =================================================================
        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();
        TripBased::applyAllReplacementShortcutsFromUpdateLog(
            delayData, getParameter("Update log file"));
        TripBased::DelayQueryData queryData = delayData.createQueryData();

        // =================================================================
        //  2. Load Bucket CH and build initial transfers (once)
        // =================================================================
        const CH::CH bucketCH(getParameter("Bucket CH data"));
        const RAPTOR::BucketCHInitialTransfers initialTransfers(
            bucketCH.forward, bucketCH.backward,
            delayData.data.numberOfStops(), Weight);

        // =================================================================
        //  3. Load ground truth (affected queries + per-query delays)
        // =================================================================
        const QueryInputData queryInputData(getParameter("Query input data"));
        const size_t requestedQueries = getParameter<size_t>("Number of queries");
        const size_t numQueries = (requestedQueries == 0)
            ? queryInputData.failedQueries.size()
            : std::min(requestedQueries, queryInputData.failedQueries.size());
        const std::vector<VertexQuery> queries(
            queryInputData.failedQueries.begin(),
            queryInputData.failedQueries.begin() + numQueries);
        std::cout << "Using " << queries.size() << " of "
                  << queryInputData.failedQueries.size()
                  << " affected queries." << std::endl;

        // =================================================================
        //  4. Per-query coverage loop
        // =================================================================
        std::vector<std::vector<RAPTOR::ArrivalLabel>> tbResults;
        std::vector<std::vector<RAPTOR::ArrivalLabel>> raptorResults;
        std::vector<std::vector<RAPTOR::ArrivalLabel>> raptorEPResults;
        std::vector<std::vector<RAPTOR::ArrivalLabel>> csaResults;
        tbResults.reserve(queries.size());
        raptorResults.reserve(queries.size());
        raptorEPResults.reserve(queries.size());
        csaResults.reserve(queries.size());

        Progress progress(queries.size());
        for (size_t i = 0; i < queries.size(); i++) {
            // --- Apply this query's delays ---
            delayData.applyDelays(queryInputData.groupedDelaysByQuery[i],
                                  true, queryInputData.allowDepartureDelays);
            delayData.refreshQueryData(queryData);

            // --- ULTRA-TB ---
            TripBased::Query<TripBased::NoProfiler> tbAlgorithm(
                queryData.tripData, initialTransfers);
            tbAlgorithm.run(queries[i].source, queries[i].departureTime,
                            queries[i].target);
            tbResults.emplace_back(tbAlgorithm.getArrivals());

            // --- Convert event shortcuts to stop-level graph ---
            Intermediate::TransferGraph shortcutGraph =
                DelayHelpers::convertShortcutsToStopGraph(
                    queryData.tripData.raptorData, queryData.tripData);

            // --- ULTRA-RAPTOR ---
            RAPTOR::Data shortcutRaptorData =
                DelayHelpers::buildShortcutRaptorData(
                    queryData.tripData.raptorData, shortcutGraph);
            RAPTOR::ULTRARAPTOR<RAPTOR::NoProfiler> raptorAlgorithm(
                shortcutRaptorData, initialTransfers);
            raptorAlgorithm.run(queries[i].source, queries[i].departureTime,
                                queries[i].target);
            raptorResults.emplace_back(raptorAlgorithm.getArrivals());

            // --- ULTRA-RAPTOR (EP) ---
            RAPTOR::Data shortcutRaptorDataEP = shortcutRaptorData;
            shortcutRaptorDataEP.sortTransferGraphEdgesByTravelTime();
            RAPTOR::ULTRARAPTOR_prune<RAPTOR::NoProfiler> raptorEPAlgorithm(
                shortcutRaptorDataEP, initialTransfers);
            raptorEPAlgorithm.run(queries[i].source, queries[i].departureTime,
                                  queries[i].target);
            raptorEPResults.emplace_back(raptorEPAlgorithm.getArrivals());

            // --- Delay-ULTRA-CSA ---
            CSA::Data csaData = DelayHelpers::buildCSADataWithShortcuts(
                queryData.tripData.raptorData, shortcutGraph);
            CSA::DelayULTRACSA<false> csaAlgorithm(csaData, bucketCH);
            csaAlgorithm.run(queries[i].source, queries[i].departureTime,
                             queries[i].target);
            csaResults.emplace_back(csaAlgorithm.getArrivals());

            progress++;
        }

        // =================================================================
        //  5. Quality comparisons against ground truth
        // =================================================================
        std::cout << "\n=== ULTRA-TB Coverage ===" << std::endl;
        const QueryStatistics tbStats(queries,
            queryInputData.failedQueryResults, tbResults);
        std::cout << tbStats << std::endl;

        std::cout << "=== ULTRA-RAPTOR Coverage ===" << std::endl;
        const QueryStatistics raptorStats(queries,
            queryInputData.failedQueryResults, raptorResults);
        std::cout << raptorStats << std::endl;

        std::cout << "=== ULTRA-RAPTOR (EP) Coverage ===" << std::endl;
        const QueryStatistics raptorEPStats(queries,
            queryInputData.failedQueryResults, raptorEPResults);
        std::cout << raptorEPStats << std::endl;

        std::cout << "=== Delay-ULTRA-CSA Coverage ===" << std::endl;
        DelayHelpers::printCSAQuality(queries,
            queryInputData.failedQueryResults, csaResults);
    }
};

// =====================================================================
//  DROP-IN REPLACEMENT for MeasureDelayULTRACSAQueryPerformance
//
//  Changes vs original:
//    1. Added TAD (TimeDependentDijkstraBucketCH)
//    2. Renamed "ULTRA-RAPTOR (prune)" → "ULTRA-RAPTOR (EP)" in labels
//    3. Made bucketCH non-const (BucketCH constructor needs mutable ptr)
//
//  REQUIRED INCLUDES — add these near the top of DelayExperiments.h
//  if not already present:
//
//    #include "../../Algorithms/Dijkstra/TimeDependentDijkstraBucketCH.h"
//    #include "../../Algorithms/Dijkstra/TimeDependentDijkstra.h"
// =====================================================================

class MeasureDelayULTRACSAQueryPerformance : public ParameterizedCommand {

public:
    MeasureDelayULTRACSAQueryPerformance(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayULTRACSAQueryPerformance",
            "Measures query performance of Delay-ULTRA-CSA, ULTRA-RAPTOR, "
            "ULTRA-RAPTOR (EP), ULTRA-TB, MR, TD-Dijkstra, and TAD.") {
        addParameter("Dijkstra RAPTOR data");
        addParameter("Trip-Based data");
        addParameter("Core CH data");
        addParameter("Bucket CH data");
        addParameter("Update log file");
        addParameter("Number of queries");
        addParameter("Start time", "14:00:00");
        addParameter("End time", "15:00:00");
    }

    virtual void execute() noexcept {
        // =================================================================
        //  1. Load input data
        // =================================================================
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();

        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();

        CH::CH coreCH(getParameter("Core CH data"));   // needed for MR only
        CH::CH bucketCH(getParameter("Bucket CH data"));

        // =================================================================
        //  2. Apply delay updates
        // =================================================================
        const int startTime = String::parseSeconds(getParameter("Start time"));
        const int endTime   = String::parseSeconds(getParameter("End time"));

        TripBased::DelayUpdateSimulator delayUpdater(
            delayData, getParameter("Update log file"));
        delayUpdater.applyUpdatesUntil(startTime, true);
        const TripBased::DelayQueryData& queryData = delayUpdater.getQueryData();

        // =================================================================
        //  3. Convert TB shortcuts to stop-level graph (shared by CSA,
        //     ULTRA-RAPTOR, ULTRA-RAPTOR (EP))
        // =================================================================
        Intermediate::TransferGraph shortcutGraph =
            DelayHelpers::convertShortcutsToStopGraph(
                queryData.tripData.raptorData, queryData.tripData);
        std::cout << "  Shortcut conversion: "
                  << queryData.tripData.stopEventGraph.numEdges()
                  << " stop-event edges -> "
                  << shortcutGraph.numEdges()
                  << " stop edges" << std::endl;

        // =================================================================
        //  4. MR baseline (Dijkstra-RAPTOR with walking transfers)
        // =================================================================
        RAPTOR::Data delayedRaptorData = queryData.tripData.raptorData;
        Graph::move(std::move(dijkstraRaptorData.transferGraph),
                    delayedRaptorData.transferGraph);
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers,
                               RAPTOR::AggregateProfiler>
            dijkstraRaptor(delayedRaptorData, coreCH);

        // =================================================================
        //  5. ULTRA-TB query
        // =================================================================
        TripBased::Query<TripBased::AggregateProfiler>
            ultraTripBased(queryData.tripData, bucketCH);

        // =================================================================
        //  6. CSA data + query
        // =================================================================
        CSA::Data csaData = DelayHelpers::buildCSADataWithShortcuts(
            queryData.tripData.raptorData, shortcutGraph);
        csaData.printInfo();

        const std::vector<int> emptyDelayVec;
        CSA::DelayULTRACSA<false> delayCSA(
            csaData, bucketCH, emptyDelayVec, 0);

        // =================================================================
        //  7. ULTRA-RAPTOR variants (shortcuts-only transfer graph)
        // =================================================================
        RAPTOR::Data shortcutRaptorData =
            DelayHelpers::buildShortcutRaptorData(queryData.tripData.raptorData,
                                    shortcutGraph);
        std::cout << "  Shortcut RAPTOR data: "
                  << shortcutRaptorData.transferGraph.numVertices()
                  << " vertices, "
                  << shortcutRaptorData.transferGraph.numEdges()
                  << " edges" << std::endl;

        // Standard ULTRA-RAPTOR
        RAPTOR::ULTRARAPTOR<RAPTOR::AggregateProfiler>
            ultraRaptor(shortcutRaptorData, bucketCH);

        // EP variant needs edges sorted by travel time
        RAPTOR::Data shortcutRaptorDataEP = shortcutRaptorData;
        shortcutRaptorDataEP.sortTransferGraphEdgesByTravelTime();
        RAPTOR::ULTRARAPTOR_prune<RAPTOR::AggregateProfiler>
            ultraRaptorEP(shortcutRaptorDataEP, bucketCH);

        // =================================================================
        //  8. Build time-dependent graphs from delayed timetable
        // =================================================================
        std::cout << "Building time-dependent graphs from delayed timetable..."
                  << std::endl;
        Intermediate::Data delayedIntermediate =
            DelayHelpers::buildIntermediateFromRAPTOR(delayedRaptorData);

        // 8a. TAD: JTS graph (no domination filtering) + BucketCH
        TimeDependentGraph tdGraph =
            TimeDependentGraph::FromIntermediate(delayedIntermediate);
        std::cout << "TD graph created: "
                  << tdGraph.numVertices() << " vertices, "
                  << tdGraph.numEdges() << " edges" << std::endl;

        using TADType = TransferAwareDijkstraBucketCH<
            TimeDependentGraph, TDD::AggregateProfiler, false, true>;
        TADType tadAlgorithm(
            tdGraph, delayedRaptorData.numberOfStops(), &bucketCH);

        // 8b. TD-Dijkstra: Classic graph (domination filtering) + BucketCH
        TimeDependentGraphClassic tdGraphClassic =
            TimeDependentGraphClassic::FromIntermediate(delayedIntermediate);
        std::cout << "Classic TD graph created: "
                  << tdGraphClassic.numVertices() << " vertices, "
                  << tdGraphClassic.numEdges() << " edges" << std::endl;

        using TDDijkstraType = TimeDependentDijkstraBucketCH<
            TimeDependentGraphClassic, TDD::AggregateProfiler, false, true>;
        TDDijkstraType tdDijkstra(
            tdGraphClassic, delayedRaptorData.numberOfStops(), &bucketCH);

        // =================================================================
        //  9. Generate random queries
        // =================================================================
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries =
            generateRandomVertexQueries(bucketCH.numVertices(), n,
                                        startTime, endTime);

        // =================================================================
        //  10. Run all algorithms
        // =================================================================
        std::cout << "\n--- MR (Dijkstra-RAPTOR) ---" << std::endl;
        Timer timer;
        const auto dijkstraResults = runQueries(queries, dijkstraRaptor);
        const double mrTime = timer.elapsedMilliseconds();

        std::cout << "\n--- TAD (BucketCH) ---" << std::endl;
        std::vector<int> tadResults;
        tadResults.reserve(n);
        timer.restart();
        {
            Progress progress(n);
            for (size_t i = 0; i < n; ++i) {
                const VertexQuery& q = queries[i];
                tadAlgorithm.run(q.source, q.departureTime, q.target);
                tadResults.push_back(tadAlgorithm.getArrivalTime(q.target));
                progress++;
            }
        }
        const double tadTime = timer.elapsedMilliseconds();

        std::cout << "\n--- TD-Dijkstra (Classic, BucketCH) ---" << std::endl;
        std::vector<int> tdResults;
        tdResults.reserve(n);
        timer.restart();
        {
            Progress progress(n);
            for (size_t i = 0; i < n; ++i) {
                const VertexQuery& q = queries[i];
                tdDijkstra.run(q.source, q.departureTime, q.target);
                tdResults.push_back(tdDijkstra.getArrivalTime(q.target));
                progress++;
            }
        }
        const double tdTime = timer.elapsedMilliseconds();

        std::cout << "\n--- ULTRA-TB ---" << std::endl;
        timer.restart();
        const auto tbResults = runQueries(queries, ultraTripBased);
        const double tbTime = timer.elapsedMilliseconds();

        std::cout << "\n--- ULTRA-RAPTOR ---" << std::endl;
        timer.restart();
        const auto urResults = runQueries(queries, ultraRaptor);
        const double urTime = timer.elapsedMilliseconds();

        std::cout << "\n--- ULTRA-RAPTOR (EP) ---" << std::endl;
        timer.restart();
        const auto urpResults = runQueries(queries, ultraRaptorEP);
        const double urpTime = timer.elapsedMilliseconds();

        std::cout << "\n--- Delay-ULTRA-CSA ---" << std::endl;
        timer.restart();
        const auto csaResults = runCSAQueries(queries, delayCSA);
        const double csaTime = timer.elapsedMilliseconds();

        // =================================================================
        //  11. Timing summary
        // =================================================================
        std::cout << "\n=== Timing Summary ===" << std::endl;
        std::cout << "  Total queries:           " << n << std::endl;
        std::cout << std::fixed;
        auto printTime = [&](const char* label, double ms) {
            std::cout << "  " << std::left << std::setw(35) << label
                      << std::setprecision(1) << ms << " ms  ("
                      << std::setprecision(3) << (ms / n) << " ms/query)"
                      << std::endl;
        };
        printTime("MR total:", mrTime);
        printTime("TAD total:", tadTime);
        printTime("TD-Dijkstra (Classic) total:", tdTime);
        printTime("ULTRA-TB total:", tbTime);
        printTime("ULTRA-RAPTOR total:", urTime);
        printTime("ULTRA-RAPTOR (EP) total:", urpTime);
        printTime("Delay-ULTRA-CSA total:", csaTime);

        std::cout << std::setprecision(2);
        auto printSpeedup = [&](const char* label, double num, double den) {
            if (den > 0) {
                std::cout << "  " << std::left << std::setw(35) << label
                          << (num / den) << "x" << std::endl;
            }
        };
        printSpeedup("Speedup CSA vs MR:", mrTime, csaTime);
        printSpeedup("Speedup CSA vs TAD:", tadTime, csaTime);
        printSpeedup("Speedup CSA vs TD-Dijkstra:", tdTime, csaTime);
        printSpeedup("Speedup CSA vs TB:", tbTime, csaTime);
        printSpeedup("Speedup CSA vs UR:", urTime, csaTime);
        printSpeedup("Speedup CSA vs URP:", urpTime, csaTime);
        printSpeedup("Speedup TAD vs MR:", mrTime, tadTime);
        printSpeedup("Speedup TD-Dijkstra vs MR:", mrTime, tdTime);

        // =================================================================
        //  12. Quality comparisons
        // =================================================================
        std::cout << "\n=== Quality: MR vs ULTRA-TB ===" << std::endl;
        const QueryStatistics tbStats(queries, dijkstraResults, tbResults);
        std::cout << tbStats << std::endl;

        std::cout << "=== Quality: MR vs ULTRA-RAPTOR ===" << std::endl;
        const QueryStatistics urStats(queries, dijkstraResults, urResults);
        std::cout << urStats << std::endl;

        std::cout << "=== Quality: MR vs ULTRA-RAPTOR (EP) ===" << std::endl;
        const QueryStatistics urpStats(queries, dijkstraResults, urpResults);
        std::cout << urpStats << std::endl;

        std::cout << "=== Quality: MR vs Delay-ULTRA-CSA ===" << std::endl;
        DelayHelpers::printCSAQuality(queries, dijkstraResults, csaResults);

        std::cout << "\n=== Quality: MR vs TAD ===" << std::endl;
        DelayHelpers::printTDQuality(queries, dijkstraResults, tadResults);

        std::cout << "\n=== Quality: MR vs TD-Dijkstra (Classic) ===" << std::endl;
        DelayHelpers::printTDQuality(queries, dijkstraResults, tdResults);
    }

private:
    // =====================================================================
    //  Query runners
    // =====================================================================
    template<typename ALGORITHM>
    inline std::vector<std::vector<RAPTOR::ArrivalLabel>> runQueries(
            const std::vector<VertexQuery>& queries,
            ALGORITHM& algorithm) const noexcept {
        Progress progress(queries.size());
        std::vector<std::vector<RAPTOR::ArrivalLabel>> results;
        results.reserve(queries.size());
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            results.emplace_back(algorithm.getArrivals());
            progress++;
        }
        algorithm.getProfiler().printStatistics();
        return results;
    }

    template<typename CSA_ALGO>
    inline std::vector<std::vector<RAPTOR::ArrivalLabel>> runCSAQueries(
            const std::vector<VertexQuery>& queries,
            CSA_ALGO& algorithm) const noexcept {
        Progress progress(queries.size());
        std::vector<std::vector<RAPTOR::ArrivalLabel>> results;
        results.reserve(queries.size());
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            results.emplace_back(algorithm.getArrivals());
            progress++;
        }
        return results;
    }
};