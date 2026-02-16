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

/**
 * MeasureDelayULTRACSAQueryPerformance
 *
 * Compares Delay-ULTRA-CSA vs Delay-ULTRA-TB vs MR on one-to-one queries,
 * using the SAME delay-tolerant shortcuts for both CSA and TB.
 *
 * The TB shortcuts are stop-event-indexed (fromStopEvent -> toStopEvent).
 * CSA needs stop-indexed transfers (fromStop -> toStop).
 * We convert by mapping each stop event to its stop and keeping
 * the minimum travel time per (fromStop, toStop) pair.
 *
 * IMPORTANT: The RAPTOR data inside queryData.tripData has IMPLICIT
 * departure buffer times (departureTime already reduced by minTransferTime).
 * Since CSA's connectionIsReachableFromStop() explicitly subtracts
 * minTransferTime, we must RESTORE the original departure times when
 * building connections to avoid double-counting the buffer.
 *
 * Required includes at the top of DelayExperiments.h:
 *
 *   #include "../../Algorithms/CSA/DelayULTRACSA.h"
 *   #include "../../DataStructures/CSA/Data.h"
 */

class MeasureDelayULTRACSAQueryPerformance : public ParameterizedCommand {

public:
    MeasureDelayULTRACSAQueryPerformance(BasicShell& shell) :
        ParameterizedCommand(shell, "measureDelayULTRACSAQueryPerformance",
            "Measures query performance of Delay-ULTRA-CSA vs "
            "Delay-ULTRA-TB and MR, using TB delay shortcuts "
            "converted to stop-indexed format for CSA.") {
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
        // -- 1. Load input data -------------------------------------------
        RAPTOR::Data dijkstraRaptorData(getParameter("Dijkstra RAPTOR data"));
        dijkstraRaptorData.useImplicitDepartureBufferTimes();
        dijkstraRaptorData.printInfo();

        // Extract stop-to-stop walking edges from the full transfer graph
        // BEFORE we move it away for MR. Used for diagnostics and control test.
        std::map<std::pair<int,int>, int> walkingEdges;
        for (StopId from(0); from < dijkstraRaptorData.numberOfStops(); from++) {
            for (const Edge edge : dijkstraRaptorData.transferGraph.edgesFrom(Vertex(from))) {
                const Vertex to = dijkstraRaptorData.transferGraph.get(ToVertex, edge);
                if (to >= dijkstraRaptorData.numberOfStops()) continue;
                if (Vertex(from) == to) continue;
                const int tt = dijkstraRaptorData.transferGraph.get(TravelTime, edge);
                auto key = std::make_pair(int(from), int(to));
                if (walkingEdges.find(key) == walkingEdges.end() || walkingEdges[key] > tt) {
                    walkingEdges[key] = tt;
                }
            }
        }
        std::cout << "  Stop-to-stop walking edges in original graph: "
                  << walkingEdges.size() << std::endl;

        TripBased::DelayData delayData(getParameter("Trip-Based data"));
        delayData.printInfo();

        const CH::CH coreCH(getParameter("Core CH data"));
        const CH::CH bucketCH(getParameter("Bucket CH data"));

        // -- 2. Apply delay updates ---------------------------------------
        const int startTime = String::parseSeconds(getParameter("Start time"));
        const int endTime   = String::parseSeconds(getParameter("End time"));

        TripBased::DelayUpdateSimulator delayUpdater(
            delayData, getParameter("Update log file"));
        delayUpdater.applyUpdatesUntil(startTime, true);
        const TripBased::DelayQueryData& queryData = delayUpdater.getQueryData();

        // -- 3. MR baseline (Dijkstra-RAPTOR) -----------------------------
        RAPTOR::Data delayedRaptorData = queryData.tripData.raptorData;
        Graph::move(std::move(dijkstraRaptorData.transferGraph),
                    delayedRaptorData.transferGraph);
        RAPTOR::DijkstraRAPTOR<RAPTOR::CoreCHInitialTransfers,
                               RAPTOR::AggregateProfiler>
            dijkstraRaptor(delayedRaptorData, coreCH);

        // -- 4. ULTRA-TB query --------------------------------------------
        TripBased::Query<TripBased::AggregateProfiler>
            ultraTripBased(queryData.tripData, bucketCH);

        // -- 5. Build CSA::Data from delayed RAPTOR + converted shortcuts -
        const RAPTOR::Data& csaRaptorSource = queryData.tripData.raptorData;
        std::cout << "\nRAPTOR data used for CSA connection building:" << std::endl;
        std::cout << "  hasImplicitBufferTimes: "
                  << csaRaptorSource.hasImplicitBufferTimes() << std::endl;
        std::cout << "  numberOfRoutes: " << csaRaptorSource.numberOfRoutes() << std::endl;
        std::cout << "  numberOfStops: " << csaRaptorSource.numberOfStops() << std::endl;
        // Show first trip's first few stop events
        if (csaRaptorSource.numberOfRoutes() > 0) {
            const RouteId r(0);
            const StopId* stops = csaRaptorSource.stopArrayOfRoute(r);
            const size_t nStops = csaRaptorSource.numberOfStopsInRoute(r);
            const RAPTOR::StopEvent* events = csaRaptorSource.firstTripOfRoute(r);
            std::cout << "  Route 0 first trip, first 3 events:" << std::endl;
            for (size_t i = 0; i < std::min(nStops, size_t(3)); i++) {
                std::cout << "    stop=" << stops[i]
                          << " dep=" << events[i].departureTime
                          << " arr=" << events[i].arrivalTime
                          << " minTransfer=" << csaRaptorSource.stopData[stops[i]].minTransferTime
                          << std::endl;
            }
        }

        CSA::Data csaData = buildCSAData(
            csaRaptorSource, queryData.tripData);

        std::cout << "\nCSA data (delayed, with converted TB shortcuts):" << std::endl;
        csaData.printInfo();

        // Diagnostic: show first few connections and some stats
        std::cout << "\n  First 5 connections:" << std::endl;
        for (size_t i = 0; i < std::min(size_t(5), csaData.connections.size()); i++) {
            const auto& c = csaData.connections[i];
            std::cout << "    [" << i << "] stop " << c.departureStopId
                      << " -> " << c.arrivalStopId
                      << "  dep=" << String::secToTime(c.departureTime)
                      << " (" << c.departureTime << ")"
                      << "  arr=" << String::secToTime(c.arrivalTime)
                      << " (" << c.arrivalTime << ")"
                      << "  trip=" << c.tripId << std::endl;
        }
        // Check if any connections have negative travel time
        size_t negativeConnections = 0;
        for (const auto& c : csaData.connections) {
            if (c.arrivalTime < c.departureTime) negativeConnections++;
        }
        std::cout << "  Connections with negative travel time: "
                  << negativeConnections << std::endl;

        // Verify connections are sorted by departure time
        {
            size_t sortErrors = 0;
            int lastDep = -1;
            for (size_t ci = 0; ci < csaData.connections.size(); ci++) {
                const int dep = csaData.connections[ci].departureTime;
                if (dep < lastDep) {
                    sortErrors++;
                    if (sortErrors <= 3) {
                        std::cout << "  SORT ERROR at [" << ci << "]: dep="
                                  << dep << " < prev=" << lastDep << std::endl;
                    }
                }
                lastDep = dep;
            }
            std::cout << "  Connection sort errors: " << sortErrors << std::endl;
            if (csaData.connections.size() > 0) {
                std::cout << "  First conn dep: "
                          << csaData.connections.front().departureTime << " ("
                          << String::secToTime(csaData.connections.front().departureTime)
                          << ")" << std::endl;
                std::cout << "  Last conn dep:  "
                          << csaData.connections.back().departureTime << " ("
                          << String::secToTime(csaData.connections.back().departureTime)
                          << ")" << std::endl;
                // Sample around where dep=51748 would land
                size_t midIdx = csaData.connections.size() / 2;
                std::cout << "  Mid conn [" << midIdx << "] dep: "
                          << csaData.connections[midIdx].departureTime << " ("
                          << String::secToTime(csaData.connections[midIdx].departureTime)
                          << ")" << std::endl;
            }
        }

        // Show shortcut edge stats
        int minShortcut = INFTY, maxShortcut = 0;
        long long totalShortcut = 0;
        for (const Vertex v : csaData.transferGraph.vertices()) {
            for (const Edge e : csaData.transferGraph.edgesFrom(v)) {
                const int tt = csaData.transferGraph.get(TravelTime, e);
                minShortcut = std::min(minShortcut, tt);
                maxShortcut = std::max(maxShortcut, tt);
                totalShortcut += tt;
            }
        }
        if (csaData.transferGraph.numEdges() > 0) {
            std::cout << "  Shortcut travel times: min="
                      << minShortcut << "s max=" << maxShortcut
                      << "s avg=" << (totalShortcut / csaData.transferGraph.numEdges())
                      << "s" << std::endl;
        }

        // Diagnostic: how many walking edges are covered by shortcuts?
        size_t covered = 0, missing = 0, shortcutOnly = 0;
        size_t ttMatch = 0, ttMismatch = 0;
        double totalWalkTT = 0, totalShortcutTT = 0;
        std::vector<std::pair<int,int>> mismatchSamples; // (walkTT, shortcutTT)
        for (const auto& [key, walkTT] : walkingEdges) {
            bool found = false;
            const Vertex from(key.first);
            for (const Edge edge : csaData.transferGraph.edgesFrom(from)) {
                if (static_cast<int>(csaData.transferGraph.get(ToVertex, edge)) == key.second) {
                    found = true;
                    const int scTT = csaData.transferGraph.get(TravelTime, edge);
                    if (scTT == walkTT) {
                        ttMatch++;
                    } else {
                        ttMismatch++;
                        totalWalkTT += walkTT;
                        totalShortcutTT += scTT;
                        if (mismatchSamples.size() < 10) {
                            mismatchSamples.emplace_back(walkTT, scTT);
                        }
                    }
                    break;
                }
            }
            if (found) covered++;
            else missing++;
        }
        for (const Vertex v : csaData.transferGraph.vertices()) {
            for (const Edge edge : csaData.transferGraph.edgesFrom(v)) {
                const int to = static_cast<int>(csaData.transferGraph.get(ToVertex, edge));
                auto key = std::make_pair(int(v), to);
                if (walkingEdges.find(key) == walkingEdges.end()) shortcutOnly++;
            }
        }
        std::cout << "\n  Walking edge coverage by TB shortcuts:" << std::endl;
        std::cout << "    Walking edges (stop-to-stop): " << walkingEdges.size() << std::endl;
        std::cout << "    Covered by shortcuts: " << covered << std::endl;
        std::cout << "    MISSING from shortcuts: " << missing << std::endl;
        std::cout << "    Shortcut-only (no walking): " << shortcutOnly << std::endl;
        std::cout << "    TravelTime matches: " << ttMatch << std::endl;
        std::cout << "    TravelTime MISMATCHES: " << ttMismatch << std::endl;
        if (ttMismatch > 0) {
            std::cout << "    Avg walk TT (mismatched): " << (totalWalkTT / ttMismatch) << "s" << std::endl;
            std::cout << "    Avg shortcut TT (mismatched): " << (totalShortcutTT / ttMismatch) << "s" << std::endl;
            std::cout << "    Sample mismatches (walk vs shortcut):" << std::endl;
            for (const auto& [w, s] : mismatchSamples) {
                std::cout << "      walk=" << w << "s  shortcut=" << s << "s" << std::endl;
            }
        }

        // -- 6. Delay-ULTRA-CSA query (TB shortcuts) ----------------------
        const std::vector<int> emptyDelayVec;
        CSA::DelayULTRACSA<false> delayCSA(
            csaData, bucketCH, emptyDelayVec, 0);

        // -- 6b. CSA with WALKING EDGES as transfer graph -----------------
        //    This is the real test: does using actual walking edges fix it?
        CSA::Data walkingCSAData = csaData;
        {
            Intermediate::TransferGraph walkGraph;
            walkGraph.addVertices(csaData.numberOfStops());
            for (size_t i = 0; i < csaData.numberOfStops(); i++) {
                walkGraph.set(Coordinates, Vertex(i),
                    csaData.transferGraph.get(Coordinates, Vertex(i)));
            }
            for (const auto& [key, tt] : walkingEdges) {
                const Edge e = walkGraph.findOrAddEdge(Vertex(key.first), Vertex(key.second));
                walkGraph.set(TravelTime, e, tt);
            }
            walkGraph.packEdges();
            std::cout << "\n  Walking transfer graph: " << walkGraph.numEdges()
                      << " edges" << std::endl;
            Graph::move(std::move(walkGraph), walkingCSAData.transferGraph);
        }
        CSA::DelayULTRACSA<false> walkingCSA(
            walkingCSAData, bucketCH, emptyDelayVec, 0);

        // -- 7. Generate random queries -----------------------------------
        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<VertexQuery> queries =
            generateRandomVertexQueries(bucketCH.numVertices(), n,
                                        startTime, endTime);

        // -- 8. Run MR queries --------------------------------------------
        std::cout << "\n--- MR (Dijkstra-RAPTOR) ---" << std::endl;
        Timer timer;
        const std::vector<std::vector<RAPTOR::ArrivalLabel>> dijkstraResults =
            runQueries(queries, dijkstraRaptor);
        const double mrTime = timer.elapsedMilliseconds();

        // -- 9. Run ULTRA-TB queries --------------------------------------
        std::cout << "\n--- ULTRA-TB ---" << std::endl;
        timer.restart();
        const std::vector<std::vector<RAPTOR::ArrivalLabel>> tbResults =
            runQueries(queries, ultraTripBased);
        const double tbTime = timer.elapsedMilliseconds();

        // -- 10. Run Delay-ULTRA-CSA queries -------------------------------
        std::cout << "\n--- Delay-ULTRA-CSA ---" << std::endl;
        timer.restart();
        std::vector<std::vector<RAPTOR::ArrivalLabel>> csaResults;
        csaResults.reserve(queries.size());
        {
            Progress progress(queries.size());
            for (size_t qi = 0; qi < queries.size(); qi++) {
                delayCSA.resetDebugCounters();
                delayCSA.run(queries[qi].source, queries[qi].departureTime, queries[qi].target);
                csaResults.emplace_back(delayCSA.getArrivals());
                if (qi < 5) {
                    std::cout << "  Query " << qi << " debug:" << std::endl;
                    delayCSA.printDebugCounters();
                }
                progress++;
            }
        }
        const double csaTime = timer.elapsedMilliseconds();

        // -- 10b. Run CSA with walking edges --------------------------------
        std::cout << "\n--- CSA with walking edges ---" << std::endl;
        timer.restart();
        std::vector<std::vector<RAPTOR::ArrivalLabel>> walkResults;
        walkResults.reserve(queries.size());
        {
            Progress progress(queries.size());
            for (size_t qi = 0; qi < queries.size(); qi++) {
                walkingCSA.resetDebugCounters();
                walkingCSA.run(queries[qi].source, queries[qi].departureTime, queries[qi].target);
                walkResults.emplace_back(walkingCSA.getArrivals());
                if (qi < 5) {
                    std::cout << "  Query " << qi << " (walking) debug:" << std::endl;
                    walkingCSA.printDebugCounters();
                }
                progress++;
            }
        }
        const double walkTime = timer.elapsedMilliseconds();

        // -- 11. Diagnostic: compare first 5 queries in detail --------
        std::cout << "\n=== Diagnostic: First 5 Queries ===" << std::endl;
        for (size_t i = 0; i < std::min(n, size_t(5)); i++) {
            const VertexQuery& q = queries[i];
            std::cout << "  Query " << i << ": src=" << q.source
                      << " tgt=" << q.target
                      << " dep=" << String::secToTime(q.departureTime)
                      << " (" << q.departureTime << "s)"
                      << std::endl;

            // MR result
            int mrEarliest = never;
            for (const auto& label : dijkstraResults[i]) {
                mrEarliest = std::min(mrEarliest, label.arrivalTime);
            }
            std::cout << "    MR:      ";
            if (mrEarliest < never) {
                std::cout << String::secToTime(mrEarliest)
                          << " (" << mrEarliest << "s)"
                          << " travel=" << (mrEarliest - q.departureTime) << "s"
                          << " labels=" << dijkstraResults[i].size();
            } else {
                std::cout << "unreachable";
            }
            std::cout << std::endl;

            // TB result
            int tbEarliest = never;
            for (const auto& label : tbResults[i]) {
                tbEarliest = std::min(tbEarliest, label.arrivalTime);
            }
            std::cout << "    TB:      ";
            if (tbEarliest < never) {
                std::cout << String::secToTime(tbEarliest)
                          << " (" << tbEarliest << "s)"
                          << " travel=" << (tbEarliest - q.departureTime) << "s"
                          << " labels=" << tbResults[i].size();
            } else {
                std::cout << "unreachable";
            }
            std::cout << std::endl;

            // CSA result
            std::cout << "    CSA:     ";
            if (!csaResults[i].empty()) {
                const int csaArr = csaResults[i].front().arrivalTime;
                std::cout << String::secToTime(csaArr)
                          << " (" << csaArr << "s)"
                          << " travel=" << (csaArr - q.departureTime) << "s";
            } else {
                std::cout << "unreachable";
            }
            std::cout << std::endl;

            // Walking CSA result
            std::cout << "    WalkCSA: ";
            if (!walkResults[i].empty()) {
                const int wArr = walkResults[i].front().arrivalTime;
                std::cout << String::secToTime(wArr)
                          << " (" << wArr << "s)"
                          << " travel=" << (wArr - q.departureTime) << "s";
            } else {
                std::cout << "unreachable";
            }
            std::cout << std::endl;

            // Check if source/target are stops
            std::cout << "    srcIsStop=" << csaData.isStop(Vertex(q.source))
                      << " tgtIsStop=" << csaData.isStop(Vertex(q.target))
                      << " numStops=" << csaData.numberOfStops()
                      << std::endl;

            // Check if CSA == WalkCSA
            const int csaArr2 = csaResults[i].empty() ? never : csaResults[i].front().arrivalTime;
            const int walkArr2 = walkResults[i].empty() ? never : walkResults[i].front().arrivalTime;
            if (csaArr2 == walkArr2) {
                std::cout << "    CSA == WalkCSA (shortcuts same as walking)" << std::endl;
            } else if (walkArr2 < csaArr2) {
                std::cout << "    *** WalkCSA BETTER by " << (csaArr2 - walkArr2) << "s ***" << std::endl;
            } else {
                std::cout << "    Shortcuts better by " << (walkArr2 - csaArr2) << "s" << std::endl;
            }
            // Compare WalkCSA vs MR
            if (walkArr2 <= mrEarliest) {
                std::cout << "    WalkCSA vs MR: CORRECT" << std::endl;
            } else if (walkArr2 < never) {
                std::cout << "    WalkCSA vs MR: detour=" << (walkArr2 - mrEarliest) << "s" << std::endl;
            }
        }

        // -- 12. Report timing --------------------------------------------
        std::cout << "\n=== Timing Summary ===" << std::endl;
        std::cout << "  Total queries:         " << n << std::endl;
        std::cout << "  MR total:              "
                  << std::fixed << std::setprecision(1)
                  << mrTime << " ms  ("
                  << std::setprecision(3)
                  << (mrTime / n) << " ms/query)" << std::endl;
        std::cout << "  ULTRA-TB total:        "
                  << std::setprecision(1)
                  << tbTime << " ms  ("
                  << std::setprecision(3)
                  << (tbTime / n) << " ms/query)" << std::endl;
        std::cout << "  Delay-ULTRA-CSA total: "
                  << std::setprecision(1)
                  << csaTime << " ms  ("
                  << std::setprecision(3)
                  << (csaTime / n) << " ms/query)" << std::endl;

        if (csaTime > 0) {
            std::cout << "  Speedup CSA vs TB:     "
                      << std::setprecision(2)
                      << (tbTime / csaTime) << "x" << std::endl;
            std::cout << "  Speedup CSA vs MR:     "
                      << std::setprecision(2)
                      << (mrTime / csaTime) << "x" << std::endl;
        }

        // -- 12. Quality: MR vs ULTRA-TB ----------------------------------
        std::cout << "\n=== Quality: MR vs ULTRA-TB ===" << std::endl;
        const QueryStatistics tbStats(queries, dijkstraResults, tbResults);
        std::cout << tbStats << std::endl;

        // -- 13. Quality: MR vs CSA (earliest-arrival comparison) ---------
        std::cout << "=== Quality: MR vs Delay-ULTRA-CSA "
                     "(TB shortcuts, earliest arrival) ===" << std::endl;
        printCSAQuality(queries, dijkstraResults, csaResults);

        std::cout << "\n=== Quality: MR vs Delay-ULTRA-CSA "
                     "(walking edges, earliest arrival) ===" << std::endl;
        printCSAQuality(queries, dijkstraResults, walkResults);
    }

private:
    // =====================================================================
    //  Build CSA::Data from delayed RAPTOR data + converted TB shortcuts
    // =====================================================================
    //
    //  CRITICAL: The RAPTOR data has IMPLICIT departure buffer times,
    //  meaning departureTime was already reduced by minTransferTime(stop).
    //  CSA's connectionIsReachableFromStop() explicitly subtracts
    //  minTransferTime, so we must ADD IT BACK to departure times here
    //  to avoid double-counting.
    //
    inline CSA::Data buildCSAData(
            const RAPTOR::Data& raptorData,
            const TripBased::Data& tbData) const noexcept {

        // 1. Build CSA stops from RAPTOR stops (uses template constructor)
        std::vector<CSA::Stop> stops;
        stops.reserve(raptorData.numberOfStops());
        for (const StopId stop : raptorData.stops()) {
            stops.emplace_back(raptorData.stopData[stop]);
        }

        // 2. Build CSA connections from delayed RAPTOR timetable
        //    RESTORE original departure times by adding back minTransferTime
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
                    // Restore explicit departure time:
                    // stored departureTime = original - minTransferTime
                    // so original = stored + minTransferTime
                    const int explicitDepartureTime =
                        events[i].departureTime +
                        raptorData.stopData[routeStops[i]].minTransferTime;
                    connections.emplace_back(
                        routeStops[i],
                        routeStops[i + 1],
                        explicitDepartureTime,
                        events[i + 1].arrivalTime,
                        nextTripId
                    );
                }
                nextTripId++;
            }
        }

        // 3. Build CSA trips (minimal -- just need the count)
        std::vector<CSA::Trip> trips(static_cast<size_t>(nextTripId));

        // 3b. SORT connections by departure time (CRITICAL for CSA!)
        std::sort(connections.begin(), connections.end(),
            [](const CSA::Connection& a, const CSA::Connection& b) {
                return a.departureTime < b.departureTime;
            });
        std::cout << "  Connections sorted by dep time: " << connections.size()
                  << " (first dep=" << (connections.empty() ? 0 : connections.front().departureTime)
                  << ", last dep=" << (connections.empty() ? 0 : connections.back().departureTime)
                  << ")" << std::endl;

        // 4. Convert TB shortcuts to stop-indexed transfer graph
        std::cout << "  TB stopEventGraph: "
                  << tbData.stopEventGraph.numVertices() << " vertices, "
                  << tbData.stopEventGraph.numEdges() << " edges"
                  << std::endl;
        std::cout << "  TB numberOfStopEvents: "
                  << tbData.numberOfStopEvents() << std::endl;
        // Show a few shortcut edges before conversion
        size_t edgesPrinted = 0;
        for (Vertex from(0); from < tbData.stopEventGraph.numVertices() && edgesPrinted < 3; from++) {
            for (const Edge edge : tbData.stopEventGraph.edgesFrom(from)) {
                if (edgesPrinted >= 3) break;
                const Vertex to = tbData.stopEventGraph.get(ToVertex, edge);
                const int tt = tbData.stopEventGraph.get(TravelTime, edge);
                const StopId fromStop = stopOfEvent(tbData, StopEventId(from));
                const StopId toStop = stopOfEvent(tbData, StopEventId(to));
                std::cout << "    SE edge: event " << from << "(stop " << fromStop
                          << ") -> event " << to << "(stop " << toStop
                          << ") tt=" << tt << std::endl;
                edgesPrinted++;
            }
        }

        Intermediate::TransferGraph shortcutGraph =
            convertShortcutsToStopGraph(raptorData, tbData);

        std::cout << "  Shortcut conversion: "
                  << tbData.stopEventGraph.numEdges()
                  << " stop-event edges -> "
                  << shortcutGraph.numEdges()
                  << " stop edges" << std::endl;

        // 5. Build CSA::Data via FromInput (MAKE_BIDIRECTIONAL=false
        //    because shortcuts are directional)
        return CSA::Data::FromInput<false>(stops, connections, trips,
                                           std::move(shortcutGraph));
    }

    // =====================================================================
    //  Convert TB stop-event-indexed shortcuts to stop-indexed graph
    // =====================================================================
    inline Intermediate::TransferGraph convertShortcutsToStopGraph(
            const RAPTOR::Data& raptorData,
            const TripBased::Data& tbData) const noexcept {
        const size_t numStops = raptorData.numberOfStops();

        Intermediate::TransferGraph graph;
        graph.addVertices(numStops);

        // Copy coordinates
        for (size_t i = 0; i < numStops; i++) {
            graph.set(Coordinates, Vertex(i),
                raptorData.transferGraph.get(Coordinates, Vertex(i)));
        }

        // Iterate TB stopEventGraph edges and map to stops
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

    // Map a stop event to its stop
    inline StopId stopOfEvent(const TripBased::Data& tbData,
                              const StopEventId event) const noexcept {
        const TripId trip = tbData.tripOfStopEvent[event];
        const StopIndex idx = tbData.indexOfStopEvent[event];
        return tbData.stopArrayOfTrip(trip)[idx];
    }

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

    template<typename CSA_ALGO>
    inline std::vector<std::vector<RAPTOR::ArrivalLabel>> runStdCSAQueries(
            const std::vector<VertexQuery>& queries,
            CSA_ALGO& algorithm) const noexcept {
        Progress progress(queries.size());
        std::vector<std::vector<RAPTOR::ArrivalLabel>> results;
        results.reserve(queries.size());
        for (const VertexQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            std::vector<RAPTOR::ArrivalLabel> labels;
            const int arr = algorithm.getEarliestArrivalTime(query.target);
            if (arr < never) {
                labels.emplace_back(arr, 1);
            }
            results.emplace_back(std::move(labels));
            progress++;
        }
        return results;
    }

    // =====================================================================
    //  Quality comparison
    // =====================================================================
    inline void printCSAQuality(
            const std::vector<VertexQuery>& queries,
            const std::vector<std::vector<RAPTOR::ArrivalLabel>>& exactResults,
            const std::vector<std::vector<RAPTOR::ArrivalLabel>>& csaResults
            ) const noexcept {
        const size_t total = queries.size();
        size_t correct    = 0;
        size_t missed     = 0;
        size_t suboptimal = 0;
        std::vector<double> detours;

        for (size_t i = 0; i < total; i++) {
            const bool exactReachable = !exactResults[i].empty();
            const bool csaReachable   = !csaResults[i].empty();

            if (!exactReachable && !csaReachable) {
                correct++;
                continue;
            }
            if (exactReachable && !csaReachable) {
                missed++;
                continue;
            }
            if (!exactReachable && csaReachable) {
                correct++;
                continue;
            }

            int exactEarliest = INFTY;
            for (const auto& label : exactResults[i]) {
                exactEarliest = std::min(exactEarliest, label.arrivalTime);
            }
            const int csaArrival = csaResults[i].front().arrivalTime;

            if (csaArrival <= exactEarliest) {
                correct++;
            } else {
                suboptimal++;
                const int travelTime =
                    exactEarliest - queries[i].departureTime;
                if (travelTime > 0) {
                    detours.emplace_back(
                        (csaArrival - exactEarliest)
                        / static_cast<double>(travelTime));
                }
            }
        }

        std::cout << "  Total queries:    " << total << std::endl;
        std::cout << "  Correct:          " << correct
                  << " (" << String::percent(
                         correct / static_cast<double>(total))
                  << ")" << std::endl;
        std::cout << "  Missed:           " << missed
                  << " (" << String::percent(
                         missed / static_cast<double>(total))
                  << ")" << std::endl;
        std::cout << "  Suboptimal:       " << suboptimal
                  << " (" << String::percent(
                         suboptimal / static_cast<double>(total))
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
};