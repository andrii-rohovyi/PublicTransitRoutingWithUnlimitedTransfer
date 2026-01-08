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

public:
    struct ArrivalLabel : public ExternalKHeapElement {
        ArrivalLabel() : arrivalTime(never), tripCount(0), parent(noVertex),
                         transferParent(noStop), routeParent(noStop) {}

        int arrivalTime;
        uint8_t tripCount;
        Vertex parent;
        StopId transferParent;
        StopId routeParent;

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
    ShortcutSearchJTS(const Intermediate::Data& data, const TimeDependentGraph& tdGraph,
                      DynamicTransferGraph& shortcutGraph, const int witnessTransferLimit) :
        data(data),
        graph(tdGraph),
        shortcutGraph(shortcutGraph),
        numberOfStops(data.numberOfStops()),
        stationOfStop(numberOfStops),
        sourceStation(),
        sourceDepartureTime(0),
        witnessTransferLimit(witnessTransferLimit),
        earliestDepartureTime(computeMinDepartureTime(data)),
        optimalCandidates(0),
        timestamp(0) {

        buildStations();
    }

    inline void run(const StopId source, const int minTime, const int maxTime) noexcept {
        if (source >= numberOfStops) return;
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
                }
            }
        }
    }

    inline size_t getNumberOfOptimalCandidates() const noexcept requires CountOptimalCandidates {
        return optimalCandidates;
    }

private:
    inline static int computeMinDepartureTime(const Intermediate::Data& data) noexcept {
        int minDep = never;
        for (const Intermediate::Trip& trip : data.trips) {
            if (!trip.stopEvents.empty()) {
                const int dep = trip.stopEvents.front().departureTime;
                if (dep < minDep) {
                    minDep = dep;
                }
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

    inline bool isStop(const Vertex v) const noexcept {
        return v < numberOfStops;
    }

    inline int getMinTransferTime(const StopId stop) const noexcept {
        if (stop >= data.stops.size()) return 0;
        return data.stops[stop].minTransferTime;
    }

    inline void setSource(const StopId source) noexcept {
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

    inline void runForDepartureTime(const DepartureLabel& label) noexcept {
        if constexpr (Debug) {
            std::cout << "   Running search for departure time: " << label.departureTime
                      << " (" << String::secToTime(label.departureTime) << ")" << std::endl;
        }

        timestamp++;
        shortcuts.clear();
        sourceDepartureTime = label.departureTime;
        runTimeDependentSearch();
    }

    inline std::vector<DepartureLabel> collectDepartures(const int minTime, const int maxTime) noexcept {
        const int cutoffTime = std::max(minTime, earliestDepartureTime);
        std::set<int> departureTimes;

        for (const StopId stop : stopsReachedByDirectTransfer) {
            const int transferTime = directTransferDistance[stop];

            if (stationOfStop[stop].representative != sourceStation.representative) continue;

            for (const Edge e : graph.edgesFrom(stop)) {
                const auto& handle = graph.get(Function, e);
                if (handle.tripCount == 0) continue;

                const auto* begin = graph.getTripsBegin(handle);
                const auto* end = graph.getTripsEnd(handle);

                for (auto it = begin; it != end; ++it) {
                    const int adjustedDep = it->departureTime + transferTime;
                    if (adjustedDep < cutoffTime) continue;
                    if (adjustedDep > maxTime) break;
                    departureTimes.insert(adjustedDep);
                }
            }
        }

        std::vector<DepartureLabel> result;
        result.reserve(departureTimes.size());
        for (const int depTime : departureTimes) {
            result.emplace_back(depTime);
        }

        return result;
    }

    inline void clear() noexcept {
        sourceStation = Station();

        std::vector<int>(graph.numVertices(), never).swap(directTransferDistance);
        stopsReachedByDirectTransfer.clear();

        std::vector<ArrivalLabel>(graph.numVertices()).swap(labels);
        std::vector<uint16_t>(graph.numVertices(), 0).swap(timestamps);

        queue.clear();
        shortcuts.clear();
    }

    inline void initialDijkstra() noexcept {
        struct SimpleDijkstraLabel : public ExternalKHeapElement {
            int distance = never;
            inline bool hasSmallerKey(const SimpleDijkstraLabel* const other) const noexcept {
                return distance < other->distance;
            }
        };

        std::vector<SimpleDijkstraLabel> dijkstraLabels(graph.numVertices());
        ExternalKHeap<2, SimpleDijkstraLabel> dijkstraQueue;

        dijkstraLabels[sourceStation.representative].distance = 0;
        directTransferDistance[sourceStation.representative] = 0;
        dijkstraQueue.update(&dijkstraLabels[sourceStation.representative]);

        while (!dijkstraQueue.empty()) {
            SimpleDijkstraLabel* current = dijkstraQueue.extractFront();
            const Vertex u = Vertex(current - &dijkstraLabels[0]);

            for (const Edge edge : graph.edgesFrom(u)) {
                const Vertex v = graph.get(ToVertex, edge);
                const int walkTime = graph.getWalkArrivalFrom(edge, 0);
                if (walkTime == never) continue;

                const int newDist = current->distance + walkTime;

                if (newDist < dijkstraLabels[v].distance) {
                    dijkstraLabels[v].distance = newDist;
                    directTransferDistance[v] = newDist;
                    dijkstraQueue.update(&dijkstraLabels[v]);
                }
            }

            if (isStop(u)) {
                stopsReachedByDirectTransfer.emplace_back(StopId(u));
            }
        }
    }

    inline void runTimeDependentSearch() noexcept {
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
        int candidatesInQueue = 0;

        while (!queue.empty()) {
            ArrivalLabel* current = queue.extractFront();
            const Vertex u = Vertex(current - &labels[0]);
            const int currentTime = current->arrivalTime;
            const uint8_t currentTrips = current->tripCount;

            if (candidatesInQueue == 0 && transferLimit != intMax && currentTime > transferLimit) {
                break;
            }

            if (currentTrips == 1 && isStop(current->transferParent)) {
                candidatesInQueue--;
            }

            if (currentTrips == 2 && isStop(u)) {
                const StopId routeParent = current->routeParent;
                const StopId transferParent = current->transferParent;

                if (isStop(routeParent) && isStop(transferParent) &&
                    transferParent != routeParent && !shortcutAlreadyExists(routeParent, transferParent)) {

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
                continue;
            }

            if (isStop(u) && currentTrips < 2) {
                for (const Edge e : graph.edgesFrom(u)) {
                    const auto& handle = graph.get(Function, e);
                    if (handle.tripCount == 0) continue;

                    const auto* begin = graph.getTripsBegin(handle);
                    const auto* end = graph.getTripsEnd(handle);

                    auto it = std::lower_bound(begin, end, currentTime,
                        [](const DiscreteTrip& trip, int time) { return trip.departureTime < time; });

                    if (it == end) continue;

                    const int tripId = it->tripId;
                    const uint16_t startIdx = it->departureStopIndex;

                    StopId newTransferParent = current->transferParent;
                    if (currentTrips == 0 &&
                        stationOfStop[u].representative == sourceStation.representative) {
                        newTransferParent = StopId(u);
                    }

                    scanTrip(tripId, startIdx + 1, it->arrivalTime, StopId(u),
                            currentTrips + 1, newTransferParent, candidatesInQueue);
                }
            }

            for (const Edge edge : graph.edgesFrom(u)) {
                const int walkTime = graph.getWalkArrivalFrom(edge, 0);
                if (walkTime == never) continue;

                const Vertex v = graph.get(ToVertex, edge);
                const int newTime = currentTime + walkTime;

                relaxWalking(v, newTime, currentTrips, current->transferParent, candidatesInQueue);
            }

            if (candidatesInQueue == 0 && transferLimit == intMax && currentTrips >= 1) {
                transferLimit = currentTime + witnessTransferLimit;
                if constexpr (Debug) {
                    std::cout << "   Transfer limit: "
                              << String::secToString(transferLimit - sourceDepartureTime) << std::endl;
                }
            }
        }

        queue.clear();
    }

    inline void scanTrip(const int tripId, const uint16_t startStopIndex, int currentArrival,
                         const StopId boardStop, const uint8_t tripCount,
                         const StopId transferParent, int& candidatesInQueue) noexcept {

        if (tripId < 0) return;

        uint32_t currentIdx = graph.getTripOffset(tripId) + startStopIndex;
        uint32_t endIdx = graph.getTripOffset(tripId + 1);

        for (uint32_t idx = currentIdx; idx < endIdx; ++idx) {
            const auto& leg = graph.getTripLeg(idx);
            const Vertex stopVertex = leg.stopId;

            StopId routeParent = (tripCount == 2) ? boardStop : noStop;

            ArrivalLabel& targetLabel = getLabel(stopVertex);

            bool dominated = (currentArrival > targetLabel.arrivalTime) ||
                            (currentArrival == targetLabel.arrivalTime && tripCount >= targetLabel.tripCount);

            if (!dominated) {
                if (targetLabel.tripCount == 1 && isStop(targetLabel.transferParent)) {
                    candidatesInQueue--;
                }
                if (tripCount == 1 && isStop(transferParent)) {
                    candidatesInQueue++;
                }

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

    inline void relaxWalking(const Vertex v, const int newTime, const uint8_t tripCount,
                             const StopId transferParent, int& candidatesInQueue) noexcept {
        ArrivalLabel& targetLabel = getLabel(v);

        bool dominated = (newTime > targetLabel.arrivalTime) ||
                        (newTime == targetLabel.arrivalTime && tripCount >= targetLabel.tripCount);

        if (!dominated) {
            if (targetLabel.tripCount == 1 && isStop(targetLabel.transferParent)) {
                candidatesInQueue--;
            }
            if (tripCount == 1 && isStop(transferParent)) {
                candidatesInQueue++;
            }

            targetLabel.arrivalTime = newTime;
            targetLabel.tripCount = tripCount;
            targetLabel.parent = noVertex;
            targetLabel.transferParent = transferParent;
            targetLabel.routeParent = noStop;
            queue.update(&targetLabel);
        }
    }

    inline bool shortcutAlreadyExists(const StopId routeParent, const StopId transferParent) const noexcept {
        if constexpr (!CountOptimalCandidates) {
            return shortcutGraph.hasEdge(transferParent, routeParent);
        } else {
            return false;
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
    const Intermediate::Data& data;
    const TimeDependentGraph& graph;
    DynamicTransferGraph& shortcutGraph;

    const size_t numberOfStops;
    std::vector<Station> stationOfStop;
    Station sourceStation;
    int sourceDepartureTime;

    std::vector<int> directTransferDistance;
    std::vector<StopId> stopsReachedByDirectTransfer;

    std::vector<ArrivalLabel> labels;
    std::vector<uint16_t> timestamps;
    ExternalKHeap<2, ArrivalLabel> queue;

    std::vector<Shortcut> shortcuts;

    int witnessTransferLimit;
    int earliestDepartureTime;
    size_t optimalCandidates;
    uint16_t timestamp;
};

}