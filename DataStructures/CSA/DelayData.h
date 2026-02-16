#pragma once

#include <vector>
#include <iostream>
#include <iomanip>
#include <utility>

#include "../RAPTOR/Data.h"
#include "../TripBased/Data.h"
#include "../TripBased/DelayData.h"
#include "../TripBased/Shortcut.h"
#include "../Graph/Graph.h"

#include "../../Helpers/String/String.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Vector/Vector.h"

namespace CSA {

/**
 * CSA::DelayData
 *
 * Holds the delay-tolerant shortcut graph produced by DelayCSABuilder,
 * along with per-stop-event delay arrays and buffer parameters.
 *
 * The shortcut graph uses the same DynamicDelayGraph / DelayGraph format
 * as TripBased::DelayData (edges carry TravelTime, MinOriginDelay,
 * MaxOriginDelay). This makes it compatible with the existing
 * TripBased update/query pipeline — you can inject the shortcut graph
 * into a TripBased::DelayData via Graph::move() or toTripBasedDelayData().
 *
 * Responsibilities:
 *   - Store the CSA-computed delay shortcut graph
 *   - Track per-event arrival/departure delays
 *   - Filter shortcuts based on current delays (for query construction)
 *   - Convert to TripBased::DelayData for use with existing update/query code
 *   - Serialize/deserialize for persistence
 */
class DelayData {

public:
    // ─── Constructors ───────────────────────────────────────────────────

    /**
     * Construct from RAPTOR data and delay buffer parameters.
     * The shortcut graph is initially empty — populate it via
     * Graph::move() from a DelayCSABuilder result.
     */
    DelayData(const RAPTOR::Data& raptorData,
              const int arrivalDelayBuffer,
              const int departureDelayBuffer) :
        tripData(raptorData),
        arrivalDelayBuffer(arrivalDelayBuffer),
        departureDelayBuffer(departureDelayBuffer),
        arrivalDelay(tripData.numberOfStopEvents(), 0),
        departureDelay(tripData.numberOfStopEvents(), 0) {
    }

    /**
     * Deserialize from file.
     */
    DelayData(const std::string& fileName) {
        deserialize(fileName);
        arrivalDelay.resize(tripData.numberOfStopEvents(), 0);
        departureDelay.resize(tripData.numberOfStopEvents(), 0);
    }

    DelayData() :
        arrivalDelayBuffer(0),
        departureDelayBuffer(0) {
    }

    // ─── Conversion to TripBased::DelayData ─────────────────────────────

    /**
     * Create a TripBased::DelayData from this CSA delay data.
     *
     * This is the canonical way to bridge CSA-computed shortcuts into
     * the existing TripBased update/query pipeline. The method:
     *   1. Constructs TripBased::DelayData from the internal raptorData
     *   2. Moves the shortcut graph into it
     *   3. Copies delay arrays if they are non-zero
     *
     * After calling this method, the stopEventGraph of this CSA::DelayData
     * is moved (emptied). Use the returned TripBased::DelayData for all
     * subsequent update/query operations.
     */
    inline TripBased::DelayData toTripBasedDelayData() noexcept {
        TripBased::DelayData result(tripData.raptorData,
                                    arrivalDelayBuffer,
                                    departureDelayBuffer);
        Graph::move(std::move(stopEventGraph), result.stopEventGraph);
        result.arrivalDelay = arrivalDelay;
        result.departureDelay = departureDelay;
        return result;
    }

    // ─── Accessors ──────────────────────────────────────────────────────

    inline size_t numberOfStopEvents() const noexcept {
        return tripData.numberOfStopEvents();
    }

    inline size_t numberOfStops() const noexcept {
        return tripData.numberOfStops();
    }

    inline int delayedArrivalTime(const StopEventId stopEvent) const noexcept {
        return tripData.arrivalTime(stopEvent) + arrivalDelay[stopEvent];
    }

    inline int delayedDepartureTime(const StopEventId stopEvent) const noexcept {
        return tripData.departureTime(stopEvent) + departureDelay[stopEvent];
    }

    inline int getTransferSlack(const StopEventId from, const StopEventId to,
                                const int travelTime) const noexcept {
        return delayedDepartureTime(to) - delayedArrivalTime(from) - travelTime;
    }

    inline bool isTransferFeasible(const StopEventId from, const StopEventId to,
                                   const int travelTime) const noexcept {
        return getTransferSlack(from, to, travelTime) >= 0;
    }

    // ─── Shortcut filtering (mirrors TripBased::DelayData logic) ────────

    /**
     * Filter the shortcut graph based on current delays.
     * Returns (feasible shortcuts, infeasible shortcuts).
     *
     * A shortcut from→to is kept if:
     *   1. MinOriginDelay <= arrivalDelay[from]
     *   2. The transfer is temporally feasible under current delays
     *   3. MaxOriginDelay >= arrivalDelay[from] (or MaxOriginDelay >= arrivalDelayBuffer)
     */
    inline std::pair<TransferGraph, TransferGraph> filterShortcutGraph() const noexcept {
        TransferGraph result;
        TransferGraph infeasibleShortcuts;
        result.reserve(stopEventGraph.numVertices(), stopEventGraph.numEdges());
        infeasibleShortcuts.reserve(stopEventGraph.numVertices(), stopEventGraph.numEdges());

        for (const Vertex fromStopEvent : stopEventGraph.vertices()) {
            result.addVertex();
            infeasibleShortcuts.addVertex();
            for (const Edge shortcut : stopEventGraph.edgesFrom(fromStopEvent)) {
                if (stopEventGraph.get(MinOriginDelay, shortcut) > arrivalDelay[fromStopEvent])
                    continue;

                const Vertex toStopEvent = stopEventGraph.get(ToVertex, shortcut);
                const int travelTime = stopEventGraph.get(TravelTime, shortcut);

                if (!isTransferFeasible(StopEventId(fromStopEvent),
                                        StopEventId(toStopEvent), travelTime)) {
                    infeasibleShortcuts.addEdge(fromStopEvent, toStopEvent,
                                                stopEventGraph.edgeRecord(shortcut));
                } else {
                    const int maxOriginDelay = stopEventGraph.get(MaxOriginDelay, shortcut);
                    if (maxOriginDelay < arrivalDelayBuffer &&
                        maxOriginDelay < arrivalDelay[fromStopEvent])
                        continue;
                    result.addEdge(fromStopEvent, toStopEvent,
                                   stopEventGraph.edgeRecord(shortcut));
                }
            }
        }
        return std::make_pair(result, infeasibleShortcuts);
    }

    // ─── Delay application ──────────────────────────────────────────────

    inline void applyDelays(const std::vector<TripBased::DelayIncident>& incidents,
                            const bool ignoreMaxDelay,
                            const bool allowDepartureDelays,
                            const bool allowOvertaking = true) noexcept {
        for (const auto& incident : incidents) {
            delayTripFromEvent(incident.trip, incident.startIndex,
                               incident.delay, allowOvertaking, ignoreMaxDelay);
        }
        if (!ignoreMaxDelay) validateDelays();
        if (!allowDepartureDelays) clearDepartureDelays();
    }

    inline void applyDelayUpdates(const std::vector<TripBased::DelayUpdate>& updates,
                                  const bool allowDepartureDelays) noexcept {
        for (const auto& update : updates) {
            StopEventId event = update.firstEvent;
            for (size_t i = 0; i < update.arrivalDelay.size(); i++, event++) {
                arrivalDelay[event] = update.arrivalDelay[i];
                if (allowDepartureDelays)
                    departureDelay[event] = update.departureDelay[i];
            }
        }
    }

    inline void clearDepartureDelays() noexcept {
        Vector::fill(departureDelay, 0);
    }

    inline void validateDelays() noexcept {
        for (size_t i = 0; i < arrivalDelay.size(); i++) {
            Ensure(arrivalDelay[i] >= 0, "Arrival delay of stop event " << i << " is negative!");
            Ensure(arrivalDelay[i] <= arrivalDelayBuffer,
                   "Arrival delay of stop event " << i << " exceeds max delay!");
            Ensure(departureDelay[i] >= 0,
                   "Departure delay of stop event " << i << " is negative!");
            Ensure(departureDelay[i] <= departureDelayBuffer,
                   "Departure delay of stop event " << i << " exceeds max delay!");
        }
    }

    // ─── Info / IO ──────────────────────────────────────────────────────

    inline void printInfo() const noexcept {
        std::cout << "CSA Delay Data:" << std::endl;
        std::cout << "   Number of Stops:          " << std::setw(12)
                  << String::prettyInt(numberOfStops()) << std::endl;
        std::cout << "   Number of Stop Events:    " << std::setw(12)
                  << String::prettyInt(numberOfStopEvents()) << std::endl;
        std::cout << "   Number of Shortcuts:      " << std::setw(12)
                  << String::prettyInt(stopEventGraph.numEdges()) << std::endl;
        std::cout << "   Arrival Delay Buffer:     " << std::setw(12)
                  << arrivalDelayBuffer << std::endl;
        std::cout << "   Departure Delay Buffer:   " << std::setw(12)
                  << departureDelayBuffer << std::endl;
    }

    inline void serialize(const std::string& fileName) const noexcept {
        tripData.serialize(fileName + ".tripBased");
        stopEventGraph.writeBinary(fileName + ".delayGraph");
        IO::serialize(fileName, arrivalDelayBuffer, departureDelayBuffer);
    }

    inline void deserialize(const std::string& fileName) noexcept {
        tripData.deserialize(fileName + ".tripBased");
        stopEventGraph.readBinary(fileName + ".delayGraph");
        IO::deserialize(fileName, arrivalDelayBuffer, departureDelayBuffer);
    }

private:
    inline void delayTripFromEvent(const TripId trip, const StopIndex index,
                                   const int delay, const bool allowOvertaking,
                                   const bool ignoreMaxDelay) noexcept {
        const StopEventId startingEvent =
            StopEventId(tripData.firstStopEventOfTrip[trip] + index);
        const bool hasNextTrip =
            (tripData.routeOfTrip[trip] == tripData.routeOfTrip[trip + 1]);

        for (StopEventId ev = startingEvent;
             ev < tripData.firstStopEventOfTrip[trip + 1]; ev++) {
            arrivalDelay[ev] += delay;
            departureDelay[ev] += delay;
            if (!ignoreMaxDelay) {
                arrivalDelay[ev] = std::min(arrivalDelayBuffer, arrivalDelay[ev]);
                departureDelay[ev] = std::min(departureDelayBuffer, departureDelay[ev]);
            }
            if (!allowOvertaking && hasNextTrip) {
                const StopEventId nextEv =
                    StopEventId(ev + tripData.numberOfStopsInTrip(trip));
                arrivalDelay[ev] = std::min(arrivalDelay[ev],
                    delayedArrivalTime(nextEv) - tripData.arrivalTime(ev));
                departureDelay[ev] = std::min(departureDelay[ev],
                    delayedDepartureTime(nextEv) - tripData.departureTime(ev));
            }
        }

        const StopEventId lastEv(tripData.firstStopEventOfTrip[trip + 1] - 1);
        const int realDepTime = tripData.departureTime(lastEv) +
            tripData.raptorData.minTransferTime(tripData.getStopOfStopEvent(lastEv));
        arrivalDelay[lastEv] = std::min(arrivalDelay[lastEv],
            realDepTime + departureDelay[lastEv] - tripData.arrivalTime(lastEv));

        for (StopEventId nextEv = lastEv; nextEv != startingEvent; nextEv--) {
            const StopEventId curEv = StopEventId(nextEv - 1);
            const int rdTime = tripData.departureTime(curEv) +
                tripData.raptorData.minTransferTime(tripData.getStopOfStopEvent(curEv));
            departureDelay[curEv] = std::min(departureDelay[curEv],
                delayedArrivalTime(nextEv) - rdTime);
            arrivalDelay[curEv] = std::min(arrivalDelay[curEv],
                rdTime + departureDelay[curEv] - tripData.arrivalTime(curEv));
        }
    }

public:
    TripBased::Data tripData;
    DelayGraph stopEventGraph;
    int arrivalDelayBuffer;
    int departureDelayBuffer;
    std::vector<int> arrivalDelay;
    std::vector<int> departureDelay;
};

} // namespace CSA